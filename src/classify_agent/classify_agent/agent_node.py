#!/usr/bin/env python3
"""Language front-end for the Piper classification demo.

Flow:
    text / speech
      -> LLM (or deterministic fallback)
      -> validated CommandSpec(JSON)
      -> /classify/command (RobotCommand service)
      -> existing perception + MTC task
      -> /classify/status /classify/result monitored here

The LLM never sees or emits joint angles.  The C++ task keeps all motion,
collision and grasp checks.
"""
import json
import os
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from classify_interfaces.msg import SceneStatus, TaskResult, TaskStatus
from classify_interfaces.srv import RobotCommand

from classify_agent.llm import LLMClient
from classify_agent.parser import CommandSpec, fallback_parse, validate_spec

TERMINAL_STATES = ("DONE", "FAILED", "CANCELLED", "HOLDING")


class LanguageAgent(Node):
    def __init__(self) -> None:
        super().__init__('classify_language_agent')
        self.declare_parameter('use_llm', False)
        self.declare_parameter('llm_model', 'deepseek-chat')
        self.declare_parameter('llm_base_url', 'https://api.deepseek.com')
        self.declare_parameter('llm_api_key_env', 'DEEPSEEK_API_KEY')
        self.declare_parameter('use_voice', False)
        self.declare_parameter('voice_model',
                               os.path.expanduser('~/models/faster-whisper-medium'))
        self.declare_parameter('voice_language', 'zh')
        self.declare_parameter('enable_tts', False)
        self.declare_parameter('result_timeout', 300.0)
        self.declare_parameter('command', '')

        self.use_llm = bool(self.get_parameter('use_llm').value)
        self.use_voice = bool(self.get_parameter('use_voice').value)
        self.enable_tts = bool(self.get_parameter('enable_tts').value)
        self.voice_model = str(self.get_parameter('voice_model').value)
        self.voice_language = str(self.get_parameter('voice_language').value)
        self.result_timeout = float(self.get_parameter('result_timeout').value)
        self.command_text = str(self.get_parameter('command').value or '').strip()
        api_key_env = str(self.get_parameter('llm_api_key_env').value)
        self.api_key = os.environ.get(api_key_env, '')
        self.llm = LLMClient(
            self.api_key,
            str(self.get_parameter('llm_base_url').value),
            str(self.get_parameter('llm_model').value))
        if self.use_llm and not self.llm.enabled:
            self.get_logger().warn(
                f'{api_key_env} is not set; using the deterministic parser. '
                f'Export it to enable LLM intent parsing.')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SceneStatus, '/classify/scene_status',
                                 self._on_scene, qos)
        self.create_subscription(TaskStatus, '/classify/status',
                                 self._on_status, qos)
        self.create_subscription(TaskResult, '/classify/result',
                                 self._on_result, qos)
        self.text_pub = self.create_publisher(String, '/classify/command_text', 10)
        self.spec_pub = self.create_publisher(String, '/classify/task_spec', qos)
        self.command_client = self.create_client(RobotCommand, '/classify/command')
        self.cancel_client = self.create_client(Trigger, '/classify/cancel')
        self.ready_client = self.create_client(Trigger, '/classify/scene_ready')

        self.last_scene = None
        self.last_status = None
        self.last_result = None
        self._scene_wall = 0.0
        self._status_count = 0
        self._result_count = 0
        self._last_status_key = None
        self._whisper = None
        self.get_logger().info(
            'Language agent ready. Type a command, type "voice" to record, '
            'or type "quit" to exit.')

    # ------------------------------------------------------------------ input
    def interactive_loop(self) -> None:
        print('\n可用示例:')
        print('  把红色方块放到蓝色盒子里')
        print('  拿一个蓝色方块')
        print('  桌上有几个红块')
        print('  把左边的红块放进红盒')
        print('  回家 / 停止 / 扫描')
        print('输入 voice 录音 4 秒，输入 quit 退出。\n')
        while rclpy.ok():
            try:
                text = input('你 > ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not text:
                continue
            if text.lower() in ('quit', 'exit', 'q'):
                break
            if text.lower() == 'voice':
                if not self.use_voice:
                    self._say('请加 --ros-args -p use_voice:=true 后启用语音输入')
                    continue
                text = self._listen()
                if not text:
                    continue
            self.handle_text(text)

    def handle_text(self, text: str) -> None:
        self.text_pub.publish(String(data=text))
        print(f'\n你说: {text}')
        spec = None
        if self.use_llm and self.llm.enabled:
            raw = self.llm.parse(text)
            if raw and '_error' not in raw:
                try:
                    spec = validate_spec(raw, source='llm')
                    print(f'[llm] {json.dumps(raw, ensure_ascii=False)}')
                except ValueError as error:
                    print(f'[llm] 输出无效: {error}; 使用规则解析')
            elif raw and '_error' in raw:
                print(f"[llm] {raw['_error']}; 使用规则解析")
        if spec is None:
            spec = fallback_parse(text)
        payload = json.dumps(spec.to_dict(), ensure_ascii=False)
        print(f'[task] {payload}')
        self.spec_pub.publish(String(data=payload))
        self._dispatch(spec)

    # --------------------------------------------------------------- dispatch
    def _dispatch(self, spec: CommandSpec) -> None:
        if spec.intent == 'unknown':
            self._say(spec.reply or '请说明要操作的颜色，以及要放到哪个盒子')
        elif spec.intent == 'stop':
            response = self._call_trigger(self.cancel_client, 3.0)
            if response is None:
                self._say('/classify/cancel 服务不可用')
            else:
                self._say(response.message if response.message else '已请求停止')
        elif spec.intent == 'reset':
            self._say('请运行: bash scripts/reset_classify.sh')
        elif spec.intent == 'count':
            self._count(spec)
        elif spec.intent in ('scan', 'list'):
            self._scan()
        elif spec.intent == 'home':
            self._send_command(spec)
        elif spec.intent in ('pick_place', 'pick_hold'):
            self._send_command(spec)
        else:
            self._say(f'暂不支持的任务: {spec.intent}')

    # ---------------------------------------------------------------- commands
    def _send_command(self, spec: CommandSpec) -> None:
        if not self.command_client.wait_for_service(timeout_sec=5.0):
            self._say('/classify/command 服务不可用，请先启动完整仿真')
            return
        request = RobotCommand.Request()
        request.intent = spec.intent
        request.colors = spec.colors
        request.quantity = int(spec.quantity)
        request.target_bin = spec.target_bin
        request.selector = spec.selector

        status_before = self._status_count
        result_before = self._result_count
        future = self.command_client.call_async(request)
        if not self._wait_future(future, 10.0):
            self._say('等待 /classify/command 响应超时')
            return
        response = future.result()
        print(f'[robot] accepted={response.accepted} {response.message}')
        if not response.accepted:
            self._say(response.message)
            return

        status = self._wait_terminal(status_before, self.result_timeout)
        if status is None:
            self._say('任务超时，未收到终态反馈')
            return
        print(f'[result] state={status.state} detail={status.detail}')
        if self._result_count > result_before and self.last_result is not None:
            result = self.last_result
            print(f'[result] success={result.success} '
                  f'succeeded={result.succeeded} failed={result.failed} '
                  f'elapsed={result.elapsed_sec:.1f}s')
            if result.objects:
                print('[result] objects: ' + ', '.join(
                    f'{obj}->{bin}' for obj, bin in zip(result.objects, result.bins)))
        if status.state == 'HOLDING':
            self._say('已抓起工件并保持在桌面上方；再次操作前请重置仿真')
        elif status.state == 'DONE':
            self._say('任务完成')
        elif status.state == 'CANCELLED':
            self._say('任务已取消；夹爪保持当前位置，重跑前请重置')
        else:
            self._say(f'任务失败: {status.detail}')

    # --------------------------------------------------------------- queries
    def _scene_snapshot(self) -> SceneStatus | None:
        if self.last_scene is None or time.monotonic() - self._scene_wall > 2.0:
            self._call_trigger(self.ready_client, 3.0)
            time.sleep(0.5)
        if self.last_scene is None or time.monotonic() - self._scene_wall > 2.0:
            return None
        return self.last_scene

    def _count(self, spec: CommandSpec) -> None:
        scene = self._scene_snapshot()
        if scene is None:
            self._say('还没有收到场景状态，请确认视觉节点已启动')
            return
        colors = dict(zip(scene.object_ids, scene.object_colors))
        counts: dict[str, int] = {}
        for object_id in scene.fresh_objects:
            color = colors.get(object_id, 'unknown')
            counts[color] = counts.get(color, 0) + 1
        if spec.colors:
            text = ', '.join(f'{color} {counts.get(color, 0)} 个' for color in spec.colors)
        else:
            text = ', '.join(f'{color} {count} 个'
                             for color, count in sorted(counts.items()))
        self._say('当前可见: ' + (text or '没有新鲜工件'))

    def _scan(self) -> None:
        response = self._call_trigger(self.ready_client, 5.0)
        if response is None:
            self._say('/classify/scene_ready 服务不可用')
            return
        scene = self._scene_snapshot()
        state = '就绪' if response.success else '未就绪'
        detail = response.message or (scene.detail if scene else '')
        self._say(f'场景{state}: {detail}')
        if scene is not None:
            if scene.fresh_objects:
                print('[scene] fresh: ' + ', '.join(scene.fresh_objects))
            if scene.stale_objects:
                print('[scene] stale: ' + ', '.join(scene.stale_objects))

    # ----------------------------------------------------------------- helpers
    def _wait_future(self, future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def _wait_terminal(self, status_before: int, timeout: float) -> TaskStatus | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.last_status
            if (self._status_count > status_before and status is not None
                    and status.state in TERMINAL_STATES):
                return status
            time.sleep(0.1)
        return None

    def _call_trigger(self, client, timeout: float):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout):
            return None
        try:
            return future.result()
        except Exception:
            return None

    # ---------------------------------------------------------------- callbacks
    def _on_scene(self, message: SceneStatus) -> None:
        self.last_scene = message
        self._scene_wall = time.monotonic()

    def _on_status(self, message: TaskStatus) -> None:
        self._status_count += 1
        self.last_status = message
        key = (message.state, message.stage, message.completed,
               message.total, message.current_object, message.held_object)
        if key != self._last_status_key:
            self._last_status_key = key
            suffix = f' {message.current_object}->{message.target_bin}' if message.current_object else ''
            held = f' held={message.held_object}' if message.held_object else ''
            print(f'[status] {message.state} {message.stage} '
                  f'{message.completed}/{message.total}{suffix}{held}')

    def _on_result(self, message: TaskResult) -> None:
        self._result_count += 1
        self.last_result = message

    # ---------------------------------------------------------------- voice/tts
    def _listen(self) -> str:
        try:
            import sounddevice as sd
            from faster_whisper import WhisperModel
        except ImportError as error:
            self._say(f'语音依赖未安装: {error}. 可继续用文本输入')
            return ''
        try:
            if self._whisper is None:
                print(f'[asr] 加载模型 {self.voice_model} ...')
                self._whisper = WhisperModel(
                    self.voice_model, device='cpu', compute_type='int8')
            print('[asr] 录音 4 秒...')
            audio = sd.rec(int(4.0 * 16000), samplerate=16000,
                           channels=1, dtype='float32')
            sd.wait()
            segments, _ = self._whisper.transcribe(
                audio.flatten(), language=self.voice_language, beam_size=1,
                vad_filter=True,
                initial_prompt='机械臂分拣指令：红色、蓝色、料盒、拿起、放下、回家、停止、扫描。')
            text = ''.join(segment.text for segment in segments).strip()
            print(f'[asr] {text}')
            return text
        except Exception as error:
            self._say(f'语音识别失败: {error}')
            return ''

    def _say(self, text: str) -> None:
        print(f'[agent] {text}')
        if not self.enable_tts:
            return
        for command in (['espeak-ng', '-v', 'zh', '-s', '150', text],
                        ['espeak', '-v', 'zh', text],
                        ['spd-say', text]):
            if shutil.which(command[0]):
                try:
                    subprocess.run(command, timeout=10,
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
                    return
                except Exception:
                    continue


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LanguageAgent()
    # Spin in a background thread so the stdin loop and scripted commands can
    # wait on services/futures without owning the executor.
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        if node.command_text:
            node.handle_text(node.command_text)
        else:
            node.interactive_loop()
    finally:
        rclpy.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
