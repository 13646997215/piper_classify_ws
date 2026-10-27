#!/usr/bin/env python3
# Copyright 2026 piper_classify_ws contributors
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Legacy voice front end for batch start, cancel and scene queries."""

import os
import threading
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        import sounddevice
        from faster_whisper import WhisperModel
        import openai

        self.audio = sounddevice

        # --- 参数 ---
        self.declare_parameter('language', 'zh')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('voice_model',
                               os.path.expanduser('~/models/faster-whisper-medium'))
        self.declare_parameter('api_key_env', 'DEEPSEEK_API_KEY')
        self.declare_parameter('api_base_url', 'https://api.deepseek.com')

        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.voice_model = str(self.get_parameter('voice_model').value)

        # --- 加载 Whisper Medium 模型 ---
        self.get_logger().info(f'正在加载 Whisper 模型: {self.voice_model} ...')
        self.model = WhisperModel(self.voice_model, device='cpu', compute_type='int8')
        self.get_logger().info('Whisper 模型加载完成')

        # --- DeepSeek API 配置（密钥只从环境变量读取，禁止写入仓库） ---
        api_key_env = str(self.get_parameter('api_key_env').value)
        self.deepseek_api_key = os.environ.get(api_key_env, '').strip()
        self.deepseek_client = None
        if self.deepseek_api_key:
            self.deepseek_client = openai.OpenAI(
                api_key=self.deepseek_api_key,
                base_url=str(self.get_parameter('api_base_url').value)
            )
        else:
            self.get_logger().warn(
                f'{api_key_env} 未设置，语音节点将无法使用大模型解析意图；'
                f'请先 export {api_key_env}=...')

        # --- 发布识别文本 ---
        self.text_pub = self.create_publisher(String, '/voice_text', 10)

        # --- 服务客户端 ---
        self.start_client = self.create_client(Trigger, '/classify/start')
        self.cancel_client = self.create_client(Trigger, '/classify/cancel')
        self.scene_ready_client = self.create_client(Trigger, '/classify/scene_ready')

        # --- 启动键盘监听线程 ---
        self.recording = True
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info('语音指令节点已就绪。按 [回车键] 开始说话...')

    def _keyboard_loop(self):
        """Record speech when Enter is pressed."""
        while self.recording and rclpy.ok():
            try:
                input("\n>>> 按 [回车键] 开始说话（录音4秒）...")
                if not rclpy.ok():
                    break
                self._record_and_transcribe()
            except EOFError:
                break

    def _record_and_transcribe(self):
        duration = 4.0
        self.get_logger().info(f'录音中（{duration} 秒）...')
        audio = self.audio.rec(
            int(duration * self.sample_rate), samplerate=self.sample_rate,
            channels=1, dtype='float32')
        self.audio.wait()
        self.get_logger().info('录音结束，正在识别...')
        self._transcribe_and_act(audio.flatten())

    def _transcribe_and_act(self, audio):
        try:
            segments, _ = self.model.transcribe(
                audio, language=self.language, beam_size=1, vad_filter=True,
                initial_prompt="机械臂分拣指令：开始分拣，启动，停止，取消，暂停，检查场景，状态，捡起红色方块，捡起绿色方块。"
            )
            text = ''.join(seg.text for seg in segments).strip()
        except Exception as e:
            self.get_logger().error(f'转写失败: {e}')
            return

        if not text:
            self.get_logger().warn('没有听到有效声音')
            return

        self.get_logger().info(f'识别到: "{text}"')
        self.text_pub.publish(String(data=text))

        # ----- 请求 DeepSeek 大模型解析意图 -----
        self.get_logger().info('正在请求 DeepSeek 理解意图...')
        actions = self._ask_deepseek(text)

        if not actions:
            self.get_logger().warn('大模型未能解析出有效指令')
            return

        self.get_logger().info(f'大模型解析出的动作序列: {actions}')
        self._execute_actions(actions)

    def _ask_deepseek(self, user_text):
        if self.deepseek_client is None:
            self.get_logger().error(
                '未配置 API Key，无法进行大模型意图解析；'
                '请 export DEEPSEEK_API_KEY=... 后重启节点')
            return None

        system_prompt = """你是一个 Piper 机械臂的智能语音助手。你需要将用户的自然语言拆解为按顺序执行的 JSON 动作数组。
目前系统支持的基础动作有：
- start_classify: 启动全部分拣任务（抓红蓝方块）
- cancel_classify: 取消或停止当前任务
- check_scene: 检查视觉场景是否就绪
- reset_simulation: 提示用户在外部终端运行 bash scripts/reset_classify.sh；不会在节点内重置，后续动作不执行

注意：
1. 当前系统不支持统计方块数量，如果用户问数量，请返回 {"action": "none", "reply": "抱歉，系统暂不支持统计数量，请查看视觉界面。"}
2. 如果用户要求抓取特定颜色，但系统暂无该精细功能，返回 {"action": "none", "reply": "当前系统暂不支持只抓取特定颜色"}。
3. 用户可能会表达得很口语化，或包含多个步骤，请严格只输出一个 JSON 数组（不要带markdown标记），按执行顺序排列。

例如：
用户说：暂停一下，然后看看现在桌面是什么情况
输出：[
  {"action": "cancel_classify"},
  {"action": "check_scene"}
]

用户说：重新开始分拣
输出：[
  {"action": "reset_simulation"},
  {"action": "start_classify"}
]
"""
        try:
            response = self.deepseek_client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_text}
                ],
                temperature=0.1,
                response_format={"type": "json_object"}  # 强制 JSON
            )
            content = response.choices[0].message.content.strip()
            # 如果返回的是单个对象，包成数组
            parsed = json.loads(content)
            if isinstance(parsed, dict):
                return [parsed]
            return parsed
        except Exception as e:
            self.get_logger().error(f'DeepSeek API 调用失败: {e}')
            return None

    def _execute_actions(self, actions):
        """Dispatch actions, stopping the queue when an external reset is needed."""
        for act in actions:
            action_name = act.get("action")
            if action_name == "reset_simulation":
                self.get_logger().warning(
                    '请在外部终端运行 bash scripts/reset_classify.sh；'
                    'Gazebo 世界重置不能重置任务状态，本次动作队列已停止')
                return
            if action_name == "start_classify":
                self.get_logger().info('执行: 启动分拣')
                self._call_trigger(self.start_client, '/classify/start')
                time.sleep(1.0)  # 间隔一下，防止同时调用服务
            elif action_name == "cancel_classify":
                self.get_logger().info('执行: 取消分拣')
                self._call_trigger(self.cancel_client, '/classify/cancel')
                time.sleep(1.0)
            elif action_name == "check_scene":
                self.get_logger().info('执行: 检查场景')
                self._call_trigger(self.scene_ready_client, '/classify/scene_ready')
                time.sleep(1.0)
            elif action_name == "none":
                self.get_logger().info(f"回复: {act.get('reply')}")

    def _call_trigger(self, client, name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'{name} 服务不可用')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_response(f, name))

    def _on_response(self, future, name):
        try:
            resp = future.result()
            status = '成功' if resp.success else f'失败: {resp.message}'
            self.get_logger().info(f'{name} -> {status}')
        except Exception as e:
            self.get_logger().error(f'{name} 调用异常: {e}')

    def destroy_node(self):
        self.recording = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
