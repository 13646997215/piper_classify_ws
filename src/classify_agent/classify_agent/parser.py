"""Language-to-command parsing for the Piper demo.

The LLM is only asked for a small JSON object.  Every field is validated here
before it can reach the robot task.  A deterministic parser keeps the demo
usable when no LLM key or network is available.
"""
from dataclasses import asdict, dataclass, field
import re
from typing import Any


INTENTS = ("pick_place", "pick_hold", "home", "stop", "scan", "count", "list", "reset", "unknown")
SELECTORS = ("nearest", "leftmost", "rightmost", "first")


@dataclass
class CommandSpec:
    intent: str = "unknown"
    colors: list[str] = field(default_factory=list)
    quantity: int = 0          # 0 = all matching objects
    target_bin: str = ""       # empty = same colour as the object
    selector: str = "nearest"
    reply: str = ""
    source: str = "rule"       # rule | llm

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def normalize_color(value: str) -> str:
    text = str(value).strip().lower()
    if text in ("red", "r", "红", "红色", "红块", "红方块"):
        return "red"
    if text in ("blue", "b", "蓝", "蓝色", "蓝块", "蓝方块"):
        return "blue"
    if text in ("green", "g", "绿", "绿色", "绿块", "绿方块"):
        return "green"
    return ""


def validate_spec(data: dict[str, Any], source: str = "llm") -> CommandSpec:
    """Validate and normalize an LLM-produced object.

    Raises ValueError with a user-readable reason.  The caller can then fall
    back to the deterministic parser or ask a clarification question.
    """
    if not isinstance(data, dict):
        raise ValueError("LLM response is not a JSON object")
    intent = str(data.get("intent", "unknown")).strip().lower()
    if intent not in INTENTS:
        raise ValueError(f"unknown intent: {intent}")

    colors: list[str] = []
    raw_colors = data.get("colors", [])
    if isinstance(raw_colors, str):
        raw_colors = [raw_colors]
    if not isinstance(raw_colors, (list, tuple)):
        raise ValueError("colors must be a list")
    for item in raw_colors:
        color = normalize_color(str(item))
        if color == "green":
            raise ValueError("绿色是参考物，不能搬运")
        if color and color not in colors:
            colors.append(color)

    try:
        quantity = int(data.get("quantity", 0))
    except (TypeError, ValueError):
        raise ValueError("quantity must be an integer") from None
    if quantity < 0:
        raise ValueError("quantity must be 0 or positive")
    quantity = min(quantity, 4)

    target_bin = str(data.get("target_bin", "") or "").strip().lower()
    target_bin = normalize_color(target_bin) if target_bin else ""
    if target_bin == "green":
        raise ValueError("绿色料盒不存在")
    if target_bin not in ("", "red", "blue"):
        raise ValueError("target_bin must be red, blue or empty")

    selector = str(data.get("selector", "nearest") or "nearest").strip().lower()
    if selector not in SELECTORS:
        selector = "nearest"

    reply = str(data.get("reply", "") or "")

    if intent in ("pick_place", "pick_hold") and not colors:
        raise ValueError("pick commands need a red or blue target colour")

    return CommandSpec(intent=intent, colors=colors, quantity=quantity,
                       target_bin=target_bin, selector=selector,
                       reply=reply, source=source)


def _colors(text: str) -> list[str]:
    result: list[str] = []
    if re.search(r"红|red", text):
        result.append("red")
    if re.search(r"蓝|blue", text):
        result.append("blue")
    return result


def _quantity(text: str) -> int:
    match = re.search(r"(\d+)\s*(?:个|块|件|只)?", text)
    if match and int(match.group(1)) > 0:
        return min(int(match.group(1)), 4)
    chinese = {"一": 1, "两": 2, "二": 2, "三": 3, "四": 4,
               "五": 5, "六": 6, "七": 7, "八": 8}
    for character, value in chinese.items():
        if character in text:
            return min(value, 4)
    if re.search(r"所有|全部|都|all|both|every", text):
        return 0
    return 1


def _selector(text: str) -> str:
    if re.search(r"最左|左边|左侧|left", text):
        return "leftmost"
    if re.search(r"最右|右边|右侧|right", text):
        return "rightmost"
    if re.search(r"第一个|first", text):
        return "first"
    if re.search(r"最近|nearest|closest", text):
        return "nearest"
    return "nearest"


def _target_bin(text: str) -> str:
    if re.search(r"(红|red)\s*(?:色)?\s*(?:盒|料盒|盒子|bin|box)", text):
        return "red"
    if re.search(r"(蓝|blue)\s*(?:色)?\s*(?:盒|料盒|盒子|bin|box)", text):
        return "blue"
    return ""


def _without_bin_phrases(text: str) -> str:
    return re.sub(r"(红|蓝|red|blue)\s*(?:色)?\s*(?:盒|料盒|盒子|bin|box)", " ", text)


def fallback_parse(text: str) -> CommandSpec:
    """Rule-based parser for common Chinese/English demo commands."""
    raw = text.strip()
    if not raw:
        return CommandSpec(intent="unknown", reply="没有听到指令")

    lower = raw.lower()
    if re.search(r"重置|重新开始|重新分拣|reset", lower):
        return CommandSpec(intent="reset", reply="重置需要停止并重新启动仿真")
    if re.search(r"停止|停下|停下来|取消|别动|stop|cancel|halt", lower):
        return CommandSpec(intent="stop")
    if re.search(r"回家|回\s*home|go\s*home|复位|恢复姿态|home", lower):
        return CommandSpec(intent="home")
    if re.search(r"几个|多少|数量|数一下|count|how\s+many", lower):
        return CommandSpec(intent="count", colors=_colors(_without_bin_phrases(lower)))
    if re.search(r"扫描|看看|有什么|有哪些|场景|桌面|list|scan|what.*see", lower):
        return CommandSpec(intent="scan")

    target_bin = _target_bin(lower)
    object_text = _without_bin_phrases(lower)
    if re.search(r"绿|green", object_text):
        return CommandSpec(intent="unknown", reply="绿色是参考物，不能搬运")

    colors = _colors(object_text)
    placing = bool(re.search(r"放|放入|放到|放进|搬到|移到|place|put|move.*to", object_text))
    picking = bool(re.search(r"拿|抓|捡|夹|pick|grab|hold|lift|take", object_text))
    if not placing and not picking:
        return CommandSpec(intent="unknown", reply="请说要拿哪个颜色，或者放到哪个盒子")
    if not colors:
        return CommandSpec(intent="unknown", reply="请说明目标颜色，例如红色或蓝色")

    intent = "pick_place" if placing else "pick_hold"
    return CommandSpec(intent=intent, colors=colors, quantity=_quantity(object_text),
                       target_bin=target_bin, selector=_selector(object_text))


SYSTEM_PROMPT = """你是一个松灵 Piper 机械臂的自然语言任务解析器。
只输出一个 JSON 对象，不要输出 Markdown、解释或代码块。

可用 intent:
- pick_place: 抓起工件并放到料盒
- pick_hold: 抓起工件并保持在桌面 above（不放盒）
- home: 机械臂回到观察姿态
- stop: 停止/取消当前任务
- scan: 检查视觉场景是否就绪
- count: 查询可见工件数量
- list: 列出可见工件
- reset: 重置仿真（需要外部脚本）
- unknown: 无法理解或需要澄清

JSON schema:
{
  "intent": "pick_place | pick_hold | home | stop | scan | count | list | reset | unknown",
  "colors": ["red", "blue"],
  "quantity": 0,
  "target_bin": "red | blue | \"\"",
  "selector": "nearest | leftmost | rightmost | first",
  "reply": "需要澄清时给用户的一句话"
}

规则:
1. 只有红色和蓝色工件可以被搬运；绿色是参考物，永远不能抓。
2. target_bin 为空表示放进与工件同色的料盒。
3. quantity=0 表示所有匹配工件；默认 1。
4. selector 默认 nearest；leftmost/rightmost 以机械臂 base_link 的 +Y/-Y 方向解释。
5. 不要输出关节角、坐标、速度或任何底层运动指令。
6. 用户说“拿起/抓/拿”但没有说放盒时用 pick_hold；说了“放到/放进/搬到”时用 pick_place。
7. 用户没有指定颜色时，用 unknown 并在 reply 里询问颜色。
8. 只输出 JSON 对象本身。

示例:
用户: 把红色方块放到蓝色盒子里
输出: {"intent":"pick_place","colors":["red"],"quantity":1,"target_bin":"blue","selector":"nearest","reply":""}

用户: 拿一个蓝色方块
输出: {"intent":"pick_hold","colors":["blue"],"quantity":1,"target_bin":"","selector":"nearest","reply":""}

用户: 桌上有几个红块
输出: {"intent":"count","colors":["red"],"quantity":0,"target_bin":"","selector":"nearest","reply":""}

用户: 把两个红块都放进红盒
输出: {"intent":"pick_place","colors":["red"],"quantity":2,"target_bin":"red","selector":"nearest","reply":""}

用户: 回家
输出: {"intent":"home","colors":[],"quantity":0,"target_bin":"","selector":"nearest","reply":""}

用户: 停止
输出: {"intent":"stop","colors":[],"quantity":0,"target_bin":"","selector":"nearest","reply":""}
"""
