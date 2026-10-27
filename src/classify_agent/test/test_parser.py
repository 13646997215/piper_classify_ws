from classify_agent.parser import fallback_parse, validate_spec


def test_place_red_into_blue_bin():
    spec = fallback_parse('把红色方块放到蓝色盒子里')
    assert spec.intent == 'pick_place'
    assert spec.colors == ['red']
    assert spec.target_bin == 'blue'
    assert spec.quantity == 1


def test_pick_blue_and_hold():
    spec = fallback_parse('拿一个蓝色方块')
    assert spec.intent == 'pick_hold'
    assert spec.colors == ['blue']
    assert spec.target_bin == ''
    assert spec.quantity == 1


def test_two_red_into_red_bin():
    spec = fallback_parse('把两个红块都放进红盒')
    assert spec.intent == 'pick_place'
    assert spec.colors == ['red']
    assert spec.target_bin == 'red'
    assert spec.quantity == 2


def test_selectors_and_counts():
    assert fallback_parse('拿左边的红块').selector == 'leftmost'
    assert fallback_parse('拿最近的红块').selector == 'nearest'
    assert fallback_parse('桌上有几个红块').intent == 'count'
    assert fallback_parse('回家').intent == 'home'
    assert fallback_parse('停止').intent == 'stop'


def test_green_is_rejected():
    spec = fallback_parse('把绿块放进红盒')
    assert spec.intent == 'unknown'
    assert '绿' in spec.reply


def test_missing_color_asks_for_clarification():
    spec = fallback_parse('把方块放进红盒')
    assert spec.intent == 'unknown'
    assert spec.reply


def test_validate_llm_output_normalizes_values():
    spec = validate_spec({
        'intent': 'pick_place',
        'colors': ['红'],
        'quantity': 1,
        'target_bin': '蓝',
        'selector': 'leftmost',
    })
    assert spec.colors == ['red']
    assert spec.target_bin == 'blue'
    assert spec.selector == 'leftmost'


def test_validate_rejects_green_and_joint_output():
    try:
        validate_spec({'intent': 'pick_place', 'colors': ['green']})
    except ValueError as error:
        assert '绿' in str(error)
    else:
        raise AssertionError('green must be rejected')
