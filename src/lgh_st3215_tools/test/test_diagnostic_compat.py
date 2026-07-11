from lgh_st3215_tools.diagnostic_compat import diagnostic_level_to_int


def test_humble_byte_representation():
    assert diagnostic_level_to_int(b'\x00') == 0
    assert diagnostic_level_to_int(b'\x01') == 1
    assert diagnostic_level_to_int(b'\x02') == 2


def test_integer_representation():
    assert diagnostic_level_to_int(0) == 0
    assert diagnostic_level_to_int(2) == 2


def test_invalid_byte_length():
    try:
        diagnostic_level_to_int(b'')
    except ValueError:
        pass
    else:
        raise AssertionError('empty byte field should be rejected')
