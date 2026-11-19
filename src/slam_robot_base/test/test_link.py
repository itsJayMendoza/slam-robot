import pytest

from slam_robot_base.link import DEFAULT_COUNTS_PER_M, BaseLink, LinkError, parse_state


class FakePort:
    def __init__(self, replies):
        self.replies, self.written = list(replies), []

    def write(self, data):
        self.written.append(data.decode())

    def readline(self):
        return (self.replies.pop(0) + "\n").encode() if self.replies else b""


def test_parse_state():
    s = parse_state("e 1200 -35 3010 -12 0")
    assert (s.left, s.right, s.left_cps, s.right_cps, s.fault) == (1200, -35, 3010.0, -12.0, 0)
    assert s.fault_text == "none"


def test_fault_text():
    assert parse_state("e 0 0 0 0 17").fault_text == "left stalled, left overspeed"


@pytest.mark.parametrize("line", ["ok stopped", "e 1 2 3", "e a b c d e", ""])
def test_bad_reply(line):
    with pytest.raises(LinkError):
        parse_state(line)


def test_velocity_rounds_and_skips_comments():
    port = FakePort(["# fault 1", "e 10 20 30 40 1"])
    link = BaseLink(port)
    s = link.velocity(1234.6, -99.4)
    assert port.written == ["v 1235 -99\n"]
    assert s.fault == 1 and link.notices == ["# fault 1"]


def test_no_reply():
    with pytest.raises(LinkError, match="no reply"):
        BaseLink(FakePort([])).read()


def test_stop_must_be_acknowledged():
    BaseLink(FakePort(["ok stopped"])).stop()
    with pytest.raises(LinkError):
        BaseLink(FakePort(["err unknown command"])).stop()


def test_default_scale():
    assert DEFAULT_COUNTS_PER_M == pytest.approx(178200)


def test_stray_line_skipped():
    # the tail of a boot banner cut by reset_input_buffer
    link = BaseLink(FakePort(["EN pin", "e 1 2 3 4 0"]))
    assert link.read().left == 1 and link.skipped == ["EN pin"]


def test_error_reply_not_skipped():
    with pytest.raises(LinkError, match="err unknown"):
        BaseLink(FakePort(["err unknown command"])).stop()
