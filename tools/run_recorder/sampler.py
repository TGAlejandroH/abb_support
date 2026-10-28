"""The recorder's sampling loop: one RWS session, polled as fast as it answers.

Every tick reads the FAST channels (program and motion pointer, TCP); the
SLOW channels (weld data, signals, execution state) are read one per tick in
rotation, so adding a slow channel does not slow the pointer and the TCP down.
A sample stores raw text values; interpreting them is the analysis' job, so a
recording can be re-analysed after the checks change.

Channel kinds:
    ("pointers",)                -> [pp, mp], each [module, routine, line] or None
    ("tcp", tool, wobj)          -> [x, y, z] mm
    ("tcp", tool, wobj, unit)    -> the same, read only while ``unit`` is active
    ("joints", unit)             -> {"rax_1": deg, ...} of a positioner (active axes only)
    ("symbol", module, name)     -> RAPID value literal
    ("signal", path)             -> lvalue text
    ("exec",)                    -> running / stopped
"""

import threading

from rws_reader import now


def _read(reader, channel):
    kind = channel[0]
    if kind == "pointers":
        return [list(p) if p else None for p in reader.pointers()]
    if kind == "tcp":
        return list(reader.tcp(channel[1], channel[2]))
    if kind == "joints":
        return reader.joints(channel[1])
    if kind == "symbol":
        return reader.symbol(channel[1], channel[2])
    if kind == "signal":
        return reader.signal(channel[1])
    if kind == "exec":
        return reader.exec_state()
    raise ValueError("unknown channel %r" % (channel,))


def channel_key(channel):
    # A TCP channel may name the unit its work object moves with; the key leaves it out.
    return ":".join(channel[:3] if channel[0] == "tcp" else channel)


def _unit_inactive(channel, sample):
    """A TCP channel in a moving work object is read only when the same tick read that unit's
    joints as active (the joints channel is added first). Reading it while the unit is
    deactivated logs 50066 on the controller."""
    return channel[0] == "tcp" and len(channel) > 3 and not sample.get("joints:" + channel[3])


class Sampler(threading.Thread):
    """Polls until ``halt`` is set or ``should_stop(sample)`` returns True.

    ``samples`` is a list of dicts: ``t`` (s since ``t0``), ``dt`` (s the tick
    took), and one entry per channel read in that tick (missing = not read
    this tick). Read errors are counted and the session is replaced after
    three in a row, so a lockout does not end the recording.
    """

    def __init__(self, reader, fast, slow=(), t0=None, should_stop=None, max_seconds=900.0):
        super().__init__(daemon=True)
        self.reader = reader
        self.fast = list(fast)
        self.slow = list(slow)
        self.t0 = now() if t0 is None else t0
        self.should_stop = should_stop
        self.max_seconds = max_seconds
        self.samples = []
        self.read_errors = []
        self.halt = threading.Event()

    def add_fast(self, channel):
        """Add a fast channel while running (e.g. the TCP in the weld's work object, once the
        weld module is known). A list append is atomic, and each tick copies the list."""
        if channel not in self.fast:
            self.fast.append(channel)

    def add_slow(self, channel):
        if channel not in self.slow:
            self.slow.append(channel)

    def run(self):
        rotation = 0
        failures = 0
        while not self.halt.is_set() and now() - self.t0 < self.max_seconds:
            start = now()
            sample = {"t": round(start - self.t0, 4)}
            channels = list(self.fast)
            if self.slow:
                channels.append(self.slow[rotation % len(self.slow)])
                rotation += 1
            for channel in channels:
                if _unit_inactive(channel, sample):
                    continue
                try:
                    sample[channel_key(channel)] = _read(self.reader, channel)
                    failures = 0
                except Exception as exc:
                    failures += 1
                    self.read_errors.append((round(now() - self.t0, 4), channel_key(channel), str(exc)[:200]))
                    if failures >= 3:
                        self.reader.reconnect()
                        failures = 0
            sample["dt"] = round(now() - start, 4)
            self.samples.append(sample)
            if self.should_stop and self.should_stop(sample):
                break
