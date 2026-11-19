"""python-can bus for the servo frames (sent) and the pressure frames (received). On SocketCAN the bitrate is set by `ip link`, not here."""


class ServoCan:
    def __init__(self, interface="socketcan", channel="can0"):
        import can
        self._can = can
        self.bus = can.Bus(interface=interface, channel=channel)

    def send(self, frame):
        self.bus.send(self._can.Message(arbitration_id=frame.arb_id, is_extended_id=frame.extended,
                                        data=frame.data))

    def recv(self):
        """Next received frame as (arb_id, extended, data bytes), or None if none is waiting."""
        m = self.bus.recv(timeout=0)
        if m is None:
            return None
        return m.arbitration_id, m.is_extended_id, bytes(m.data)

    def close(self):
        self.bus.shutdown()
