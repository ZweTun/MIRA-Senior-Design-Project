import struct
from pathlib import Path
import sys

from planner.models import CellPos, FeedbackPacket, GuidanceState
from planner.mosaic import Mosaic

def pack_feedback(packet: FeedbackPacket) -> bytes:
    # float magnitude, int8 dx, int8 dy, uint8 state
    return struct.pack("<fbbB", packet.magnitude, packet.dx, packet.dy, int(packet.state))


def debug_send(packet: FeedbackPacket) -> None:
    print(
        f"send -> mag={packet.magnitude:.2f}, "
        f"dx={packet.dx}, dy={packet.dy}, state={packet.state.name}, current_pos=({packet.current_pos.x},{packet.current_pos.y})"
    )




def send_serial_packet(ser, packet):
    line = (
        f"{packet.magnitude:.2f},"
        f"{packet.dx},"
        f"{packet.dy},"
        f"{int(packet.state)},"
        f"{packet.current_pos.x},"
        f"{packet.current_pos.y}\n"
    )
    # print("TX:", line.strip())
    ser.write(line.encode("ascii", errors="ignore"))
