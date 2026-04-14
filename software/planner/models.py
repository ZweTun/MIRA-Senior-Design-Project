from dataclasses import dataclass
from enum import IntEnum

def __init__(self):
    pass


@dataclass(frozen=True, order=True)
class CellPos:
    x: int
    y: int



class GuidanceState(IntEnum):
    LOST = 0
    GUIDE = 1
    CLOSE = 2
    REACHED = 3
    CORRECT = 4
    INCORRECT = 5


@dataclass
class FeedbackPacket:
    magnitude: float
    dx: int
    dy: int
    state: GuidanceState
    current_pos: CellPos = CellPos(-1, -1)

