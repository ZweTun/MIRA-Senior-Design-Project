import math
from typing import Optional, Set
from planner.models import CellPos, FeedbackPacket, GuidanceState

def __init__(self):
    pass

def calculate_vector(targets, current):
    if not targets:
        return None

    closest_target = next(iter(targets))
    min_distance = float("inf")

    for target in targets:
        distance = abs(current.x - target.x) + abs(current.y - target.y)
        if distance < min_distance:
            min_distance = distance
            closest_target = target

    return CellPos(
        x=abs(current.x - closest_target.x),
        y=abs(current.y - closest_target.y),
    )

# def calculate_vector(targets: Set[CellPos], current: CellPos):
#     if not targets:
#         return None

#     closest_target = None
#     min_distance = float("inf")

#     for target in targets:
#         distance = abs(current.x - target.x) + abs(current.y - target.y)  
#         if distance < min_distance:
#             min_distance = distance
#             closest_target = target

#     return CellPos(
#         x=closest_target.x - current.x,   # signed
#         y=closest_target.y - current.y,   # signed
#     )


def classify_state(magnitude: float) -> GuidanceState:
    if magnitude < 0:
        return GuidanceState.LOST
    if magnitude <= 0.05:
        return GuidanceState.REACHED
    if magnitude <= 1.0:
        return GuidanceState.CLOSE
    return GuidanceState.GUIDE


def make_feedback_from_diff(diff: Optional[CellPos], current_pos: CellPos, state=None) -> FeedbackPacket:
    if diff is None:
        return FeedbackPacket(
            magnitude=-1.0,
            dx=0,
            dy=0,
            state=GuidanceState.LOST,
            current_pos=current_pos,

        )

    dx = diff.x
    dy = diff.y
    magnitude = math.sqrt(dx * dx + dy * dy)
    state = classify_state(magnitude) if state is None else state

    return FeedbackPacket(
        magnitude=float(magnitude),
        dx=int(dx),
        dy=int(dy),
        state=state,
        current_pos=current_pos,
    )