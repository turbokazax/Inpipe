ANGLE_PER_POS_TICK = 0.00059259

def deg(angle: float) -> int:
    return int(angle / ANGLE_PER_POS_TICK)