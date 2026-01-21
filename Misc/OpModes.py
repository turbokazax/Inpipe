from enum import IntEnum
class OpModes(IntEnum):
    CURRENT = 0
    VELOCITY = 1
    POSITION = 3
    EXTENDED_POSITION = 4
    PWM = 16