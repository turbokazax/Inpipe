from enum import Enum

class ControlTable(Enum):
    OPERATING_MODE= 11
    DRIVE_MODE = 10
    TORQUE_ENABLE= 512
    MOVING = 570
    HOMING_OFFSET = 20
    # PWM:
    GOAL_PWM = 548 # ( 0 to 2009 (100%) ) unit: 1 = 0.0497776%
    GOAL_CURRENT = 550 # -4500 to 4500 A, unit: 1 = 1 A
    PWM_LIMIT = 36 
    # position:
    GOAL_POSITION= 564
    CURRENT_POSITION= 580
    MAX_POSITION_LIMIT = 48
    MIN_POSITION_LIMIT = 52
    POSITION_kP = 532
    POSITION_kI = 530
    POSITION_kD = 528
    # velo:
    GOAL_VELOCITY = 552
    VELOCITY_kP = 526
    VELOCITY_kI = 524
    CURRENT_VELOCITY = 576
    #Profile velo / accel:
    PROFILE_VELOCITY = 560
    PROFILE_ACCELERATION = 556
    # led:
    LED_RED = 513
    LED_GREEN = 514
    LED_BLUE = 515
    # tick:
    REALTIME_TICK = 568


