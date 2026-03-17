from enum import IntEnum

class ControlTable_M42(IntEnum):
    # -----------------------
    # EEPROM AREA (non-volatile)
    # -----------------------
    MODEL_NUMBER = 0              # (R) 2 bytes
    MODEL_INFORMATION = 2         # (R) 4 bytes
    FIRMWARE_VERSION = 6          # (R) 1 byte

    ID = 7                        # (RW) 1 byte
    BAUD_RATE = 8                 # (RW) 1 byte
    RETURN_DELAY_TIME = 9         # (RW) 1 byte

    # NOTE: address 10 is not defined in this M42 manual.
    OPERATING_MODE = 11           # (RW) 1 byte

    HOMING_OFFSET = 13            # (RW) 4 bytes
    MOVING_THRESHOLD = 17         # (RW) 4 bytes

    TEMPERATURE_LIMIT = 21        # (RW) 1 byte
    MAX_VOLTAGE_LIMIT = 22        # (RW) 2 bytes
    MIN_VOLTAGE_LIMIT = 24        # (RW) 2 bytes

    ACCELERATION_LIMIT = 26       # (RW) 4 bytes
    TORQUE_LIMIT = 30             # (RW) 2 bytes
    VELOCITY_LIMIT = 32           # (RW) 4 bytes

    MAX_POSITION_LIMIT = 36       # (RW) 4 bytes
    MIN_POSITION_LIMIT = 40       # (RW) 4 bytes

    EXTERNAL_PORT_MODE_1 = 44     # (RW) 1 byte
    EXTERNAL_PORT_MODE_2 = 45     # (RW) 1 byte
    EXTERNAL_PORT_MODE_3 = 46     # (RW) 1 byte
    EXTERNAL_PORT_MODE_4 = 47     # (RW) 1 byte

    SHUTDOWN = 48                 # (RW) 1 byte

    # NOTE: Indirect Address range exists (49..559) but is intentionally omitted.

    # -----------------------
    # RAM AREA (volatile)
    # -----------------------
    TORQUE_ENABLE = 562           # (RW) 1 byte

    LED_RED = 563                 # (RW) 1 byte
    LED_GREEN = 564               # (RW) 1 byte
    LED_BLUE = 565                # (RW) 1 byte

    # Velocity controller gains
    VELOCITY_I_GAIN = 586         # (RW) 2 bytes
    VELOCITY_P_GAIN = 588         # (RW) 2 bytes

    # Position gain(s) available on M42
    POSITION_P_GAIN = 594         # (RW) 2 bytes

    GOAL_POSITION = 596           # (RW) 4 bytes
    GOAL_VELOCITY = 600           # (RW) 4 bytes
    GOAL_TORQUE = 604             # (RW) 2 bytes
    GOAL_ACCELERATION = 606       # (RW) 4 bytes

    MOVING = 610                  # (R) 1 byte

    PRESENT_POSITION = 611        # (R) 4 bytes
    PRESENT_VELOCITY = 615        # (R) 4 bytes
    PRESENT_CURRENT = 621         # (R) 2 bytes
    PRESENT_INPUT_VOLTAGE = 623   # (R) 2 bytes
    PRESENT_TEMPERATURE = 625     # (R) 1 byte

    EXTERNAL_PORT_DATA_1 = 626    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_2 = 628    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_3 = 630    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_4 = 632    # (R/RW) 2 bytes

    # NOTE: Indirect Data range exists (634..889) but is intentionally omitted.

    REGISTERED_INSTRUCTION = 890  # (R) 1 byte
    STATUS_RETURN_LEVEL = 891     # (RW) 1 byte
    HARDWARE_ERROR_STATUS = 892   # (R) 1 byte

# from enum import IntEnum

class ControlTable_L42(IntEnum):
    # -----------------------
    # EEPROM AREA (non-volatile)
    # -----------------------
    MODEL_NUMBER = 0              # (R) 2 bytes
    MODEL_INFORMATION = 2         # (R) 4 bytes
    FIRMWARE_VERSION = 6          # (R) 1 byte

    ID = 7                        # (RW) 1 byte
    BAUD_RATE = 8                 # (RW) 1 byte
    RETURN_DELAY_TIME = 9         # (RW) 1 byte

    # NOTE: address 10 is not defined in this L42 manual.
    OPERATING_MODE = 11           # (RW) 1 byte

    HOMING_OFFSET = 13            # (RW) 4 bytes
    MOVING_THRESHOLD = 17         # (RW) 4 bytes

    TEMPERATURE_LIMIT = 21        # (RW) 1 byte
    MAX_VOLTAGE_LIMIT = 22        # (RW) 2 bytes
    MIN_VOLTAGE_LIMIT = 24        # (RW) 2 bytes

    ACCELERATION_LIMIT = 26       # (RW) 4 bytes
    TORQUE_LIMIT = 30             # (RW) 2 bytes
    VELOCITY_LIMIT = 32           # (RW) 4 bytes

    MAX_POSITION_LIMIT = 36       # (RW) 4 bytes
    MIN_POSITION_LIMIT = 40       # (RW) 4 bytes

    EXTERNAL_PORT_MODE_1 = 44     # (RW) 1 byte
    EXTERNAL_PORT_MODE_2 = 45     # (RW) 1 byte
    EXTERNAL_PORT_MODE_3 = 46     # (RW) 1 byte
    EXTERNAL_PORT_MODE_4 = 47     # (RW) 1 byte

    SHUTDOWN = 48                 # (RW) 1 byte

    # Indirect Address range starts at 49 (2 bytes each), up to 559
    INDIRECT_ADDRESS_1 = 49       # (RW) 2 bytes

    # -----------------------
    # RAM AREA (volatile)
    # -----------------------
    TORQUE_ENABLE = 562           # (RW) 1 byte

    LED_RED = 563                 # (RW) 1 byte
    LED_GREEN = 564               # (RW) 1 byte
    LED_BLUE = 565                # (RW) 1 byte

    # Velocity controller gains
    VELOCITY_I_GAIN = 586         # (RW) 2 bytes
    VELOCITY_P_GAIN = 588         # (RW) 2 bytes

    # Position PID gains (manual describes 590/592/594 as D/I/P)
    POSITION_D_GAIN = 590         # (RW) 2 bytes
    POSITION_I_GAIN = 592         # (RW) 2 bytes
    POSITION_P_GAIN = 594         # (RW) 2 bytes

    GOAL_POSITION = 596           # (RW) 4 bytes
    GOAL_VELOCITY = 600           # (RW) 4 bytes
    GOAL_TORQUE = 604             # (RW) 2 bytes
    GOAL_ACCELERATION = 606       # (RW) 4 bytes

    MOVING = 610                  # (R) 1 byte

    PRESENT_POSITION = 611        # (R) 4 bytes
    PRESENT_VELOCITY = 615        # (R) 4 bytes
    PRESENT_CURRENT = 621         # (R) 2 bytes
    PRESENT_INPUT_VOLTAGE = 623   # (R) 2 bytes
    PRESENT_TEMPERATURE = 625     # (R) 1 byte

    EXTERNAL_PORT_DATA_1 = 626    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_2 = 628    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_3 = 630    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_4 = 632    # (R/RW) 2 bytes

    # Indirect Data range starts at 634 (1 byte each), up to 889
    INDIRECT_DATA_1 = 634         # (RW) 1 byte

    REGISTERED_INSTRUCTION = 890  # (R) 1 byte
    STATUS_RETURN_LEVEL = 891     # (RW) 1 byte
    HARDWARE_ERROR_STATUS = 892   # (R) 1 byte

class ControlTable_H42P(IntEnum):
    # -----------------------
    # EEPROM AREA (non-volatile)
    # -----------------------
    MODEL_NUMBER = 0              # (R)  2 bytes
    MODEL_INFORMATION = 2         # (R)  4 bytes
    FIRMWARE_VERSION = 6          # (R)  1 byte

    ID = 7                        # (RW) 1 byte
    BAUD_RATE = 8                 # (RW) 1 byte
    RETURN_DELAY_TIME = 9         # (RW) 1 byte

    DRIVE_MODE = 10               # (RW) 1 byte
    OPERATING_MODE = 11           # (RW) 1 byte
    SECONDARY_ID = 12             # (RW) 1 byte   # "Secondary(Shadow) ID"
    PROTOCOL_TYPE = 13            # (RW) 1 byte

    HOMING_OFFSET = 20            # (RW) 4 bytes
    MOVING_THRESHOLD = 24         # (RW) 4 bytes

    TEMPERATURE_LIMIT = 31        # (RW) 1 byte
    MAX_VOLTAGE_LIMIT = 32        # (RW) 2 bytes
    MIN_VOLTAGE_LIMIT = 34        # (RW) 2 bytes

    PWM_LIMIT = 36                # (RW) 2 bytes
    CURRENT_LIMIT = 38            # (RW) 2 bytes
    ACCELERATION_LIMIT = 40       # (RW) 4 bytes
    VELOCITY_LIMIT = 44           # (RW) 4 bytes

    MAX_POSITION_LIMIT = 48       # (RW) 4 bytes
    MIN_POSITION_LIMIT = 52       # (RW) 4 bytes

    EXTERNAL_PORT_MODE_1 = 56     # (RW) 1 byte
    EXTERNAL_PORT_MODE_2 = 57     # (RW) 1 byte
    EXTERNAL_PORT_MODE_3 = 58     # (RW) 1 byte
    EXTERNAL_PORT_MODE_4 = 59     # (RW) 1 byte

    STARTUP_CONFIGURATION = 60    # (RW) 1 byte
    SHUTDOWN = 63                 # (RW) 1 byte

    # NOTE: Indirect Address range exists in the manual, but is intentionally omitted.

    # -----------------------
    # RAM AREA (volatile)
    # -----------------------
    TORQUE_ENABLE = 512           # (RW) 1 byte

    LED_RED = 513                 # (RW) 1 byte
    LED_GREEN = 514               # (RW) 1 byte
    LED_BLUE = 515                # (RW) 1 byte

    STATUS_RETURN_LEVEL = 516     # (RW) 1 byte
    REGISTERED_INSTRUCTION = 517  # (R)  1 byte
    HARDWARE_ERROR_STATUS = 518   # (R)  1 byte

    VELOCITY_I_GAIN = 524         # (RW) 2 bytes
    VELOCITY_P_GAIN = 526         # (RW) 2 bytes

    POSITION_D_GAIN = 528         # (RW) 2 bytes
    POSITION_I_GAIN = 530         # (RW) 2 bytes
    POSITION_P_GAIN = 532         # (RW) 2 bytes

    FEEDFORWARD_2ND_GAIN = 536    # (RW) 2 bytes
    FEEDFORWARD_1ST_GAIN = 538    # (RW) 2 bytes

    BUS_WATCHDOG = 546            # (RW) 1 byte

    GOAL_PWM = 548                # (RW) 2 bytes
    GOAL_CURRENT = 550            # (RW) 2 bytes
    GOAL_VELOCITY = 552           # (RW) 4 bytes

    PROFILE_ACCELERATION = 556    # (RW) 4 bytes
    PROFILE_VELOCITY = 560        # (RW) 4 bytes

    GOAL_POSITION = 564           # (RW) 4 bytes

    REALTIME_TICK = 568           # (R)  2 bytes
    MOVING = 570                  # (R)  1 byte
    MOVING_STATUS = 571           # (R)  1 byte

    PRESENT_PWM = 572             # (R)  2 bytes
    PRESENT_CURRENT = 574         # (R)  2 bytes
    PRESENT_VELOCITY = 576        # (R)  4 bytes
    PRESENT_POSITION = 580        # (R)  4 bytes

    VELOCITY_TRAJECTORY = 584     # (R)  4 bytes
    POSITION_TRAJECTORY = 588     # (R)  4 bytes

    PRESENT_INPUT_VOLTAGE = 592   # (R)  2 bytes
    PRESENT_TEMPERATURE = 594     # (R)  1 byte

    EXTERNAL_PORT_DATA_1 = 600    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_2 = 602    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_3 = 604    # (R/RW) 2 bytes
    EXTERNAL_PORT_DATA_4 = 606    # (R/RW) 2 bytes

    BACKUP_READY = 878            # (R)  1 byte

    # NOTE: Indirect Data range exists in the manual, but is intentionally omitted.



