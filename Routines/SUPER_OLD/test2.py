from Mechas.DCMotor import DCMotor
from Inpipe.Mechas.MotorGroup import MotorGroup
from Misc.OpModes import OpModes
from Misc.Colors import Colors

motor1 = DCMotor(1)
motor2 = DCMotor(2)
motor3 = DCMotor(3)
motor4 = DCMotor(4)
motorGroup = MotorGroup(motor1, motor2, motor3, motor4)
# motorGroup.setOpmode(OpModes.VELOCITY)
# motorGroup.enableTorque()
# motorGroup.zero()
# motor1.setOpMode(OpModes.VELOCITY)
# motor1.setGoalVelocity(0)
motor1.setOpMode(OpModes.POSITION)
motor1.enableTorque()
motor1.setGoalPosition(180_000)

