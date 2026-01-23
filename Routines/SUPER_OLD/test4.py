from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Misc.OpModes import OpModes
from Logic.Trajectory import Trajectory, TrajectoryManager
import time

motor1 = DCMotor(0)
pos1 = motor1.getCurrentPosition()
# class Trajectory:
#     def __init__(self, *positions):
#         self.completed = False
#         self.positions = positions
#     def addPosition(self, pos):
#         self.positions.append(pos)

# class TrajectoryManager:
#     def __init__(self, trajectory):
#         self.trajectory = trajectory
#         self.current_index = 0
#     def getNextPosition(self):
#         if self.current_index < len(self.trajectory.positions):
#             pos = self.trajectory.positions[self.current_index]
#             self.current_index += 1
#             return pos
#         else:
#             self.trajectory.completed = True
#             return None
#     def follow(self, motor: DCMotor, condition = True):
#         if not self.trajectory.completed and not motor.isMoving(verbose=False) and condition:
#             next_pos = self.getNextPosition()
#             if next_pos is not None:
#                 motor.setGoalPosition(next_pos)
#                 if motor.reachedGoalPosition():
#                     print(f"Reached position {next_pos}.")
#             else:
#                 print("Trajectory completed.")
trm = TrajectoryManager(Trajectory(100_000, 300_000, 500_000, 800_000, 1_000_000, 800_000, 500_000, 300_000, 150_000, 0))
global polling
polling = True

class test4(Routine):
    def __init__(self):
        super().__init__()
        motor1.enableTorque()
        motor1.setOpMode(OpModes.EXTENDED_POSITION)
        # motor1.setOpMode(OpModes.PWM)
        # motor1.setGoalPWM(0)
        if abs(motor1.getCurrentPosition()) >5 : motor1.forceZero()
        motor1.setReverseMode(False)
        self.polling = True
    def loop(self):
        trm.follow(motor1, condition=self.polling)
        # motor1.getCurrentPosition()
        # motor1.getTorque() 
        # motor1.getPWMLimit(verbose=True)
        # motor1.setGoalPWM(-2008)
        # time.sleep(0.01)
        # motor1.setGoalPosition(motor1.getMaxPositionLimit() + 3_000_000)
        # motor1.rotateByAngle(90, times = 1)f
        # motor1.isMoving()
        # i = input()
        # i = ''
        # if self.polling: i = input()
        # if i == 'q':
        #     motor1.rotateByAngle(90, times=1)
        # elif i == 'a':
        #     motor1.rotateByAngle(-90, times=1)
        # elif i == 'z':
        #     self.polling = not self.polling
        #     trm.follow(motor1)
        TrajectoryManager.follow(trm, motor1)
        pass

    def run(self):
        try:
            while True:
                self.loop()
        except KeyboardInterrupt:
            print("Routine stopped by user.")
            self.onStop()

    def onStop(self):
        # motor1.getInfo(verbose="True")["opmode"]
        motor1.stop()
        # motor1.setGoalPWM(0)
        # if motor1.getInfo()["opmode"] == OpModes.TORQUE and motor1.getGoalCurrent() != 0:
        #     motor1.setGoalCurrent(0)


if __name__ == "__main__":
    routine = test4()
    routine.run()