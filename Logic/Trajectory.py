from Mechas.DCMotor import DCMotor
import time

class Trajectory:
    def __init__(self, *positions):
        self.completed = False
        self.positions = positions
    def addPosition(self, pos):
        self.positions.append(pos)

class TrajectoryManager:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.current_index = 0
        self.trajectory_completed = False
        self.looped = False
    def setTrajectory(self, trajectory):
        self.trajectory = trajectory
    def setLooped(self, looped: bool):
        self.looped = looped
    def getNextPosition(self):
        if self.current_index < len(self.trajectory.positions):
            pos = self.trajectory.positions[self.current_index]
            self.current_index += 1
            return pos
        elif self.looped:
            self.current_index = 0
            pos = self.trajectory.positions[self.current_index]
            self.current_index += 1
            return pos
        else:
            print("Trajectory completed.")
            self.trajectory.completed = True
            return None
    def follow(self, motor: DCMotor, condition = True):
        # if not self.trajectory.completed and not motor.isMoving(verbose=False) and condition:
        #     next_pos = self.getNextPosition()
        #     if next_pos is not None and not motor.isMoving(verbose=False):
        #         motor.setGoalPosition(next_pos)
        print(f"Current Index: {self.current_index}, Motor Position: {motor.getCurrentPosition(verbose=False)}")
        # if motor.reachedGoalPosition(tolerance=50) and not self.current_index == 0:
        #     time.sleep(3)
        #     print(f"REACHED position {motor.getCurrentPosition()}.")
        # elif self.current_index == 0:
        #     next_pos = self.getNextPosition()
        #     if next_pos is not None:
        #         motor.setGoalPosition(next_pos)
        # else:
        #     if not self.trajectory.completed and not motor.isMoving(verbose=False) and condition:
        #         next_pos = self.getNextPosition()
        #         if next_pos is not None:
        #             motor.setGoalPosition(next_pos)
        if not self.trajectory.completed and not motor.isMoving(verbose=False) and condition:
            next_pos = self.getNextPosition()
            if motor.reachedGoalPosition(tolerance=50) and not self.current_index == 0:
                print(f"REACHED position {motor.getCurrentPosition(verbose=False)}.")
            if next_pos is not None and not motor.isMoving(verbose=False):
                motor.setGoalPosition(next_pos)
    def followGroup(self, motor_group, condition = True):
        for motor in motor_group.motors:
            self.follow(motor, condition=condition)
    def isCompleted(self):
        return self.trajectory.completed
#Example:
# trm = TrajectoryManager(Trajectory(100_000, 300_000, 500_000, 800_000, 1_000_000, 800_000, 500_000, 300_000, 150_000, 0))
# It means the motor will move to 100_000, then 300_000, then 500_000, and so on...
# You can call trm.follow(motor) in a loop to make the motor follow the trajectory.
# Also i plan to implement the features like waiting for a certain condition to move to the next position.
# Although it will be done later.
# My favourite uma musume is Oguri cap