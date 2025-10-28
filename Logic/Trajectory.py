from Mechas.DCMotor import DCMotor

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
    def getNextPosition(self):
        if self.current_index < len(self.trajectory.positions):
            pos = self.trajectory.positions[self.current_index]
            self.current_index += 1
            return pos
        else:
            self.trajectory.completed = True
            return None
    def follow(self, motor: DCMotor, condition = True):
        if not self.trajectory.completed and not motor.isMoving(verbose=False) and condition:
            next_pos = self.getNextPosition()
            if next_pos is not None:
                motor.setGoalPosition(next_pos)
                if motor.reachedGoalPosition():
                    print(f"Reached position {next_pos}.")
            else:
                print("Trajectory completed.")
    def isCompleted(self):
        return self.trajectory.completed
# trm = TrajectoryManager(Trajectory(100_000, 300_000, 500_000, 800_000, 1_000_000, 800_000, 500_000, 300_000, 150_000, 0))