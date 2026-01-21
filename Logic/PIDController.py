class PIDController():
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._last_error = 0.0
        self._integral = 0.0
        self.output_limit = None  
        self.integral_limit = None

    def setPID(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def setKp(self, kp):
        self.kp = kp
    def setKi(self, ki):
        self.ki = ki
    def setKd(self, kd):
        self.kd = kd

    def setOutputLimit(self, limit):
        self.output_limit = limit

    def setIntegralLimit(self, limit):
        self.integral_limit = limit

    def calculate(self, reference, target, dt):
        error = target - reference
        self._integral += error * dt
        derivative = (error - self._last_error) / dt if dt > 0 else 0.0

        if self.integral_limit is not None:
            self._integral = max(min(self._integral, self.integral_limit), -self.integral_limit)

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)

        if self.output_limit is not None:
            output = max(min(output, self.output_limit), -self.output_limit)

        self._last_error = error
        return output