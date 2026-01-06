
import numpy as np
class ArduinoPID:
    def __init__(self, Kp, Ki, Kd, Ts, out_min=-1023, out_max=1023):
        self.Kp = Kp
        self.Ki = Ki * Ts
        self.Kd = Kd / Ts

        self.out_min = out_min
        self.out_max = out_max

        self.integral = 0.0
        self.last_input = 0.0

    def step(self, setpoint, input_):
        error = setpoint - input_
        d_input = input_ - self.last_input

        # integral term (same as in Arduino implementation)
        self.integral += self.Ki * error
        self.integral = np.clip(self.integral, self.out_min, self.out_max)

        # PID output
        output = self.Kp * error + self.integral - self.Kd * d_input
        output = np.clip(output, self.out_min, self.out_max)

        self.last_input = input_
        return output