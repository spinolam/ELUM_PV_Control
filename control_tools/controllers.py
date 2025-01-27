
class PIController:
    """
    Class of Controller Proportional+Integrator
    Initial parameters:

    kp (float): Proportional gain. 
    ki (float): Integral gain.
    delta_t (float): Sample time (s).
    integral (float): Integral error.

    """
    def __init__(self, kp, ki, delta_t=1):
        self.kp = kp
        self.ki = ki
        self.delta_t = delta_t
        self.integral_error = 0

    def integral_action(self, setpoint: float,measured_value: float) -> float:
        # Update integral term
        error = setpoint - measured_value
        self.integral_error += error*self.delta_t 
        # Return PI action
        return self.kp * error + self.ki * self.integral_error
    
    def reset_integral(self):
        self.integral = 0

class FeedForwardController(PIController):


    def __init__(self, kp, ki, delta_t, kff):
        # Call the parent class's initializer
        super().__init__(kp, ki, delta_t)
        self.kff = -kff


    def feedfoward_action(self, setpoint,measured_value,measured_disturbance):
        # Update integral term
        error = setpoint - measured_value
        self.integral_error += error*self.delta_t 
        # Return PI action
        uPI= self.kp * error + self.ki * self.integral_error
        # Calculate feedforward
        uFF=(self.kff)*measured_disturbance 
        # Return PI+Feedforward action
        return uPI+1*uFF

    def reset_feedforward(self):
        self.feedforward = 0