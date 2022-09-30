import time

class PIDController:

    previous = 0.0
    integral = 0.0

    def __init__(self, Kp=0.1, Ki=0.1, Kd=0.1, useP=True, useI=True, useD=True):
        self.Kp = Kp 
        self.Ki = Ki 
        self.Kd = Kd
        self.useP = useP
        self.useI = useI
        self.useD = useD
        self.lasttime = time.time()


    def move(self, current, target=0.0):
        """ Determines the wheel angle for the next step using the PID controller. 
        Use on regular intervals for best performance.

        Parameters
        ----------
        current : int or float
             
        target : int or float, default is 0.0

        Returns
        ----------
        float, angle to set for the front wheels (if constants are set correctly when initializing the controller)

        """
        output = 0.0
        error = target - current
        dt = time.time() - self.lasttime

        # Integral 
        self.integral = self.integral + error * dt 
        derivative = float(error - self.previous) / float(dt)
        self.previous = error
        if self.useP:
            output += self.Kp * error
        if self.useI:
            output += self.Ki * self.integral
        if self.useD:
            output += self.Kd * derivative
        return output

