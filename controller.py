import time

class PIDController:
    """ Use the PID controller to control wheels according to intersection point.
    """
    previous = 0.0
    integral = 0.0

    def __init__(self, Kp=0.1, Ki=0.0, Kd=0.0, useP=True, useI=True, useD=True):
        """
        Parameters
        ----------
        - Kp : float, constant for P (proportional) section of the PID. Default is 0.1
        - Ki : float, constant for I (integral) section of the PID. Default is 0.0
        - Kd : float, constant for D (derivative) section of the PID. Default is 0.0
        - useP, useI, useD : boolean, PID controller can work as P, PI etc. 
                             Turn these to False to not use that part of the PID.
                             Default is True. It is usually a good practice to use P.
        """
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
        self.integral += error * dt 
        derivative = float(error - self.previous) / float(dt)
        self.previous = error
        if self.useP:
            output += self.Kp * error
        if self.useI:
            output += self.Ki * self.integral
        if self.useD:
            output += self.Kd * derivative
        self.lasttime = time.time()
        return output

