import numpy as np

""" TODO: Move this file to a more appropriate location
    TODO: Test formulas and fix as necessary (not tested yet)

    Implements differential steering.
    Assumes driving in 3 states:
        - left 
        - straight
        - right
    
"""

def get_wheel_speeds(left: bool, base_speed: float, width: float, length: float) -> tuple: 
    """Calculates the differential speed of wheels when turning such that the base speed is maintained.
        Returns the resulting wheel write speeds in the same units as the base speed.

        Args:
            left (bool): true for turning left, false for turning right
            base_speed (float): default robot speed, in SI units m/s
            width (float): the physical width between left and right wheels, in SI units m
            length (float): the physical length between front and back wheels, in SI units m
        Returns:
            tuple: (left_back_wheel_speed, right_back_wheel_speed) for turning at a 30deg angle

    """

    # calculated time for a single revolution should be the same for each wheel
    #
    # v_in / (2*pi*r_mid - width/2) = v_out / (2*pi*r_mid + width/2) = v_mid / (2*pi*r_mid)
    # for 30 deg angle, r_mid = length/np.sqrt(3)
    inner_wheel_speed = base_speed * (1 - np.sqrt(3) * width / (2 * length))
    outer_wheel_speed = base_speed * (1 + np.sqrt(3) * width / (2 * length))

    if (left):
        left_wheel_speed = inner_wheel_speed
        right_wheel_speed = outer_wheel_speed
    else:
        right_wheel_speed = inner_wheel_speed
        left_wheel_speed = outer_wheel_speed

    return (left_wheel_speed,right_wheel_speed)