import numpy as np

""" TODO: Move this file to a more appropriate location
    TODO: Test formulas and fix as necessary (not tested yet)

    Implements differential steering.
    Assumes driving in 3 states:
        - left 
        - straight
        - right
    
"""

PI = np.pi


def get_back_wheel_speeds(left: bool, base_speed: float, width: float, length: float, angle: float = 30) -> tuple:
    """Calculates the differential speed of wheels when turning such that the base speed is maintained.
        Returns the resulting back wheel write speeds in the same units as the base speed.

        Args:
            left (bool): true for turning left, false for turning right
            base_speed (float): default robot speed, in SI units m/s
            width (float): the physical width between left and right wheels, in SI units m
            length (float): the physical length between front and back wheels, in SI units m
            angle (float): (Optional) the angle that the front of the robot is turning, in
                            degrees, defaults to 30 deg
        Returns:
            tuple: the following wheel speeds: 
                ( (left_front, right_front) ,  (left_back, right_back) )

    """

    # calculated time for a single revolution should be the same for each wheel
    #
    # v_in / (2*pi*r_mid - width/2) = v_out / (2*pi*r_mid + width/2) = v_mid / (2*pi*r_mid)
    # for 30 deg angle, r_mid = length/tan(30deg)

    rad_mid = length / np.tan(angle * PI / 180)
    rad_front_in = np.sqrt((rad_mid - width / 2)**2 + length**2)
    rad_front_out = np.sqrt((rad_mid + width / 2)**2 + length**2)

    inner_back = base_speed * (1 - width / (2 * rad_mid))
    outer_back = base_speed * (1 + width / (2 * rad_mid))

    inner_front = base_speed * rad_front_in / rad_mid
    outer_front = base_speed * rad_front_out / rad_mid

    if (left):
        left_back = inner_back
        right_back = outer_back

        left_front = inner_front
        right_front = outer_front
    else:
        right_back = inner_back
        left_back = outer_back

        right_front = inner_front
        left_front = outer_front
    
    return ((left_front, right_front), (left_back, right_back))
