import math

FORWARD_SPEED = 3.0
KP_HEADING = 3.0

def compute_control(x, y, theta, target):

    dx = target[0] - x
    dy = target[1] - y

    distance = math.sqrt(dx*dx + dy*dy)
    desired_heading = math.atan2(dy, dx)

    heading_error = desired_heading - theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    # Pivot-then-drive logic to eliminate snaking
    if abs(heading_error) > 0.35:  
        linear = 0.0               
        angular = KP_HEADING * heading_error
    else:
        linear = FORWARD_SPEED     
        angular = KP_HEADING * heading_error

    # Standard differential drive kinematics
    left = linear - angular
    right = linear + angular

    return left, right, distance