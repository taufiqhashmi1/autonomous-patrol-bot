# obstacle_avoidance.py

def compute_avoidance(fl_val, fr_val, l_val, r_val, threshold=80.0, max_speed=6.28):
    """
    Evaluates sensor readings and outputs override velocities.
    Returns: (is_avoiding, left_velocity, right_velocity)
    """
    front_obstacle = fl_val > threshold or fr_val > threshold
    left_obstacle = l_val > threshold
    right_obstacle = r_val > threshold

    if front_obstacle:
        if left_obstacle:
            # Blocked front and left: Spin right
            return True, max_speed, -max_speed
        elif right_obstacle:
            # Blocked front and right: Spin left
            return True, -max_speed, max_speed
        else:
            # Blocked dead ahead: Default spin right
            return True, max_speed, -max_speed
            
    elif left_obstacle:
        # Glancing obstacle on left: Veer right
        return True, max_speed, max_speed * 0.4
        
    elif right_obstacle:
        # Glancing obstacle on right: Veer left
        return True, max_speed * 0.4, max_speed

    # No obstacles
    return False, 0.0, 0.0