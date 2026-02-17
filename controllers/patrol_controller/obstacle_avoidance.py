def compute_wall_follow(fl_val, fr_val, l_val, r_val, threshold=150.0, max_speed=3.14, follow_dir="LEFT"):
    crash_threshold = 400.0 

    # --- 1. ANTI-WEDGE / DE-COLLISION ---
    # Now includes side sensors. If snagged, reverse asymmetrically to peel away.
    if fl_val > crash_threshold or fr_val > crash_threshold or l_val > crash_threshold or r_val > crash_threshold:
        if l_val > crash_threshold:
            # Snagged on left: back up while twisting right
            return -max_speed, -max_speed * 0.5
        elif r_val > crash_threshold:
            # Snagged on right: back up while twisting left
            return -max_speed * 0.5, -max_speed
        else:
            return -max_speed, -max_speed

    # --- 2. SMOOTH DYNAMIC BUG2 ---
    front = fl_val > threshold or fr_val > threshold
    left = l_val > threshold
    right = r_val > threshold

    if follow_dir == "LEFT":
        if front:
            return max_speed, -max_speed
        elif left:
            # Increased the outward curve from 0.9 to 0.75 to steer wider around complex shapes
            return max_speed, max_speed * 0.75
        else:
            return max_speed * 0.5, max_speed
            
    elif follow_dir == "RIGHT":
        if front:
            return -max_speed, max_speed
        elif right:
            return max_speed * 0.75, max_speed
        else:
            return max_speed, max_speed * 0.5

    return 0.0, 0.0