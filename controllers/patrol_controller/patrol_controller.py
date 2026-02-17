import sys
from pathlib import Path
import math
from controller import Robot

sys.path.append(str(Path(__file__).resolve().parent))

from kml_parser import load_kml
from geo_convert import convert_to_local
from navigation import compute_control
from obstacle_avoidance import compute_wall_follow

robot = Robot()
timestep = int(robot.getBasicTimeStep())
project_path = robot.getProjectPath()

gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

ps_front_right = robot.getDevice("ps0")
ps_front_right.enable(timestep)
ps_right = robot.getDevice("ps2")
ps_right.enable(timestep)
ps_left = robot.getDevice("ps5")
ps_left.enable(timestep)
ps_front_left = robot.getDevice("ps7")
ps_front_left.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

try:
    latlon = load_kml("patrol_area.kml", project_path)
    waypoints = convert_to_local(latlon, arena_size=4.0)
except Exception as e:
    print("KML LOAD FAILED:", e)
    waypoints = [(0, 0)]

current_wp = 0
step_count = 0

state = "NAVIGATING"
hit_dist_to_target = float('inf')
follow_direction = "LEFT"
prev_wp = waypoints[-1] if waypoints else (0, 0)

print("Controller started. ENU Mapping & Tangent Bug Hybrid active...")

while robot.step(timestep) != -1:
    
    v = gps.getValues()
    n = compass.getValues()

    if math.isnan(v[0]) or math.isnan(n[0]):
        continue
    
    x = v[0]          
    y = v[1]         
    
    angle_north = math.atan2(n[1], n[0])
    theta = (math.pi / 2.0) - angle_north
    theta = math.atan2(math.sin(theta), math.cos(theta)) 

    target = waypoints[current_wp]
    left, right, distance = compute_control(x, y, theta, target)

    MAX_SPEED = 3.14
    left = max(min(left, MAX_SPEED), -MAX_SPEED)
    right = max(min(right, MAX_SPEED), -MAX_SPEED)

    fl = ps_front_left.getValue()
    fr = ps_front_right.getValue()
    l = ps_left.getValue()
    r = ps_right.getValue()

    front_blocked = fl > 150.0 or fr > 150.0
    right_blocked = r > 150.0
    left_blocked = l > 150.0

    A = target[1] - prev_wp[1]
    B = prev_wp[0] - target[0]
    C = (target[0] * prev_wp[1]) - (prev_wp[0] * target[1])
    denominator = math.sqrt(A**2 + B**2)
    m_line_dist = abs(A*x + B*y + C) / denominator if denominator != 0 else 0

    # --- STATE MACHINE ---
    if state == "NAVIGATING":
        if front_blocked:
            state = "AVOIDING"
            hit_dist_to_target = distance
            
            target_angle = math.atan2(target[1] - y, target[0] - x)
            angle_diff = math.atan2(math.sin(target_angle - theta), math.cos(target_angle - theta))
            
            if right_blocked and not left_blocked:
                follow_direction = "RIGHT" 
            elif left_blocked and not right_blocked:
                follow_direction = "LEFT"
            elif angle_diff > 0:
                follow_direction = "RIGHT"
            else:
                follow_direction = "LEFT"
    
    elif state == "AVOIDING":
        # Calculate if we are physically pointing at the target
        target_angle = math.atan2(target[1] - y, target[0] - x)
        heading_error = math.atan2(math.sin(target_angle - theta), math.cos(target_angle - theta))
        facing_target = abs(heading_error) < 0.5  # roughly 30 degrees

        # Escape Condition 1: Bug2 M-Line (Lowered threshold for small objects)
        bug2_escape = (distance < hit_dist_to_target - 0.02) and (m_line_dist < 0.15)
        
        # Escape Condition 2: Tangent Bug Clear Sightline
        clear_escape = facing_target and not front_blocked and not left_blocked and not right_blocked

        # Execute escape if either condition is met and the front is clear
        if (bug2_escape or clear_escape) and not front_blocked:
            state = "NAVIGATING"

    if state == "AVOIDING":
        left, right = compute_wall_follow(fl, fr, l, r, threshold=150.0, max_speed=MAX_SPEED, follow_dir=follow_direction)

    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

    step_count += 1
    if step_count % 50 == 0:
        print(f"[{state}] Pos: ({x:.2f}, {y:.2f}) | M-Line: {m_line_dist:.2f} | TgtDist: {distance:.2f} | Dir: {follow_direction}")

    if distance < 0.20:
        print(f">>> REACHED WAYPOINT {current_wp}")
        state = "NAVIGATING"
        prev_wp = waypoints[current_wp]
        current_wp = (current_wp + 1) % len(waypoints)