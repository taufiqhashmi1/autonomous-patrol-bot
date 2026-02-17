import sys
from pathlib import Path
import math
from controller import Robot

sys.path.append(str(Path(__file__).resolve().parent))

from kml_parser import load_kml
from geo_convert import convert_to_local
from navigation import compute_control

robot = Robot()
timestep = int(robot.getBasicTimeStep())
project_path = robot.getProjectPath()

gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

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

print("Controller started. ENU Coordinate mapping active...")

while robot.step(timestep) != -1:
    
    v = gps.getValues()
    n = compass.getValues()

    # SAFETY GATE: Prevent crash loop from uninitialized Webots sensors
    if math.isnan(v[0]) or math.isnan(n[0]):
        continue
    
    # ENU Frame: Index 0 is East (X), Index 1 is North (Y), Index 2 is Up (Altitude)
    x = v[0]          
    y = v[1]         
    
    # Compass Yaw calculation for ENU world
    angle_north = math.atan2(n[1], n[0])
    theta = (math.pi / 2.0) - angle_north
    theta = math.atan2(math.sin(theta), math.cos(theta)) 

    target = waypoints[current_wp]
    left, right, distance = compute_control(x, y, theta, target)

    MAX_SPEED = 6.28
    left = max(min(left, MAX_SPEED), -MAX_SPEED)
    right = max(min(right, MAX_SPEED), -MAX_SPEED)

    left_motor.setVelocity(left)
    right_motor.setVelocity(right)

    step_count += 1
    if step_count % 50 == 0:
        print(f"Pos: ({x:.2f}, {y:.2f}) | Tgt: ({target[0]:.2f}, {target[1]:.2f}) | Head: {math.degrees(theta):.0f}° | L/R: {left:.1f}/{right:.1f}")

    if distance < 0.15:
        print(f">>> REACHED WAYPOINT {current_wp}")
        current_wp = (current_wp + 1) % len(waypoints)