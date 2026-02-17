from controller import Robot, Motor, DistanceSensor, Camera, InertialUnit
import threading
import time
import numpy as np
import cv2

TIME_STEP = 32
MAX_SPEED = 10.0
WALL_DIST_TARGET = 0.20
CORNER_THRESHOLD = 0.25
RPI_LATENCY = 0.2

class SplitBrainRobot:
    def __init__(self):
        self.robot = Robot()
        
        self.shared = {
            "status": "IDLE", "cmd": None, "camera_frame": None,
            "lock": threading.Lock()
        }

        self.init_hardware()
        
        self.orb = cv2.ORB_create(nfeatures=500)
        self.ref_descriptors = None
        self.state = "START"

    def init_hardware(self):
        self.m1 = self.robot.getDevice('motor1')
        self.m3 = self.robot.getDevice('motor3')
        self.m2 = self.robot.getDevice('motor2')
        self.m4 = self.robot.getDevice('motor4')

        for m in [self.m1, self.m2, self.m3, self.m4]:
            m.setPosition(float('inf'))
            m.setVelocity(0)

        self.tof_front = self.robot.getDevice('ds_left')
        self.tof_front.enable(TIME_STEP)
        
        self.tof_wall = self.robot.getDevice('ds_front')
        self.tof_wall.enable(TIME_STEP)

        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(TIME_STEP)
        
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(TIME_STEP * 2)

    def run_rpi_logic(self):
        print("[RPi] Booted. Waiting for camera warm-up...")
        time.sleep(1.0)
        
        while True:
            time.sleep(RPI_LATENCY)
            
            img_np = None
            status = ""
            with self.shared["lock"]:
                if self.shared["camera_frame"] is not None:
                    raw = self.shared["camera_frame"]
                    img_np = np.frombuffer(raw, np.uint8).reshape(
                        (self.camera.getHeight(), self.camera.getWidth(), 4)
                    )[:, :, :3]
                    img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
                status = self.shared["status"]

            if img_np is None: continue

            if np.sum(img_np[:,:,2] > 200) > 5000:
                print("[RPi] THREAT DETECTED! Sending STOP.")
                with self.shared["lock"]:
                    self.shared["cmd"] = "STOP"
                continue

            if status == "AT_CORNER_WAITING":
                print("[RPi] Verifying Location...")
                gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)
                kp, des = self.orb.detectAndCompute(gray, None)
                
                if self.ref_descriptors is None:
                    print("[RPi] Setting Global Anchor.")
                    self.ref_descriptors = des
                    cmd = "CONTINUE"
                else:
                    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                    matches = bf.match(self.ref_descriptors, des)
                    
                    if len(matches) > 15:
                        print(f"[RPi] Loop Closed! ({len(matches)} matches). Correcting Drift.")
                        cmd = "CORRECT_POS"
                    else:
                        print(f"[RPi] No Match ({len(matches)}). Continuing.")
                        cmd = "CONTINUE"
                
                with self.shared["lock"]:
                    self.shared["cmd"] = cmd
            
            with self.shared["lock"]:
                self.shared["camera_frame"] = None

    def run_esp32_logic(self):
        rpi_thread = threading.Thread(target=self.run_rpi_logic)
        rpi_thread.daemon = True
        rpi_thread.start()

        print("[ESP32] Systems Online. Following RIGHT Wall.")
        
        kp = 10.0
        prev_error = 0
        
        while self.robot.step(TIME_STEP) != -1:
            
            d_front = self.tof_front.getValue()
            d_right = self.tof_wall.getValue()
            
            img = self.camera.getImage()
            with self.shared["lock"]:
                self.shared["camera_frame"] = img
                cmd = self.shared["cmd"]

            if cmd == "STOP":
                self.set_4wd_vel(0, 0)
                continue
            elif cmd in ["CONTINUE", "CORRECT_POS"]:
                with self.shared["lock"]: self.shared["cmd"] = None
                self.state = "TURN_LEFT"

            if self.state == "START":
                self.state = "WALL_FOLLOW"

            elif self.state == "WALL_FOLLOW":
                if d_front < CORNER_THRESHOLD:
                    self.set_4wd_vel(0, 0)
                    self.state = "CHECKPOINT"
                    print("[ESP32] Corner Ahead. Waiting for RPi.")
                else:
                    error = WALL_DIST_TARGET - d_right
                    correction = kp * error
                    
                    base_speed = MAX_SPEED * 0.5
                    left_speed = base_speed + correction
                    right_speed = base_speed - correction
                    
                    self.set_4wd_vel(left_speed, right_speed)

            elif self.state == "CHECKPOINT":
                self.set_4wd_vel(0, 0)
                with self.shared["lock"]:
                    self.shared["status"] = "AT_CORNER_WAITING"

            elif self.state == "TURN_LEFT":
                self.set_4wd_vel(-2.0, 2.0)
                if d_front > CORNER_THRESHOLD + 0.1:
                    self.state = "WALL_FOLLOW"
                    with self.shared["lock"]: self.shared["status"] = "MOVING"

    def set_4wd_vel(self, left, right):
        l = min(MAX_SPEED, max(-MAX_SPEED, left))
        r = min(MAX_SPEED, max(-MAX_SPEED, right))
        
        self.m1.setVelocity(l)
        self.m3.setVelocity(l)
        self.m2.setVelocity(r)
        self.m4.setVelocity(r)

if __name__ == "__main__":
    bot = SplitBrainRobot()
    bot.run_esp32_logic()
