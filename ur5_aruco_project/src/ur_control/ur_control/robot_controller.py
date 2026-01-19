import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # --- KONFIGURACJA ---
        self.target_id = 3      # <--- ZMIANA NA ID 3
        self.move_step = 0.2    # Krok ruchu
        self.debug_window_name = "Podglad Sterowania"
        
        # Publisher & Subscriber
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        self.current_joints = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.br = CvBridge()
        
        # Słownik 4x4
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters()
        
        # Logika startowa
        self.homing_done = False
        self.connection_cycles = 0

        self.get_logger().info(f"Robot Controller Started! Target ID: {self.target_id}")

    def joint_state_callback(self, msg):
        temp_joints = {}
        for name, pos in zip(msg.name, msg.position):
            temp_joints[name] = pos
        
        try:
            self.current_joints = [temp_joints[name] for name in self.joint_names]
            
            # --- BAZOWANIE ---
            if not self.homing_done:
                subs = self.traj_pub.get_subscription_count()
                if subs > 0:
                    self.connection_cycles += 1
                    # Czekamy chwilę na stabilizację
                    if self.connection_cycles > 60:
                        self.move_to_home_position()
                        self.homing_done = True
                else:
                    self.connection_cycles = 0
        except KeyError:
            pass

    def move_to_home_position(self):
        self.get_logger().info(">>> POZYCJA BAZOWA <<<")
        home_joints = list(self.current_joints)
        home_joints[1] = -1.57 # Ramię poziomo
        home_joints[2] = 1.57  # Łokieć w dół
        self.send_traj_point(home_joints, duration_sec=4.0)

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        # Jeśli robot nie jest gotowy, wyświetl info
        if self.current_joints is None or not self.homing_done:
            cv2.putText(frame, "OCZEKIWANIE NA ROBOTA...", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow(self.debug_window_name, frame)
            cv2.waitKey(1)
            return

        # Detekcja
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # Rysowanie linii środkowej
        height, width = frame.shape[:2]
        center_line_y = height // 2
        cv2.line(frame, (0, center_line_y), (width, center_line_y), (0, 0, 255), 2)

        target_pan = self.current_joints[0]
        detected = False
        status_text = "BRAK TAGA"
        color = (0, 0, 255)

        if ids is not None:
            # Ta funkcja rysuje kolorowe obrysy wokół WSZYSTKICH wykrytych tagów
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.target_id: # ID 3
                    detected = True
                    
                    c = corners[i][0]
                    # Środek taga
                    cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                    cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                    
                    # Dodatkowa kropka w środku
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                    if cy < center_line_y:
                        status_text = "TAG GORA -> PRZOD"
                        color = (0, 255, 0) # Zielony
                        target_pan += self.move_step
                    else:
                        status_text = "TAG DOL -> TYL"
                        color = (0, 255, 255) # Żółty
                        target_pan -= self.move_step
                    break
        
        cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow(self.debug_window_name, frame)
        cv2.waitKey(1)

        if detected:
            self.send_traj_point([target_pan] + list(self.current_joints[1:]), duration_sec=0.8)

    def send_traj_point(self, positions, duration_sec=1.0):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        msg.points = [pt]
        self.traj_pub.publish(msg)

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()