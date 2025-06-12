#!/usr/bin/env python3
import sys
import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QProgressBar, QTextEdit
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QObject

class RosNode(Node, QObject):
    """ROS2 노드와 PyQt 시그널을 처리하는 클래스"""
    lane_image_signal = pyqtSignal(np.ndarray)
    raw_compressed_image_signal = pyqtSignal(np.ndarray)
    raw_image_signal = pyqtSignal(np.ndarray)
    velocity_signal = pyqtSignal(float, float)
    drive_mode_signal = pyqtSignal(str)
    user_name_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str)
    fsm_state_signal = pyqtSignal(str)

    def __init__(self):
        Node.__init__(self, 'autorace_gui_node')
        QObject.__init__(self)
        
        self.cv_bridge = CvBridge()
        
        qos_profile_sensor_data = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_profile_transient_local = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.create_subscription(CompressedImage, '/detect/image_lane/compressed', self.lane_image_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.raw_compressed_image_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.raw_image_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        self.create_subscription(String, '/control/current_drive_mode', self.drive_mode_callback, 10)
        self.create_subscription(String, '/control/user_name', self.user_name_callback, qos_profile=qos_profile_transient_local)
        self.create_subscription(String, '/control/fsm_state', self.fsm_state_callback, 10)
            
        self.log_signal.emit("GUI 노드 초기화 완료.")

    def lane_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.lane_image_signal.emit(cv_image)
        except Exception as e:
            self.log_signal.emit(f"[Error] Lane Image: {e}")

    def raw_compressed_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.raw_compressed_image_signal.emit(cv_image)
        except Exception as e:
            self.log_signal.emit(f"[Error] Raw Compressed Image: {e}")

    def raw_image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.raw_image_signal.emit(cv_image)
        except Exception as e:
            self.log_signal.emit(f"[Error] Raw Image: {e}")

    def velocity_callback(self, msg):
        self.velocity_signal.emit(msg.linear.x, msg.angular.z)

    def drive_mode_callback(self, msg):
        self.drive_mode_signal.emit(msg.data)

    def user_name_callback(self, msg):
        self.user_name_signal.emit(msg.data)

    def fsm_state_callback(self, msg):
        self.fsm_state_signal.emit(msg.data)
        self.log_signal.emit(f"[FSM State] {msg.data}")

class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.mode_colors = {
            'eco': {'active': '#27ae60', 'inactive': '#ecf0f1'},
            'comfort': {'active': '#2980b9', 'inactive': '#ecf0f1'},
            'sport': {'active': '#c0392b', 'inactive': '#ecf0f1'}
        }
        self.neutral_bg_color = '#2c3e50'
        self.init_ui()
        self.connect_signals()
        
    def init_ui(self):
        self.setWindowTitle('AutoRace Monitoring Dashboard')
        self.setGeometry(100, 100, 1280, 720)
        self.setStyleSheet(f"background-color: {self.neutral_bg_color};")

        main_layout = QVBoxLayout()
        video_layout = QHBoxLayout()
        status_layout = QGridLayout()

        top_bar_layout = QHBoxLayout()
        title_label = QLabel("AutoRace Monitoring Dashboard")
        title_label.setFont(QFont('Arial', 16, QFont.Bold))
        title_label.setStyleSheet("color: white;")
        self.user_name_label = QLabel("User: Not logged in")
        self.user_name_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.user_name_label.setStyleSheet("color: #f1c40f;")
        self.user_name_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        top_bar_layout.addWidget(title_label)
        top_bar_layout.addStretch(1)
        top_bar_layout.addWidget(self.user_name_label)
        
        main_layout.addLayout(top_bar_layout)
        main_layout.addLayout(video_layout, stretch=5)
        main_layout.addLayout(status_layout, stretch=2)

        self.video_lane = self.create_video_widget('/detect/image_lane/compressed')
        self.video_raw_compressed = self.create_video_widget('/camera/image_raw/compressed')
        self.video_raw = self.create_video_widget('/camera/image_raw')
        
        video_layout.addWidget(self.video_lane['group_box'])
        video_layout.addWidget(self.video_raw_compressed['group_box'])
        video_layout.addWidget(self.video_raw['group_box'])

        # 주행모드 라벨들
        self.mode_labels = {
            'eco': self.create_styled_label('ECO'), 
            'comfort': self.create_styled_label('COMFORT'), 
            'sport': self.create_styled_label('SPORT')
        }
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(self.mode_labels['eco'])
        mode_layout.addWidget(self.mode_labels['comfort'])
        mode_layout.addWidget(self.mode_labels['sport'])
        
        # 속도 게이지
        self.linear_gauge, linear_layout = self.create_gauge('Linear Speed', 0, 200) 
        self.angular_gauge, angular_layout = self.create_gauge('Angular Speed', -2000, 2000)

        self.speed_text_label = QLabel('Linear: 0.0 m/s, Angular: 0.0 rad/s')
        self.speed_text_label.setAlignment(Qt.AlignCenter)
        self.speed_text_label.setFont(QFont('Arial', 12))
        self.speed_text_label.setStyleSheet("color: white;")

        # FSM 이벤트 로그용 QTextEdit 추가 (주행모드 영역 옆)
        self.fsm_log_box = QTextEdit()
        self.fsm_log_box.setReadOnly(True)
        self.fsm_log_box.setStyleSheet("background-color: #222; color: #f39c12; border-radius: 5px;")
        self.fsm_log_box.setFont(QFont('Courier New', 9))
        self.fsm_log_box.setMinimumHeight(50)
        self.fsm_log_box.setPlaceholderText("FSM Event Log")

        # 주행모드 영역 크기 조정: mode_layout 절반 너비 할당
        fsm_and_mode_layout = QHBoxLayout()
        fsm_and_mode_layout.addLayout(mode_layout, 1)  # 주행모드 표시 (절반 너비)
        fsm_and_mode_layout.addWidget(self.fsm_log_box, 1)  # FSM 이벤트 로그 (절반 너비)

        # status_layout 첫 행에 fsm_and_mode_layout 배치
        status_layout.addLayout(fsm_and_mode_layout, 0, 0, 1, 2)
        status_layout.addLayout(linear_layout, 1, 0)
        status_layout.addLayout(angular_layout, 1, 1)
        status_layout.addWidget(self.speed_text_label, 2, 0, 1, 2)

        # 기존 로그 박스는 그대로 유지
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("background-color: #34495e; color: white;")
        self.log_box.setFont(QFont('Courier New', 9))
        status_layout.addWidget(self.log_box, 3, 0, 1, 2)

        self.update_drive_mode('')  # 초기에는 아무 모드도 활성화하지 않음
        self.setLayout(main_layout)
        self.show()

    def create_video_widget(self, title):
        from PyQt5.QtWidgets import QGroupBox
        group_box = QGroupBox(title)
        group_box.setFont(QFont('Arial', 10, QFont.Bold))
        group_box.setStyleSheet("QGroupBox { color: white; border: 1px solid gray; margin-top: 0.5em; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }")
        vbox = QVBoxLayout()
        label = QLabel("Waiting for image...")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("background-color: black; color: white; border-radius: 5px;")
        vbox.addWidget(label)
        group_box.setLayout(vbox)
        return {'group_box': group_box, 'label': label}

    def create_styled_label(self, text):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont('Arial', 14, QFont.Bold))
        label.setMinimumHeight(50)
        return label

    def create_gauge(self, name, min_val, max_val):
        layout = QVBoxLayout()
        label = QLabel(name)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("color: white;")
        progress = QProgressBar()
        progress.setRange(min_val, max_val)
        progress.setTextVisible(False)
        progress.setStyleSheet("""
            QProgressBar { border: 1px solid grey; border-radius: 5px; background-color: #566573; }
            QProgressBar::chunk { background-color: #f39c12; }
        """)
        layout.addWidget(label)
        layout.addWidget(progress)
        return progress, layout

    def connect_signals(self):
        self.ros_node.lane_image_signal.connect(self.update_lane_image)
        self.ros_node.raw_compressed_image_signal.connect(self.update_raw_compressed_image)
        self.ros_node.raw_image_signal.connect(self.update_raw_image)
        self.ros_node.velocity_signal.connect(self.update_velocity)
        self.ros_node.drive_mode_signal.connect(self.update_drive_mode)
        self.ros_node.user_name_signal.connect(self.update_user_name)
        self.ros_node.log_signal.connect(self.update_log)
        self.ros_node.fsm_state_signal.connect(self.update_fsm_log)

    @pyqtSlot(np.ndarray)
    def update_lane_image(self, cv_img):
        self.update_image(self.video_lane['label'], cv_img)

    @pyqtSlot(np.ndarray)
    def update_raw_compressed_image(self, cv_img):
        self.update_image(self.video_raw_compressed['label'], cv_img)

    @pyqtSlot(np.ndarray)
    def update_raw_image(self, cv_img):
        self.update_image(self.video_raw['label'], cv_img)

    def update_image(self, label, cv_img):
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img).scaled(label.width(), label.height(), Qt.KeepAspectRatio)
        label.setPixmap(pixmap)

    @pyqtSlot(float, float)
    def update_velocity(self, linear, angular):
        lin_val = int(linear * 100)  # m/s -> 0~200 (예시 스케일)
        ang_val = int(angular * 1000)  # rad/s -> -2000~2000 (예시 스케일)
        self.linear_gauge.setValue(max(0, min(200, lin_val)))
        self.angular_gauge.setValue(max(-2000, min(2000, ang_val)))
        self.speed_text_label.setText(f'Linear: {linear:.2f} m/s, Angular: {angular:.2f} rad/s')

    @pyqtSlot(str)
    def update_drive_mode(self, mode):
        mode = mode.lower()
        for m, label in self.mode_labels.items():
            if m == mode:
                label.setStyleSheet(f"background-color: {self.mode_colors[m]['active']}; color: white; border-radius: 10px;")
            else:
                label.setStyleSheet(f"background-color: {self.mode_colors[m]['inactive']}; color: black; border-radius: 10px;")

    @pyqtSlot(str)
    def update_user_name(self, user_name):
        self.user_name_label.setText(f"User: {user_name}")

    @pyqtSlot(str)
    def update_log(self, text):
        self.log_box.append(text)
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    @pyqtSlot(str)
    def update_fsm_log(self, fsm_text):
        self.fsm_log_box.append(fsm_text)
        self.fsm_log_box.verticalScrollBar().setValue(self.fsm_log_box.verticalScrollBar().maximum())

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    ros_node = RosNode()
    main_win = MainWindow(ros_node)

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    exit_code = app.exec()
    rclpy.shutdown()
    ros_thread.join()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
