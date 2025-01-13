from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import sqlite3
import json
import cv2
from flask import Flask, render_template, Response
import threading
import os
import numpy as np
from geometry_msgs.msg import Point
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Flask App 설정
# BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# TEMPLATE_DIR = os.path.join(BASE_DIR, 'templates')
# app = Flask(__name__, template_folder=TEMPLATE_DIR)

app = Flask(__name__, template_folder=os.path.join(os.path.dirname('/home/viator/ws/chob3_ws/src/expo_bot/expo_bot/templates'), 'templates'))

class SysMonitorNode(Node):
    def __init__(self):
        super().__init__('sys_monitor_node')
        
        # 데이터베이스 접근 스레드 안전을 위한 락
        self.db_lock = threading.Lock()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT 또는 RELIABLE
            depth=10
        )

        # ROS2 토픽 구독 설정
        self.subscription1 = self.create_subscription(
            CompressedImage, 'crowd_image', self.crowd_image_callback, 10)
        self.subscription2 = self.create_subscription(
            CompressedImage, '/amr_camera_image', self.amr_image_callback, qos_profile)
        self.json_subscription = self.create_subscription(
            String, '/detected_object', self.json_callback, 10)
        
        #목표좌표 퍼블리셔
        self.target_coordinate_publisher_ = self.create_publisher(Point, '/target_coordinate', qos_profile) 

        # CvBridge 초기화
        self.bridge = CvBridge()
        self.latest_frame1 = None  # 최신 프레임 저장
        self.latest_frame2 = None

        # SQLite3 데이터베이스 초기화
        self.conn = sqlite3.connect('asset/detected_objects.db', check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.initialize_database()

        # 목표 좌표 5개 이상 리스트에 저장하고 하나씩 action 으로 실행하도록 구현
        self.get_target_positions_from_file()
              

    def initialize_database(self):
        # 테이블 생성 및 컬럼 확인
        with self.db_lock:
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS detected_objects (
                    class TEXT,
                    confidence REAL,
                    object_count INTEGER,
                    zone INTEGER,
                    time REAL
                )
            ''')
            self.conn.commit()

    def crowd_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            with self.db_lock:  # Lock을 사용하여 thread safety 확보
                self.latest_frame1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode crowd image: {e}")

    def amr_image_callback(self, msg):
        try:
            print("amr_camera_image:" + msg.data)
            np_arr = np.frombuffer(msg.data, np.uint8)
            with self.db_lock:  # Lock을 사용하여 thread safety 확보
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if img is None:
                    self.get_logger().warn("Failed to decode image")
                else:
                    self.latest_frame2 = img
                    self.get_logger().info("Received new image frame")

                self.latest_frame2 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode AMR image: {e}")

    def json_callback(self, msg):
        try:
            data = json.loads(msg.data)
            with self.db_lock:
                for item in data:
                    self.cursor.execute('''
                        INSERT INTO detected_objects (class, confidence, object_count, zone, time)
                        VALUES (?, ?, ?, ?, ?)
                    ''', (item.get('class'), item.get('confidence'), item.get('object_count'),
                          item.get('zone'), item.get('time')))
                self.conn.commit()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON Decode Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error saving data: {e}")

    def get_target_positions_from_file(self):
        # goal_position.json 파일에서 데이터 읽기
        file_path = './src/expo_bot/resource/goal_position.json'
        try:
            with open(file_path, "r", encoding='utf-8') as file:
                json_data = json.load(file)
                
                if len(json_data) >= 5:
                    target_positions = []
                    # 가장 최근의 5개 데이터 가져오기
                    for target_data in json_data[-5:]:
                        # Pose 데이터를 가져오는데 'pose' 내부의 'pose' -> 'position'을 참고
                        position = target_data['pose']['pose']['position']
                        
                        target_position = Point(
                            x=position['x'],  # x 좌표
                            y=position['y'],  # y 좌표
                            z=position['z']   # z 좌표
                        )
                        target_positions.append(target_position)

                    self.get_logger().info(f"목표 좌표 5개 전송 중...")
                    # 각 목표 좌표를 퍼블리시
                    for target_position in target_positions:
                        self.target_coordinate_publisher_.publish(target_position)
                else:
                    self.get_logger().warn(f"goal_position.json에 데이터가 충분하지 않습니다.")
        except Exception as e:
            self.get_logger().error(f"파일을 읽는 중 오류 발생: {e}")
            

    def destroy_node(self):
        self.conn.close()
        super().destroy_node()


# Flask 이미지 스트리밍 제너레이터
def generate_frames1(latest_frame_getter):
    while True:
        frame = latest_frame_getter()
        if frame is not None:
            try:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           jpeg.tobytes() + b'\r\n')
            except Exception as e:
                app.logger.error(f"Error encoding frame: {e}")
        else:
            continue

# Flask 이미지 스트리밍 제너레이터
def generate_frames2(latest_frame_getter):
    while True:
        frame = latest_frame_getter()
        if frame is not None:
            try:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           jpeg.tobytes() + b'\r\n')
            except Exception as e:
                app.logger.error(f"Error encoding frame: {e}")
        else:
            continue


@app.route('/')
def index():
    with sys_monitor_node.db_lock:
        sys_monitor_node.cursor.execute('''
            SELECT class, confidence, object_count, zone, time
            FROM detected_objects
            ORDER BY time DESC
            LIMIT 10
        ''')
        detected_data = sys_monitor_node.cursor.fetchall()

    formatted_data = [
        (class_name, confidence, object_count, zone,
         datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S'))
        for class_name, confidence, object_count, zone, timestamp in detected_data
    ]
    return render_template('two_cam_index.html', detected_data=formatted_data)

@app.route('/video_feed1')
def video_feed1():
    return Response(generate_frames1(lambda: sys_monitor_node.latest_frame1),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    return Response(generate_frames2(lambda: sys_monitor_node.latest_frame2),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_detected_data')
def get_detected_data():
    with sys_monitor_node.db_lock:
        sys_monitor_node.cursor.execute('''
            SELECT class, confidence, object_count, zone, time
            FROM detected_objects
            ORDER BY time DESC
            LIMIT 10
        ''')
        detected_data = sys_monitor_node.cursor.fetchall()

    formatted_data = [
        {
            'class_name': class_name,
            'confidence': confidence,
            'object_count': object_count,
            'zone': zone,
            'time': datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
        }
        for class_name, confidence, object_count, zone, timestamp in detected_data
    ]
    return json.dumps(formatted_data)

def main():
    rclpy.init()
    global sys_monitor_node
    sys_monitor_node = SysMonitorNode()

    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5311, debug=False, use_reloader=False))
    flask_thread.start()

    try:
        rclpy.spin(sys_monitor_node)
    except Exception as e:
        sys_monitor_node.get_logger().error(f"Exception during spin: {e}")
    finally:
        sys_monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
