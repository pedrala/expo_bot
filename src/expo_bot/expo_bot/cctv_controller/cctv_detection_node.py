import json
import time
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # OpenCV 이미지를 ROS2 이미지 메시지로 변환
from sensor_msgs.msg import CompressedImage
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

# CCTVCrowdDetectionNode 클래스 정의
class CCTVCrowdDetectionNode(Node):
    def __init__(self):
        super().__init__('cctv_detection_node')  # 노드 이름 설정

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT 또는 RELIABLE
            depth=10
        )

        self.crowd_image_publisher_ = self.create_publisher(CompressedImage, '/crowd_image', qos_profile)  #CompressedImage 이미지 퍼블리셔 생성 
        self.detected_object_publisher_ = self.create_publisher(String, '/detected_object', qos_profile) #객체인식된 정보 퍼블리셔 생성
      
        self.json_data = []  # 데이터를 저장할 리스트 초기화
        self.bridge = CvBridge()  # CvBridge 인스턴스 생성S
        self.blue_box_margin = 50  # 파란색 바운딩 박스의 여백 (픽셀 단위)
        self.model = YOLO('./src/expo_bot/resource/cctv.pt')  # YOLO 모델 로드
        self.cap = cv2.VideoCapture(0)  # 카메라 사용
        self.last_alert = None  # 마지막 알림 시간 추적 (초기화)
   
        self.json_data = []  # JSON 파일에 저장할 데이터를 위한 리스트

    def run_detection(self):
        while True:
            success, img = self.cap.read()  # 카메라로부터 이미지 읽기
            results = self.model(img, stream=True)  # YOLO 모델을 이용해 객체 감지

            # 파란색 바운딩 박스의 경계 정의
            height, width, _ = img.shape
            margin = self.blue_box_margin
            bbox_x1, bbox_y1 = margin, margin
            bbox_x2, bbox_y2 = width - margin, height - margin

            # 파란색 바운딩 박스를 화면에 그림
            cv2.rectangle(img, (bbox_x1, bbox_y1), (bbox_x2, bbox_y2), (255, 0, 0), 2)

            object_count = 0  # 물체 카운트 초기화

            for r in results:
                boxes = r.boxes  # 감지된 객체들의 바운딩 박스 정보

                for box in boxes:
                    # 바운딩 박스 좌표 추출
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    center_x = (x1 + x2) // 2  # 객체 중심 x 좌표
                    center_y = (y1 + y2) // 2  # 객체 중심 y 좌표
                    confidence = float(box.conf[0].item())  # 객체의 정확도 (Tensor -> float 변환)
                    class_id = int(box.cls[0].item())  # 객체의 클래스 ID (Tensor -> int 변환)

                    # 클래스 이름을 가져오기 (예: 'car', 'person', etc.)
                    class_name = self.model.names[class_id]

                    # 객체의 중심이 파란색 바운딩 박스 내부에 있는지 확인
                    if bbox_x1 < center_x < bbox_x2 and bbox_y1 < center_y < bbox_y2:
                        # 객체가 바운딩 박스 내부에 들어오면, 색상 반전
                        img[y1:y2, x1:x2] = cv2.bitwise_not(img[y1:y2, x1:x2])

                        # 최근에 알림을 보낸 시간과 비교하여, 일정 시간 간격으로만 알림을 보냄
                        current_time = time.time()
                        if self.last_alert is None or current_time - self.last_alert >= 1:  # 1초 간격
                            # 침입 알림: 객체의 중심 위치를 퍼블리시
                            intrusion_location = Point(x=float(center_x), y=float(center_y), z=0.0)
                            self.get_logger().info(f"물체가 바운딩 박스 안으로 들어옴: 위치 ({center_x}, {center_y})")
                            self.last_alert = current_time  # 알림 보낸 시간 업데이트

                        # 객체 카운트 증가
                        object_count += 1

                    # 객체의 바운딩 박스를 화면에 그림
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # 클래스 이름과 confidence를 표시
                    label = f"{class_name} {confidence:.2f}"
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 물체가 3개 이상인 경우에만 이미지 저장
            if object_count >= 3 and confidence > 0.7:

                # JSON 파일에 객체 정보 저장
                self.json_data.append({        
                    'class': class_name,  
                    'confidence': confidence,                 
                    'object_count': object_count,
                    'zone': 1,
                    'time': current_time
                })

                # JSON 데이터를 문자열로 변환 후 퍼블리시
                json_str = json.dumps(self.json_data)  # JSON 문자열로 변환
                msg = String()  # String 메시지 객체 생성
                msg.data = json_str
                self.detected_object_publisher_.publish(msg) # 객체 인식된 정보 퍼블리셔 생성

            
            # 이미지를 ROS2 이미지 메시지로 변환하여 퍼블리시
            # 이미지 압축 후 퍼블리시
            _, encoded_image = cv2.imencode('.jpg', img)  # 이미지 JPEG 형식으로 압축
            image_message = CompressedImage()
            image_message.header.stamp = self.get_clock().now().to_msg()  # to_msg()로 Time 객체 변환
            image_message.format = "jpeg"
            image_message.data = np.array(encoded_image).tobytes()  # 압축된 이미지를 바이트로 변환

            #image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.crowd_image_publisher_.publish(image_message)

            # 화면에 결과 표시
            #cv2.imshow('crowd_location', img)

            # if cv2.waitKey(1) == ord('q'):
            #     break

        # 카메라 자원 해제 및 창 닫기
        self.cap.release()
        cv2.destroyAllWindows()


# ROS2 노드 실행 함수
def main():
    rclpy.init()
    node = CCTVCrowdDetectionNode()  # CCTVCrowdDetectionNode 객체 생성
    node.run_detection()  # 객체 탐지 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
