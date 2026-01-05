import os
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
import colorsys
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# ROS2 메시지 및 서비스
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from od_msg.srv import SrvDepthPosition  # 서비스 메시지 타입

class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None

        # RealSense 토픽 구독
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

    def camera_info_callback(self, msg):
        self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def color_callback(self, msg):
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Color frame error: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth frame error: {e}")

class RealVisionServer(ImgNode):
    def __init__(self):
        super().__init__()
        
        # ===============================
        # 1. 모델 로드
        # ===============================
        pkg_path = get_package_share_directory("nurse_bot")
        tool_model_path = os.path.join(pkg_path, "resource", "tool_best.pt")
        hand_model_path = os.path.join(pkg_path, "resource", "hand_landmarker.task")

        self.get_logger().info("Loading Models...")
        
        # YOLO (도구용)
        try:
            self.model_tool = YOLO(tool_model_path)
            self.get_logger().info(f"🔧 Tool Model Loaded")
        except Exception as e:
            self.get_logger().error(f"Failed to load Tool Model: {e}")
            self.model_tool = None

        # MediaPipe (.task 파일용)
        try:
            base_options = python.BaseOptions(model_asset_path=hand_model_path)
            options = vision.HandLandmarkerOptions(
                base_options=base_options,
                num_hands=1,
                min_hand_detection_confidence=0.6,
                min_hand_presence_confidence=0.6,
                min_tracking_confidence=0.5,
                # 서비스 요청 시 1장만 처리하므로 IMAGE 모드 사용
                running_mode=vision.RunningMode.IMAGE 
            )
            self.landmarker = vision.HandLandmarker.create_from_options(options)
            self.get_logger().info("👉 MediaPipe HandLandmarker (.task) Loaded")
        except Exception as e:
            self.get_logger().error(f"Failed to load Hand Task: {e}")
            self.landmarker = None

        # 시각화용 설정 (무지개색 관절)
        self.HAND_CONNECTIONS = [
	    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
	    (0, 9), (9, 10), (10, 11), (11, 12), (0, 13), (13, 14), (14, 15), (15, 16),
	    (0, 17), (17, 18), (18, 19), (19, 20)
	]
        self.landmark_colors = []
        for i in range(21):
            r, g, b = colorsys.hsv_to_rgb(i / 21.0, 1.0, 1.0)
            self.landmark_colors.append((int(b * 255), int(g * 255), int(r * 255)))

        # 데이터 공유 변수 (마지막 감지 결과 저장용)
        self.detected_point = None      
        self.detected_label = ""        
        self.latest_hand_landmarks = [] 
        self.latest_tool_kpts = []  
        self.detected_angle = 0.0
        self.latest_bbox = None    

        # ===============================
        # 2. 서비스 서버 생성
        # ===============================
        self.srv = self.create_service(SrvDepthPosition, '/get_3d_position', self.detect_callback)
        
        # 화면 갱신용 타이머 (인식 로직 없음, 단순히 그림만 그림)
        self.create_timer(0.1, self.display_timer_callback)
        
        self.get_logger().info("👀 Real Vision Service Ready! Waiting for Request...")

    def detect_callback(self, request, response):
        """서비스 요청이 들어왔을 때만 실행되는 함수"""
        target = request.target
        self.get_logger().info(f"📥 Request Received: Find '{target}'")

        if self.color_frame is None or self.depth_frame is None or self.intrinsics is None:
            self.get_logger().warn("⚠️ Camera frames not ready yet.")
            return response

        # 초기화 (새로운 요청이 왔으므로 이전 결과 초기화)
        u, v = 0, 0
        z_mm = 0.0
        found = False
        self.detected_point = None
        self.latest_hand_landmarks = []
        self.latest_tool_kpts = []
        self.detected_label = ""
        self.detected_angle = 0.0
        self.latest_bbox = None
         # ★★★ [추가된 기능] 거리 측정 모드 (measure) ★★★
        if target == "measure":
            h, w = self.depth_frame.shape
            u, v = w // 2, h // 3 # 화면 정중앙 좌표
            
            # 중앙 픽셀의 깊이값 가져오기 (mm 단위)
            z_mm = float(self.depth_frame[v, u])
            
            if z_mm > 0:
                self.get_logger().info(f"📏 바닥까지 거리 측정됨: {z_mm}mm")
                # X, Y는 0으로 보내고 Z(깊이)만 채워서 보냄
                response.depth_position = [0.0, 0.0, z_mm]
                return response
            else:
                self.get_logger().warn("⚠️ 거리 측정 실패 (너무 가깝거나 멉니다)")
                return response
        # ==========================================
        # CASE 1: 손 인식 (MediaPipe)
        # ==========================================
        if target.lower() in ['hand']:
            if self.landmarker:
                img_rgb = cv2.cvtColor(self.color_frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
                
                # 추론 실행
                detection_result = self.landmarker.detect(mp_image)

                if detection_result.hand_landmarks:
                    hand_landmarks = detection_result.hand_landmarks[0]
                    self.latest_hand_landmarks = hand_landmarks # 시각화용 데이터 업데이트
                    self.detected_label = "Hand"
                    
                    h, w, _ = self.color_frame.shape
                    # Wrist(0), Index(5), Pinky(17) Center
                    idx_points = [0, 5, 17]
                    sum_x, sum_y = 0, 0
                    for idx in idx_points:
                        lm = hand_landmarks[idx]
                        sum_x += int(lm.x * w)
                        sum_y += int(lm.y * h)
                    
                    u = sum_x // 3
                    v = sum_y // 3
                    found = True
                else:
                    self.get_logger().info("❌ Hand not found")

        # ==========================================
        # CASE 2: 도구 인식 (YOLO Pose)
        # ==========================================
        else:
            if self.model_tool:
                results = self.model_tool(self.color_frame, conf=0.5, iou=0.75, classes=[0,1,2,3], verbose=False)
                best_conf = 0.0
                best_center = None
                
                if results[0].boxes and results[0].keypoints is not None:
                    boxes = results[0].boxes
                    keypoints = results[0].keypoints.xy.cpu().numpy()

                    for i, box in enumerate(boxes):
                        cls_name = self.model_tool.names[int(box.cls[0])]
                        # 요청한 target 이름이 포함되는지 확인
                        if target.lower() in cls_name.lower():
                            kpts = keypoints[i]
                            # 키포인트 2개 이상일 때만 (양 끝점 가정)
                            if len(kpts) >= 2 and box.conf[0] > best_conf:
                                x1, y1 = map(int, kpts[0])
                                x2, y2 = map(int, kpts[1])
                                
                                self.latest_tool_kpts = [(x1, y1), (x2, y2)]
                                self.detected_label = f"Tool: {target}"
                                
                                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                                best_center = (cx, cy)
                                best_conf = float(box.conf[0])

                                bx1, by1, bx2, by2 = map(int, box.xyxy[0])
                                self.latest_bbox = (bx1, by1, bx2, by2)
                
                if best_center:
                    u, v = best_center
                    found = True
                else:
                    self.get_logger().info(f"❌ Tool '{target}' not found")

        # ==========================================
        # 3D 좌표 변환 및 응답 반환
        # ==========================================
        if found:
            # Depth 체크
            if 0 <= u < self.depth_frame.shape[1] and 0 <= v < self.depth_frame.shape[0]:
                z_mm = float(self.depth_frame[v, u])
            else:
                z_mm = 0.0

            if z_mm > 0:
                fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
                cx, cy = self.intrinsics['ppx'], self.intrinsics['ppy']

                x_val = (u - cx) * z_mm / fx
                y_val = (v - cy) * z_mm / fy

                self.get_logger().info(f"🎯 Success [{target}]: ({x_val:.1f}, {y_val:.1f}, {z_mm:.1f})")
                
                # 서비스 응답에 좌표 입력
                response.depth_position = [x_val, y_val, z_mm]
                
                # 시각화용 좌표 업데이트
                self.detected_point = (u, v)
            else:
                self.get_logger().warn("⚠️ Detected but Depth is 0")
        
        return response

    def display_timer_callback(self):
        """인식 결과 시각화 (인식 로직은 없음)"""
        if self.color_frame is None: return

        display_img = self.color_frame.copy()
        h, w, _ = display_img.shape

        # 1. 손 랜드마크 그리기
        if self.latest_hand_landmarks:
            for start_idx, end_idx in self.HAND_CONNECTIONS:
                p1 = self.latest_hand_landmarks[start_idx]
                p2 = self.latest_hand_landmarks[end_idx]
                cv2.line(display_img, (int(p1.x*w), int(p1.y*h)), (int(p2.x*w), int(p2.y*h)), (200, 200, 200), 2)
            for i, lm in enumerate(self.latest_hand_landmarks):
                cv2.circle(display_img, (int(lm.x*w), int(lm.y*h)), 5, self.landmark_colors[i], -1)

        # 2. 도구 키포인트 그리기
        for (tx, ty) in self.latest_tool_kpts:
            cv2.circle(display_img, (tx, ty), 5, (0, 255, 255), -1)


        # 3. 타겟 좌표 및 라벨 그리기
        if self.detected_point:
            u, v = self.detected_point
            cv2.circle(display_img, (u, v), 8, (0, 255, 0), -1)
            cv2.circle(display_img, (u, v), 12, (0, 255, 0), 2)
            cv2.putText(display_img, self.detected_label, (u + 15, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("Nurse Bot Service View", display_img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = RealVisionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
