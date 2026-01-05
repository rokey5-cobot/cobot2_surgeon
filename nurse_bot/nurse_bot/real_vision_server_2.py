import os
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from od_msg.srv import SrvDepthPosition

# 사용자님이 보여주신 ImgNode 클래스를 그대로 가져왔습니다.
# (단, 토픽 이름에 /camera가 두 번 들어간 것 같아서 일반적인 환경에 맞춰 하나로 조정했습니다. 
# 만약 실제 토픽이 /camera/camera/...라면 수정해주세요.)
class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None

        # RealSense Launch 토픽 구독
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

    def camera_info_callback(self, msg):
        # Intrinsic Matrix (K) = [fx, 0, ppx, 0, fy, ppy, 0, 0, 1]
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

class RealVisionServer(ImgNode):  # ImgNode를 상속받아서 기능 확장
    def __init__(self):
        super().__init__() # ImgNode의 초기화(구독 설정) 실행
        
        # 1. 모델 경로 설정
        pkg_path = get_package_share_directory("nurse_bot")
        tool_model = os.path.join(pkg_path, "resource", "tool_best.pt")
        hand_model = os.path.join(pkg_path, "resource", "hand_best.pt") # 파일명 확인!

        # 2. 모델 로드
        self.get_logger().info("Loading YOLO Models...")
        self.model_tool = YOLO(tool_model)
        self.get_logger().info(f"📋 학습된 도구 목록: {self.model_tool.names}")
        try:
            self.model_hand = YOLO(hand_model)
            self.get_logger().info("Hand Model Loaded")
        except:
            self.get_logger().warn(" Hand Model load failed. Check filename.")
            self.model_hand = None

        self.detected_box = None
        self.detected_label = ""
        self.create_timer(0.1, self.display_timer_callback)
        # self.create_timer(0.1, self.debug_display_callback)

        # 3. 서비스 서버 생성
        self.srv = self.create_service(SrvDepthPosition, '/get_3d_position', self.detect_callback)
        self.get_logger().info("👀 Real Vision Ready! Waiting for command...")

    # def debug_display_callback(self):
    #     """명령이 없어도 항상 화면에 인식 결과를 그려주는 함수"""
    #     if self.color_frame is None:
    #         return

    #     # 화면 복사
    #     display_img = self.color_frame.copy()
        
    #     # ★★★ 여기서 항상 도구 모델을 돌립니다 (테스트용) ★★★
    #     if self.model_tool:
    #         # 0.25 (25%) 이상이면 무조건 표시
    #         results = self.model_tool(self.color_frame, verbose=False, conf=0.25)
            
    #         for r in results:
    #             for box in r.boxes:
    #                 # 좌표
    #                 x1, y1, x2, y2 = map(int, box.xyxy[0])
    #                 # 이름 및 확률
    #                 cls_name = self.model_tool.names[int(box.cls[0])]
    #                 conf = float(box.conf[0])

    #                 # 박스 그리기 (보라색)
    #                 cv2.rectangle(display_img, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    
    #                 # 텍스트 그리기 (이름 + 확률)
    #                 label = f"{cls_name} {conf:.2f}"
    #                 cv2.putText(display_img, label, (x1, y1 - 10),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    #     # 창 띄우기
    #     cv2.imshow("Nurse Bot Debug View", display_img)
    #     cv2.waitKey(1)
    def display_timer_callback(self):
        """실시간 화면을 띄워주는 함수"""
        if self.color_frame is not None:
            # 원본 이미지 복사
            display_img = self.color_frame.copy()

            # 마지막으로 인식된 박스가 있으면 그리기
            if self.detected_box is not None:
                x1, y1, x2, y2 = self.detected_box
                # 초록색 박스
                cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 텍스트 (라벨)
                cv2.putText(display_img, self.detected_label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 창 띄우기
            cv2.imshow("Nurse Bot Camera", display_img)
            cv2.waitKey(1)


    def detect_callback(self, request, response):
        target = request.target
        self.get_logger().info(f" Request: Find '{target}'")

        if self.color_frame is None or self.depth_frame is None or self.intrinsics is None:
            self.get_logger().warn(" Waiting for Camera Data...")
            return response

        # 4. 모델 스위칭 (도구 vs 손)
        if target.lower() in ['palm']:
            model = self.model_hand
            confi = 0.6
            ioun = 0.2
            m = True
            self.get_logger().info("👉 Using HAND Model")
        else:
            model = self.model_tool
            confi = 0.7
            ioun = 0.6
            m = False
            self.get_logger().info("🔧 Using TOOL Model")

        if model is None: return response

        # 5. 추론
        results = model(self.color_frame, conf=confi, iou=ioun, verbose=False)
        
        if m:
            # 6. 결과 분석 및 좌표 계산
            box_data = None
            max_conf = 0.0

            for r in results:
                for box in r.boxes:
                    # 클래스 이름 매칭 확인
                    cls_name = model.names[int(box.cls[0])]
                    is_match = (target.lower() in cls_name.lower())
                    
                    if is_match:
                        if box.conf[0] > max_conf:
                            max_conf = box.conf[0]
                            box_data = box.xyxy[0].cpu().numpy()

            if box_data is not None:
                x1, y1, x2, y2 = map(int, box_data)
                u, v = (x1 + x2) // 2, (y1 + y2) // 2

                # 깊이 가져오기 (mm)
                z_mm = float(self.depth_frame[v, u])
                
                if z_mm > 0:
                    # 3D 변환 공식
                    fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
                    cx, cy = self.intrinsics['ppx'], self.intrinsics['ppy']

                    x_val = (u - cx) * z_mm / fx
                    y_val = (v - cy) * z_mm / fy

                    self.get_logger().info(f"🎯 Found at: X={x_val:.1f}, Y={y_val:.1f}, Z={z_mm:.1f}")
                    response.depth_position = [x_val, y_val, z_mm]
                else:
                    self.get_logger().warn("Depth is 0 (Too close/far)")
            else:
                self.get_logger().info("❌ Not Found")

            return response
        else:
            # keypoint의 중앙점
            if results[0].keypoints is not None:
                keypoints = results[0].keypoints.xy.cpu().numpy()

                for i, tools_keypoints in enumerate(keypoints):

                    if len(tools_keypoints) >= 2:
                        x1, y1 = map(int, tools_keypoints[0])
                        x2, y2 = map(int, tools_keypoints[1])
                        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                        z_mm = float(self.depth_frame[center_y, center_x])

                        if z_mm > 0:
                            # 3D 변환 공식
                            fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
                            cx, cy = self.intrinsics['ppx'], self.intrinsics['ppy']

                            x_val = (center_x - cx) * z_mm / fx
                            y_val = (center_y - cy) * z_mm / fy

                            self.get_logger().info(f"🎯 Found at: X={x_val:.1f}, Y={y_val:.1f}, Z={z_mm:.1f}")
                            response.depth_position = [x_val, y_val, z_mm]
                        else:
                            self.get_logger().warn("Depth is 0 (Too close/far)")
                    else:
                        self.get_logger().warn('keypoint 2개 안 잡힘', len(tools_keypoints))

def main():
    rclpy.init()
    node = RealVisionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
