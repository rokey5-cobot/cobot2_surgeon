import rclpy
from rclpy.node import Node
# od_msg가 없으면 빌드 에러가 날 수 있습니다. 에러 시 std_srvs 등으로 대체 필요
from od_msg.srv import SrvDepthPosition 

class MockVisionServer(Node):
    def __init__(self):
        super().__init__('mock_vision_server')
        self.srv = self.create_service(SrvDepthPosition, '/get_3d_position', self.handle_vision)
        self.get_logger().info('👀 Mock Vision Server Ready.')

    def handle_vision(self, request, response):
        target = request.target
        self.get_logger().info(f"Request received for: {target}")

        if target == 'hand':
            # [가짜 손 좌표] 카메라 기준 (단위: mm)
            # Z가 깊이입니다. 로봇이 뻗었을 때 닿을 수 있는 위치로 설정하세요.
            response.depth_position = [50.0, 50.0, 400.0] 
            self.get_logger().warn(f"Returning MOCK HAND position: {response.depth_position}")
        
        elif target in ['hammer', 'cutter', 'driver']:
            # [가짜 도구 좌표]
            response.depth_position = [-50.0, -50.0, 500.0]
            self.get_logger().warn(f"Returning MOCK TOOL position: {response.depth_position}")
            
        else:
            # 인식 실패
            response.depth_position = [0.0, 0.0, 0.0]
        
        return response

def main():
    rclpy.init()
    node = MockVisionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
