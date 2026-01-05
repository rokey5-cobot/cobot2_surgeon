import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MockVoiceServer(Node):
    def __init__(self):
        super().__init__('mock_voice_server')
        self.srv = self.create_service(Trigger, '/get_keyword', self.handle_input)
        self.get_logger().info('🎤 Mock Voice Server Ready. (Will ask for input via terminal)')

    def handle_input(self, request, response):
        print("\n" + "="*40)
        print("[Mock Voice] 명령을 입력하세요 (예: hammer pos1 또는 scalpel hand)")
        user_input = input(">> ")
        print("="*40 + "\n")
        
        # 입력 형식을 파싱 (간단히 띄어쓰기로 도구/목적지 구분)
        # 예: "hammer hand" -> 도구: hammer, 목적지: hand
        parts = user_input.split()
        
        tool = parts[0] if len(parts) > 0 else "hammer"
        target = parts[1] if len(parts) > 1 else ""
        
        # 기존 포맷(도구 / 목적지)에 맞춰서 반환
        response.success = True
        response.message = f"{tool} / {target}"
        return response

def main():
    rclpy.init()
    node = MockVoiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
