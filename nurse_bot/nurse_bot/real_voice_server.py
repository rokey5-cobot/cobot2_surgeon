import os
import tempfile
import numpy as np
from openai import OpenAI
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Bool

# API 키 (새로 발급받은 키로 꼭 바꾸세요!)
# OPENAI_API_KEY =
 

class RealVoiceServer(Node):
    def __init__(self):
        super().__init__('real_voice_server')
        self.client = OpenAI(api_key=OPENAI_API_KEY)
        self.cmd_publisher = self.create_publisher(String, '/web_command', 10)
        self.create_subscription(String, '/process_audio_file', self.audio_file_callback, 10)
        self.get_logger().info("🎤 수술실 AI 서버 준비 완료!")

    @staticmethod
    def normalize_text(text: str) -> str:
        if not text: return text
        
        replace_map = {
            # 도구 정규화
            "망치": "hammer", "해머": "hammer",
            "드라이버": "driver", "스크류": "driver",
            "니퍼": "nipper", "뺀찌": "nipper",
            "가위": "scissors",
            "커터": "cutter", "커터칼": "cutter", "칼": "cutter",
            
            # ★★★ [추가] 수술 이름 정규화 ★★★
            "맹장": "appendicitis", "맹장수술": "appendicitis",
            "골절": "fracture", "정형외과": "fracture", "뼈": "fracture",
            "봉합": "suture", "꿰매": "suture",
            
            # 방 번호
            "1번방": "room1", "2번방": "room2", "일번방": "room1", "이번방" : "room2" ,
            "3번방": "room3", "4번방": "room4", "삼번방": "room3", "사번방" : "room4" ,
            # ★ 종료 관련 단어 정규화
            "끝": "finish", "종료": "finish", "마무리": "finish", "그만": "finish",

            "멈춰": "stop", "정지": "stop", "스톱": "stop", "비상": "stop", "위험해": "stop",
            "복구": "recover", "리셋": "recover", "원위치": "recover", "다시": "recover", "홈으로": "recover"
        }
        
        norm = text.strip()
        for k, v in replace_map.items():
            norm = norm.replace(k, v)
        return norm

    def extract(self, text: str) -> str:
        # ★★★ [추가] GPT에게 수술(Surgery)도 뽑으라고 시킴 ★★★
        prompt = f"""
        사용자 입력: "{text}"
        
        [규칙]
        1. Tool: hammer, driver, nipper, scissors, cutter 중 하나.
        2. Surgery: appendicitis(맹장), fracture(골절), suture(봉합) 중 하나.
        3. Room: room1, room2, room3, room4.
        4. Position: 사용자가 '손', '여기' 등 위치를 명확히 말했을 때만 dest:hand 사용.
           (위치 언급이 없으면 dest는 생략할 것)
        5. Command: 비상 정지(stop), 복구(recover), 종료(end).

        [출력 형식]
        - 수술 요청: "surgery:수술명 / room:방번호"
        - 단순 도구 요청(위치X): "tool:도구명"  <-- 이렇게 깔끔하게!
        - 도구+위치 요청(위치O): "tool:도구명 / dest:위치"
        - 종료 요청: "cmd:end"
        - 비상 정지: "cmd:stop"
        - 복구 요청: "cmd:recover"
        
        [예시]
        "망치 줘" -> "tool:hammer"
        "망치 손으로 줘" -> "tool:hammer / dest:hand"
        "1번방에 가위 놔" -> "tool:scissors / dest:room1"
        "맹장 수술 준비" -> "surgery:appendicitis"
        "수술 끝났어" -> "cmd:end"
        "멈춰!" -> "cmd:stop"
        "로봇 복구해" -> "cmd:recover"
        """
        try:
            resp = self.client.chat.completions.create(
                model="gpt-4o", messages=[{"role": "user", "content": prompt}], temperature=0.0
            )
            return resp.choices[0].message.content.strip()
        except Exception as e:
            self.get_logger().error(f"GPT Error: {e}")
            return None

    def process_stt(self, file_path):
        # 1. 파일이 진짜 있는지, 크기는 정상인지 확인
        if not os.path.exists(file_path):
            self.get_logger().error(f"❌ 파일이 없음: {file_path}")
            return None
            
        file_size = os.path.getsize(file_path)
        self.get_logger().info(f"📁 파일 크기 확인: {file_size} bytes")
        
        if file_size < 100:
            self.get_logger().error("❌ 파일이 너무 작음 (녹음 실패 의심)")
            return None

        try:
            with open(file_path, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f, language="ko",
                    prompt="수술 준비: 맹장, 골절, 봉합. 도구: 망치, 가위, 커터칼."
                )
            return transcript.text
        except Exception as e: 
            # ★ 여기가 핵심입니다. 에러 메시지를 빨간색으로 크게 출력!
            self.get_logger().error(f"❌ STT 치명적 오류 발생!\n👉 원인: {e}")
            return None

    def audio_file_callback(self, msg):
        file_path = msg.data
        self.get_logger().info(f"📨 파일 경로 수신됨: {file_path}")
        if not os.path.exists(file_path): return

        raw_text = self.process_stt(file_path)
        if not raw_text: 
            self.get_logger().warn("⚠️ 아무 말도 인식되지 않았거나 STT 에러 발생")
            return
        self.get_logger().info(f"👂 들음: {raw_text}")

        norm_text = self.normalize_text(raw_text)
        extracted_data = self.extract(norm_text)
        self.get_logger().info(f"🧠 분석: {extracted_data}")

        if extracted_data:
            self.parse_and_publish(extracted_data)

    def parse_and_publish(self, gpt_output):
        try:
            # 특수문자 제거
            clean_str = gpt_output.replace('[', '').replace(']', '').replace('"', '').replace("'", "")
            self.cmd_publisher.publish(String(data=clean_str))
            self.get_logger().info(f"📤 전송: {clean_str}")
            # 그대로 로봇에게 전송 (로봇이 알아서 파싱함)
            # 예: "surgery:appendicitis / room:room1"
            # parts = [p.strip() for p in clean_str.split('/')]
            
            # for part in parts:
            #     if part:
            #         self.cmd_publisher.publish(String(data=part))
            #         self.get_logger().info(f"📤 전송: {part}")


        except Exception as e:
            self.get_logger().error(f"파싱 에러: {e}")

def main():
    rclpy.init()
    node = RealVoiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
