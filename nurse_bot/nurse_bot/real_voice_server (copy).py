import os
import re
import tempfile
import numpy as np
import scipy.io.wavfile as wav
import sounddevice as sd
from dotenv import load_dotenv
from pathlib import Path
from openai import OpenAI

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

# .env 파일 로드
# load_dotenv(Path(__file__).resolve().parent / ".env")
OPENAI_API_KEY = "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"

class STTMacVAD:
    def __init__(self, api_key: str, samplerate: int = 16000, device=None):
        self.client = OpenAI(api_key=api_key)
        self.samplerate = samplerate
        self.device = device
        self.chunk_duration = 0.2
        self.silence_threshold = 0.02
        self.silence_duration = 1.0
        
        # ★★★ [수정 1] 수술실 -> 공구 작업 환경으로 변경 ★★★
        self.domain_prompt = "작업 공구 요청. 도구: 망치(해머), 드라이버, 니퍼, 가위, 커터칼. 위치: pos1, pos2."

    @staticmethod
    def normalize_text(text: str) -> str:
        if not text: return text
        
        # ★★★ [수정 2] 한국어 발음을 YOLO 영어 클래스명으로 매핑 ★★★
        # 학습된 목록: {cutter, driver, hammer, nipper, scissors}
        replace_map = {
            # 망치 -> hammer
            "망치": "hammer", "해머": "hammer", "장도리": "hammer",
            # 드라이버 -> driver
            "드라이버": "driver", "십자": "driver", "일자": "driver", "스크류": "driver",
            # 니퍼 -> nipper
            "니퍼": "nipper", "뺀찌": "nipper", "펜치": "nipper",
            # 가위 -> scissors
            "가위": "scissors", "자르는거": "scissors",
            # 커터 -> cutter
            "커터": "cutter", "커터칼": "cutter", "칼": "cutter",
            
            # 위치 및 기타 정규화
            "일번방": "1번방", "이번방": "2번방", "포지션원": "pos1", "포지션투": "pos2"
        }
        
        norm = text.strip()
        for k, v in replace_map.items():
            norm = norm.replace(k, v)
        return norm

    def record_until_silence(self) -> np.ndarray:
        print("🎙️ 듣고 있습니다... (명령: '망치 줘', '니퍼 1번방' 등)")
        frames = []
        silent_time = 0.0
        with sd.InputStream(samplerate=self.samplerate, channels=1, dtype="float32", device=self.device) as stream:
            while True:
                chunk, _ = stream.read(int(self.chunk_duration * self.samplerate))
                frames.append(chunk.copy())
                rms = float(np.sqrt(np.mean(chunk ** 2)))
                if rms < self.silence_threshold:
                    silent_time += self.chunk_duration
                else:
                    silent_time = 0.0
                if silent_time >= self.silence_duration:
                    break
        audio = np.concatenate(frames, axis=0)
        return (audio * 32767).clip(-32768, 32767).astype("int16")

    def speech2text(self) -> str:
        audio_i16 = self.record_until_silence()
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tf:
            path = tf.name
        try:
            wav.write(path, self.samplerate, audio_i16)
            with open(path, "rb") as f:
                tr = self.client.audio.transcriptions.create(model="whisper-1", file=f, prompt=self.domain_prompt, language="ko")
            return self.normalize_text(tr.text)
        finally:
            os.remove(path)

class ExtractKeywordTools:
    def __init__(self, api_key: str):
        self.client = OpenAI(api_key=api_key)

    def extract(self, text: str) -> str:
        # ★★★ [수정 3] GPT에게 너의 모델 클래스 이름만 뱉으라고 강요 ★★★
        prompt = f"""
        사용자 입력: "{text}"
        
        너는 로봇 제어 시스템이다. 다음 규칙에 맞춰 정보를 추출하라.
        
        1. Tools (반드시 아래 단어 중 하나로 변환):
           - cutter, driver, hammer, nipper, scissors
        
        2. Positions:
           - pos1, pos2, pos3
           
        3. Room:
           - room1, room2, room3, room4
           
        4. Surgery:
           - (공구 작업이므로 비워둠)
           
        Output Format: [tool / pos / room / surgery]
        Example: "[hammer / pos1 / / ]"
        """
        resp = self.client.chat.completions.create(
            model="gpt-4o", messages=[{"role": "user", "content": prompt}], temperature=0.0
        )
        return resp.choices[0].message.content.strip()

class RealVoiceServer(Node):
    def __init__(self):
        super().__init__('real_voice_server')
        if not OPENAI_API_KEY:
            self.get_logger().error("❌ OPENAI_API_KEY Missing! .env 파일을 확인하세요.")
            raise ValueError("No API Key")
        
        self.stt = STTMacVAD(api_key=OPENAI_API_KEY)
        self.extractor = ExtractKeywordTools(api_key=OPENAI_API_KEY)
        
        self.srv = self.create_service(Trigger, '/get_keyword', self.handle_voice_request)
        self.get_logger().info("👂 공구 인식 보이스 준비 완료! (말씀하세요)")

    def handle_voice_request(self, request, response):
        self.get_logger().info("🗣️ 듣는 중...")
        
        try:
            raw_text = self.stt.speech2text()
            self.get_logger().info(f"📝 인식된 문장: '{raw_text}'")
        except Exception as e:
            self.get_logger().error(f"STT Error: {e}")
            response.success = False
            return response

        if not raw_text:
            response.success = False
            return response

        try:
            extracted_data = self.extractor.extract(raw_text)
            self.get_logger().info(f"🧠 추출 결과: {extracted_data}")
            
            response.success = True
            response.message = extracted_data
        except Exception as e:
            self.get_logger().error(f"GPT Error: {e}")
            response.success = False
            
        return response

def main():
    rclpy.init()
    node = RealVoiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
