import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template_string, request
import threading
import json

# --- 1. Flask 웹 서버 설정 ---
app = Flask(__name__)

# 폰 화면 디자인 (마이크 버튼 추가 + JS 음성 인식 로직)
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Nurse Bot Voice Remote</title>
    <style>
        body { font-family: 'Apple SD Gothic Neo', sans-serif; text-align: center; background-color: #222; color: white; margin: 0; padding: 20px; }
        .mic-btn {
            width: 120px; height: 120px; border-radius: 50%;
            background-color: #F44336; color: white; border: none;
            font-size: 40px; box-shadow: 0 0 20px rgba(244, 67, 54, 0.6);
            margin: 30px auto; display: block; cursor: pointer; transition: 0.2s;
        }
        .mic-btn.listening { background-color: #4CAF50; box-shadow: 0 0 30px #4CAF50; transform: scale(1.1); }
        .log-box { background: #333; padding: 15px; border-radius: 10px; font-size: 16px; min-height: 60px; margin-bottom: 20px; border: 1px solid #555; }
        .sub-text { color: #aaa; font-size: 14px; margin-top: 5px; }
        button { padding: 15px; font-size: 16px; border-radius: 10px; background: #555; color: white; border: none; margin: 5px; }
    </style>
</head>
<body>
    <h1>🎙️ 로봇 무전기</h1>
    
    <div class="log-box">
        <div id="status">버튼을 누르고 말씀하세요...</div>
        <div id="result" style="color: #0f0; font-weight: bold; margin-top: 10px;"></div>
    </div>

    <button id="micBtn" class="mic-btn" onclick="toggleSpeech()">🎙️</button>
    <div class="sub-text" id="guideText">터치해서 말하기</div>

    <hr style="margin: 30px 0; border-color: #444;">

    <div>
        <button onclick="sendText('망치 1번방')">🔨 망치 1번방</button>
        <button onclick="sendText('손으로 줘')">🤲 손으로 줘</button>
    </div>

    <script>
        // 1. 웹 브라우저 음성 인식 API 설정
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        const recognition = new SpeechRecognition();
        
        recognition.lang = 'ko-KR'; // 한국어 설정
        recognition.interimResults = false; // 중간 결과 끄기 (완성된 문장만)
        recognition.maxAlternatives = 1;

        const micBtn = document.getElementById('micBtn');
        const statusDiv = document.getElementById('status');
        const resultDiv = document.getElementById('result');
        const guideText = document.getElementById('guideText');
        let isListening = false;

        // 2. 음성 인식 시작/종료 함수
        function toggleSpeech() {
            if (isListening) {
                recognition.stop();
            } else {
                recognition.start();
            }
        }

        // 3. 이벤트 리스너들
        recognition.onstart = () => {
            isListening = true;
            micBtn.classList.add('listening');
            statusDiv.innerText = "👂 듣고 있어요! 말씀하세요.";
            guideText.innerText = "말이 끝나면 자동으로 전송됩니다";
        };

        recognition.onend = () => {
            isListening = false;
            micBtn.classList.remove('listening');
            statusDiv.innerText = "대기 중...";
            guideText.innerText = "터치해서 말하기";
        };

        recognition.onresult = (event) => {
            const transcript = event.results[0][0].transcript;
            resultDiv.innerText = "인식됨: " + transcript;
            
            // 서버로 텍스트 전송
            sendText(transcript);
        };

        recognition.onerror = (event) => {
            statusDiv.innerText = "에러: " + event.error;
        };

        // 4. 서버 전송 함수 (AJAX)
        function sendText(text) {
            statusDiv.innerText = "🚀 로봇에게 전송 중...";
            fetch('/voice_cmd', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ text: text })
            })
            .then(res => res.text())
            .then(data => {
                statusDiv.innerText = "✅ 로봇 응답: " + data;
            });
        }
    </script>
</body>
</html>
"""

# --- 2. ROS 2 노드 ---
class WebVoiceNode(Node):
    def __init__(self):
        super().__init__('web_voice_node')
        # Controller가 듣는 토픽으로 발행
        self.publisher_ = self.create_publisher(String, '/web_command', 10)
        self.get_logger().info('🌐 스마트폰 음성 서버 시작됨!')

    def pub_command(self, text):
        msg = String()
        # 간단한 파싱 로직 (필요하면 GPT 붙일 수 있음)
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Voice Command: {text}')

ros_node = None

# --- 3. Flask 라우팅 ---
@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/voice_cmd', methods=['POST'])
def voice_cmd():
    global ros_node
    data = request.json
    raw_text = data.get('text', '')
    
    print(f"🎤 폰에서 받은 음성: {raw_text}")
    
    # [간단한 키워드 매핑 로직]
    # 실제로는 여기서 GPT를 부르거나, Controller에서 처리하게 할 수 있습니다.
    command_str = ""
    
    # 1. 도구 매핑
    if "망치" in raw_text or "해머" in raw_text: command_str = "tool:hammer"
    elif "드라이버" in raw_text: command_str = "tool:driver"
    elif "니퍼" in raw_text: command_str = "tool:nipper"
    elif "가위" in raw_text: command_str = "tool:scissors"
    
    # 2. 위치/행동 매핑 (명령어 덮어쓰기 주의 - 실제론 조합해야 함)
    if "손" in raw_text or "달라" in raw_text: command_str = "dest:hand"
    elif "1번" in raw_text: command_str = "dest:pos1"
    
    if ros_node and command_str:
        ros_node.pub_command(command_str)
        return f"명령 확인 ({command_str})"
    elif ros_node:
        # 매핑 안 된 말도 일단 보냄 (Controller가 로그라도 찍게)
        ros_node.pub_command(f"raw:{raw_text}")
        return f"전송됨 ({raw_text})"
    
    return "Node Error"

# --- 4. 메인 실행 ---
def ros_spin_thread():
    rclpy.spin(ros_node)

def main():
    global ros_node
    rclpy.init()
    ros_node = WebVoiceNode()
    
    t = threading.Thread(target=ros_spin_thread)
    t.start()
    
    print("=========================================")
    print("📱 폰 크롬을 켜고 접속하세요: http://<로봇PC_IP>:5000")
    print("⚠️ 주의: 아이폰(Safari)이나 일부 안드로이드 크롬은")
    print("   'https'가 아니면 마이크 권한을 막을 수 있습니다.")
    print("   안되면 PC 크롬에서 먼저 테스트해보세요.")
    print("=========================================")
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
