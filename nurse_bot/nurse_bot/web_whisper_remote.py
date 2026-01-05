import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template_string, request
import threading
from datetime import datetime

# --- 1. Flask 웹 서버 설정 ---
app = Flask(__name__)

# (HTML 템플릿은 기존과 동일하므로 유지 - 코드 길이를 위해 생략하지 않고 그대로 둡니다)
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>High-End Voice Remote</title>
    <style>
        body { font-family: 'Apple SD Gothic Neo', sans-serif; text-align: center; background-color: #1a1a1a; color: white; margin: 0; padding: 20px; user-select: none; -webkit-user-select: none; }
        .mic-btn { width: 150px; height: 150px; border-radius: 50%; background-color: #E53935; color: white; border: none; font-size: 60px; box-shadow: 0 0 25px rgba(229, 57, 53, 0.6); margin: 50px auto; display: flex; align-items: center; justify-content: center; -webkit-tap-highlight-color: transparent; outline: none; touch-action: none; }
        .mic-btn:active, .mic-btn.recording { background-color: #ffffff; color: #E53935; transform: scale(0.95); box-shadow: 0 0 40px rgba(255, 255, 255, 0.8); }
        .log-box { background: #333; padding: 20px; border-radius: 15px; font-size: 16px; min-height: 80px; margin-bottom: 20px; border: 1px solid #444; }
        .status-text { color: #aaa; font-size: 14px; margin-top: 10px; }
        .result-text { color: #4CAF50; font-weight: bold; font-size: 18px; margin-top: 10px; word-break: keep-all; }
    </style>
</head>
<body>
    <h1>🤖 AI 음성 리모컨</h1>
    <div class="status-text">버튼을 꾹 누르고 말하세요</div>
    <div class="log-box">
        <div id="status">준비 중...</div>
        <div id="result" class="result-text"></div>
    </div>
    <button id="micBtn" class="mic-btn">🎙️</button>
    <div class="status-text" id="perm-msg">마이크 권한 확인 중...</div>

    <script>
        let mediaRecorder = null;
        let audioChunks = [];
        const micBtn = document.getElementById('micBtn');
        const statusDiv = document.getElementById('status');
        const resultDiv = document.getElementById('result');
        const permMsg = document.getElementById('perm-msg');

        async function initMic() {
            try {
                const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
                mediaRecorder = new MediaRecorder(stream);
                mediaRecorder.ondataavailable = e => audioChunks.push(e.data);
                mediaRecorder.onstop = () => {
                    const audioBlob = new Blob(audioChunks, { type: 'audio/webm' });
                    sendAudio(audioBlob);
                    audioChunks = [];
                };
                statusDiv.innerText = "준비 완료! 버튼을 누르세요";
                permMsg.innerText = "✅ 마이크 연결됨";
                permMsg.style.color = "#4CAF50";
            } catch (err) {
                statusDiv.innerText = "❌ 마이크 권한 실패";
                permMsg.innerText = "HTTPS(ngrok) 확인 필요";
                permMsg.style.color = "#E53935";
            }
        }
        initMic();

        function start(e) {
            if (e) e.preventDefault();
            if (!mediaRecorder) return;
            if (mediaRecorder.state === "inactive") {
                mediaRecorder.start();
                micBtn.classList.add('recording');
                statusDiv.innerText = "🔴 듣고 있습니다...";
                resultDiv.innerText = "";
            }
        }

        function stop(e) {
            if (e) e.preventDefault();
            if (mediaRecorder && mediaRecorder.state === "recording") {
                mediaRecorder.stop();
                micBtn.classList.remove('recording');
                statusDiv.innerText = "⏳ 서버로 전송 중...";
            }
        }

        micBtn.addEventListener('mousedown', start);
        micBtn.addEventListener('mouseup', stop);
        micBtn.addEventListener('mouseleave', stop);
        micBtn.addEventListener('touchstart', start, {passive: false});
        micBtn.addEventListener('touchend', stop, {passive: false});
    
        function sendAudio(blob) {
            const formData = new FormData();
            formData.append("audio", blob, "recording.webm");

            fetch('/upload_audio', { method: 'POST', body: formData })
            .then(r => r.json())
            .then(data => {
                statusDiv.innerText = "✅ 전송 완료";
                resultDiv.innerText = "AI 서버가 분석 중입니다...";
            })
            .catch(err => {
                statusDiv.innerText = "⚠️ 에러 발생";
            });
        }
    </script>
</body>
</html>
"""

# --- 2. ROS 2 노드 (AI 기능 제거하고 파일 경로만 전달) ---
class WebWhisperNode(Node):
    def __init__(self):
        super().__init__('web_whisper_node')
        # 파일 경로를 전달할 토픽
        self.publisher_ = self.create_publisher(String, '/process_audio_file', 10)
        self.get_logger().info('🌐 웹 리모컨 시작 (AI 처리는 RealVoiceServer로 위임)')

    def send_file_path(self, path):
        msg = String()
        msg.data = path
        self.publisher_.publish(msg)
        self.get_logger().info(f'File Sent: {path}')

ros_node = None

# --- 3. Flask ---
@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/upload_audio', methods=['POST'])
def upload_audio():
    global ros_node
    if 'audio' not in request.files: return {"status": "fail"}
    
    audio_file = request.files['audio']
    
    # 절대 경로로 저장 (ROS 노드끼리 파일 공유를 위해)
    save_dir = os.path.abspath("saved_audio")
    os.makedirs(save_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = os.path.join(save_dir, f"rec_{timestamp}.webm")
    audio_file.save(file_path)

    # ROS 토픽으로 파일 경로만 툭 던져줌
    if ros_node:
        ros_node.send_file_path(file_path)

    return {"status": "success", "path": file_path}

# --- 4. 메인 실행 ---
def ros_spin_thread():
    rclpy.spin(ros_node)

def main():
    global ros_node
    rclpy.init()
    ros_node = WebWhisperNode()
    t = threading.Thread(target=ros_spin_thread)
    t.start()
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
