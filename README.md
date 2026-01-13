# 🏥 Smart Surgical Assistant (지능형 수술 보조 협동로봇 시스템)

> **"효율·인력·안전·운영 가시성"**이라는 수술실의 핵심 문제를 동시에 해결하는 AI·로봇 기반 통합 솔루션

![Project Banner](https://via.placeholder.com/1000x300?text=Smart+Surgical+Assistant) 
## 📖 Project Overview (프로젝트 개요)

**Smart Surgical Assistant**는 수술실 내 의료진 부족 문제와 수술 도구 관리의 비효율성을 해결하기 위해 개발되었습니다. 음성 인식과 AI 비전 기술을 활용하여 협동로봇이 집도의의 명령에 따라 수술 도구를 정확히 전달하고, 수술 후 도구의 개수를 자동으로 카운팅하여 의료 사고를 예방합니다.

### 🎯 Key Objectives (핵심 목표)
* **수술실 운영 효율화:** 반복적인 도구 전달 업무 자동화
* **안전성 강화:** 수술 도구 분실 방지 및 교차 감염 예방
* **인력난 해소:** 간호 인력의 고부가 업무 집중 지원
* **디지털화:** 수술실 현황 모니터링 및 데이터 관리

---

## 🛠 Tech Stack (기술 스택)

### Environment
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu) ![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros) ![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python)

### AI & Vision
![YOLOv11](https://img.shields.io/badge/YOLO-v11_Pose-00FFFF) ![MediaPipe](https://img.shields.io/badge/MediaPipe-Hand_Tracking-blue) ![OpenAI](https://img.shields.io/badge/OpenAI-Whisper_API-412991?logo=openai)

### Backend & Database
![Flask](https://img.shields.io/badge/Flask-Server-000000?logo=flask) ![Supabase](https://img.shields.io/badge/Supabase-Database-3ECF8E?logo=supabase) ![Node.js](https://img.shields.io/badge/Node.js-Architecture-339933?logo=nodedotjs)

### Hardware
* **Robot:** Doosan Robotics M0609 (Collaborative Robot)
* **Gripper:** OnRobot RG2
* **Camera:** Intel RealSense Depth Camera, Webcam

---

## ⚙️ System Architecture (시스템 구조)

시스템은 크게 **음성 인식(Voice), 비전(Vision), 로봇 제어(Control), 웹 모니터링(Web)** 4가지 모듈로 구성됩니다.

1.  **Voice Input:** 의료진의 음성 명령("Hammer 줘")을 마이크로 수집
2.  **STT & Intent:** OpenAI Whisper를 통해 텍스트로 변환 및 의도 파악
3.  **Vision Processing:**
    * **Object Detection:** YOLOv11-pose를 사용하여 수술 도구(Hammer, Nipper 등)의 위치 및 Keypoint 추출
    * **Hand Tracking:** MediaPipe를 통해 의료진의 손 위치를 실시간 추적
4.  **Robot Control:** ROS2 기반으로 도구를 파지(Pick)하고 의료진 손으로 전달(Place)
5.  **Dashboard:** Supabase DB와 연동하여 실시간 수술실 현황 및 도구 사용량 모니터링

---

## 💡 Key Features (핵심 기능)

### 1. 수술 도구 인식 및 파지 (Tool Detection & Pick)
* **Model:** YOLOv11-pose
* **Logic:** 도구의 양 끝점(Keypoints)을 인식하고 그 중앙 좌표를 계산하여 로봇이 정확하게 도구를 파지합니다.
* **Supported Tools:** Nipper, Hammer, Driver, Scissors, Cutter

### 2. 안전한 도구 전달 (Hand Tracking & Handover)
* **Model:** MediaPipe Hand Tracking
* **Safety:** 의료진의 손목(Wrist)과 손가락 좌표를 인식하여, 손이 로봇 근처에 있을 때만 그리퍼의 힘을 해제하여 도구를 전달합니다.

### 3. 음성 제어 및 시나리오 (Voice Command)
* 수술 종류(예: 골절 수술)에 따른 도구 세팅 명령 수행
* 개별 도구 요청("Hammer 줘") 시 즉각 반응
* 긴급 정지 명령("정지!") 인식

### 4. 수술실 모니터링 및 도구 정산 (Monitoring Dashboard)
* **Pre-op:** 수술실 사용 가능 여부 확인
* **Intra-op:** 현재 사용 중인 도구 현황 실시간 카운팅
* **Post-op:** 사용된 도구와 회수된 도구의 개수를 비교 검증(Matched/Not Matched)하여 도구 분실 방지

---

## 🚀 Performance & Issues (성능 및 해결)

### Model Selection
* **Detection:** 실시간성이 중요한 수술 환경을 고려하여 `Detectron2` 대신 추론 속도가 빠른 **`YOLOv11-pose`**를 채택했습니다. (Acc: 98% 수준)
* **Tracking:** 가벼우면서도 정확한 손 추적을 위해 `ViTPose` 대신 **`MediaPipe`**를 사용하여 CPU 부하를 최소화했습니다.

### Troubleshooting
* **Depth 인식 문제:** Nipper와 같이 얇은 도구 파지 시, 중앙 지점이 바닥으로 인식되어 로봇이 충돌하는 문제가 발생했으나, Depth 임계값(Threshold) 설정을 통해 해결했습니다.

---

## 👥 Team Members (팀 소개)

| 이름 | 역할 (Role) | 담당 업무 (Responsibilities) |
|:---:|:---:|:---|
| **김효원** | PM / UI / Voice | 음성 인식 로직 구현, DB 구축, Web UI 개발 및 연동 |
| **이효원** | Vision AI | YOLO 모델 선정 및 학습, 도구/손 탐지(Detection) 구현 |
| **전형준** | Robot Control | 로봇 제어(Pick & Place) 로직, 예외 처리, 시스템 통합 |
| **황혜인** | Data / QA | 데이터셋 선정 및 라벨링, 모델 검증 및 성능 테스트 |

---

## 📝 License & References

This project was developed as a Capstone Design project.

* **Reference Paper:** *M. Wadhwa et al., "Comparison of YOLOv8 and Detectron2...", 2023.*
* **Hardware Support:** Doosan Robotics, OnRobot

---
