import os
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from supabase import create_client, Client
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # ★ 추가됨
from rclpy.callback_groups import ReentrantCallbackGroup # ★ 추가됨
from ament_index_python.packages import get_package_share_directory
from nurse_bot.onrobot import RG
# 사용자 메시지 타입
from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from std_msgs.msg import String

# Doosan & OnRobot Libs (환경에 맞게 경로 확인 필요)
import DR_init
# from robot_control.onrobot import RG # (같은 패키지에 있다고 가정하거나 경로 수정 필요)
# 임시로 OnRobot 클래스 모의 구현 (실제 파일이 있으면 import 경로 수정하세요)
class MockGripper:
    def open_gripper(self): print("[Gripper] Open")
    def close_gripper(self): print("[Gripper] Close")
    def get_status(self): return [0] # 0: Not busy

# 실제 로봇 연결 시 아래 주석 해제 및 MockGripper 대체
# from nurse_bot.onrobot import RG 

# --- 설정 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 150, 100
DEPTH_OFFSET = -10.0 # 잡을 때 깊이 보정
HANDOVER_Z_OFFSET = 100.0 # 손보다 얼마나 위에서 줄지 (mm)

GRIPPER_IP = "192.168.1.1" # 보통 두산 로봇 내부망 IP
GRIPPER_PORT = 502
GRIPPER_TYPE = "rg2"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
#-------------------수술실위치-------------
# 로봇을 책상 왼쪽 끝에 놓고 get_current_posx()로 확인한 값을 넣으세요.
TABLE_START_POS = [200.0, -180.0, 200.0, 0.0, 180.0, 0.0] 

CLEANUP_BIN_POS = [-15.5, 37.80, 57.0, -0.92, 81.8, -16.16]
# 2. 도구 사이의 간격 (mm 단위)
ITEM_GAP = 70.0  # 10cm 간격으로 나열
#-------------수술 리스트-------------
SURGERY_RECIPES = {
    "appendicitis": ["cutter", "scissors"],  # 맹장 수술 -> 커터칼, 가위
    "fracture": ["hammer","hammer", "driver","cutter","nipper"],        # 골절 수술 -> 망치, 드라이버
    "suture": ["nipper", "scissors"]         # 봉합 수술 -> 니퍼, 가위
}
#-----------데이터 베이스에 있는 도구 리스트--------------
ALL_TOOLS = ["nipper", "scissors", "hammer", "driver", "cutter"]

def main_logic():
    rclpy.init()
    node = NurseController()
    DR_init.__dsr__node = node
    # Doosan Robot 객체 초기화 (DSR_ROBOT2)
    # 실제 로봇 연결이 안 되어 있으면 이 부분에서 에러가 날 수 있으니 
    # 시뮬레이터(DR_simulation)를 켜거나 실제 로봇 연결 필요
    try:
        global movej, movel, get_current_posx, mwait , movejx
        from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, movejx
    except ImportError:
        print("DSR_ROBOT2 Import Error! 로봇 연결 상태를 확인하세요.")
        return

   
    # 메인 루프
    while rclpy.ok():
        node.run_scenario()
        
    rclpy.shutdown()

class NurseController(Node):
    def __init__(self):
        super().__init__("nurse_controller", namespace="dsr01")

        # 1. Supabase 설정 (사용자 정보 입력 필수!)
        # ==========================================
        self.SUPABASE_URL = "https://oxbytxeozwwbctwrxhpk.supabase.co"
        self.SUPABASE_KEY = "sb_publishable_zzGU-pYBPq2AhGgEhd807g_gRDtz0to"   # 본인의 KEY 입력
        # ==========================================

        self.supabase = None
        try:
            self.supabase = create_client(self.SUPABASE_URL, self.SUPABASE_KEY)
            self.get_logger().info("✅ Supabase 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ Supabase 연결 실패: {e}")

        self.current_room_id = "room1"
        self.current_surgery_name = None
        # ★ [핵심 수정] 콜백 그룹 설정 (동시 실행 허용)
        self.callback_group = ReentrantCallbackGroup()
        # 클라이언트 설정 (callback_group 추가)
        self.cli_vision = self.create_client(SrvDepthPosition, "/get_3d_position", callback_group=self.callback_group)
        self.cli_voice = self.create_client(Trigger, "/get_keyword", callback_group=self.callback_group)
        
        # 웹 명령 구독 (callback_group 추가)
        self.create_subscription(String, '/web_command', self.web_cmd_callback, 10, callback_group=self.callback_group)
        
        # ★★★ [핵심] DB 감시용 타이머 (3초마다 실행) ★★★
        self.create_timer(3.0, self.monitor_db_cleanup, callback_group=self.callback_group)

        self.vision_pub = self.create_publisher(String, '/check', 10)
        # 리소스 경로
        self.pkg_path = get_package_share_directory("nurse_bot")
        self.calib_path = os.path.join(self.pkg_path, "resource", "T_gripper2camera.npy")
        
        self.is_busy = False

        self.gripper = None
        if RG:
            try:
                self.get_logger().info(f"🔌 그리퍼 연결 시도 ({GRIPPER_IP})...")
                self.gripper = RG(GRIPPER_TYPE, GRIPPER_IP, GRIPPER_PORT)
                self.get_logger().info("✅ OnRobot 그리퍼 연결 성공!")
            except Exception as e:
                self.get_logger().error(f"❌ 그리퍼 연결 실패: {e}")
        
        if self.gripper is None:
            self.get_logger().warn("⚠️ Mock Gripper(가상) 모드로 동작합니다.")
            self.gripper = MockGripper()

        self.get_logger().info("Nurse Robot Controller Initialized.")

    #----------------------데이터베이스-------------
    # ★★★ [수정] supabase_uploader.py 방식 적용 (Upsert) ★★★
    def init_surgery_db(self, room_id, surgery_name):
        if not self.supabase: return
        try:
            current_time = datetime.now().isoformat()
            
            # 1. Rooms 테이블: upsert로 덮어쓰기 (가장 확실함)
            room_payload = {
                "room": room_id,
                "surgery_type": surgery_name,
                "availability": "in_use",
                "updated_at": current_time
            }
            self.supabase.table("rooms").upsert(room_payload, on_conflict="room").execute()

            # 2. Tool Count 테이블: 0으로 초기화해서 덮어쓰기
            tool_payload = {
                "room": room_id,
                "updated_at": current_time
            }
            # 모든 도구 0으로 세팅
            for t in ALL_TOOLS:
                tool_payload[t] = 0
                
            self.supabase.table("tool_count").upsert(tool_payload, on_conflict="room").execute()
            
            self.get_logger().info(f"💾 DB[{room_id}] 수술 시작: {surgery_name} (Upsert 완료)")

        except Exception as e:
            self.get_logger().error(f"❌ DB 초기화 실패: {e}")
    
    # --- DB Monitor Logic ---
    def monitor_db_cleanup(self):
        """3초마다 DB를 확인해서 정리(Cleanup) 조건이 맞는지 확인"""
        if self.is_busy or not self.supabase: return
        
        try:
            # 1. 현재 방의 상태 확인 (rooms 테이블)
            room_res = self.supabase.table("rooms").select("availability").eq("room", self.current_room_id).execute()
            if not room_res.data: return
            
            room_status = room_res.data[0]['availability']
            
            # 2. 방 상태가 'Cleanup'일 때만 검사 결과 확인
            if room_status == "Cleanup":
                # check_count 테이블에서 해당 방의 '가장 최신' 결과 가져오기
                check_res = self.supabase.table("check_count")\
                    .select("check")\
                    .eq("room", self.current_room_id)\
                    .order("created_at", desc=True)\
                    .limit(1)\
                    .execute()
                
                if check_res.data:
                    is_checked = check_res.data[0]['check']
                    
                    # ★ 조건 충족: Cleanup 상태이고 + 검수가 True(통과)면 정리 시작!
                    if is_checked is True:
                        self.get_logger().info(f"🧹 [자동 감지] {self.current_room_id} 정리 시작조건 충족! (Check=True)")
                        self.perform_cleanup_sequence()

        except Exception as e:
            self.get_logger().error(f"DB Monitor Error: {e}")

    # --- Actions ---
    def perform_cleanup_sequence(self):
        """모든 도구를(중복 포함) 찾아서 정리함으로 이동"""
        self.is_busy = True
        self.get_logger().info("🧹 --- 정리(Cleanup) 모드 시작 ---")
        
        # 1. 탐색 위치로 이동 (전체가 잘 보이는 높은 위치)
        tool_clean =[-46.577, 3.021, 85.977, -0.002, 91.002, -46.942]
        scan_pose = [-90, 31.55, 36.7, 0.0, 110.3, 0.0] 
        movej(tool_clean, vel=VELOCITY, acc=ACC)
        movej(scan_pose, vel=VELOCITY, acc=ACC)
        mwait()
        # time.sleep(2.0) # 4초 정도면 충분히 이동함
        
        self.get_logger().info("✅ 이동 완료. 스캔 루프 진입")
        
        # 2. 모든 도구 타입에 대해 반복 탐색
        for tool_name in ALL_TOOLS:
            # ★★★ [핵심] while True: 해당 도구가 안 보일 때까지 계속 반복 ★★★
            while True:
                self.get_logger().info(f"🔍 '{tool_name}' 스캔 중... (남은 것 모두 처리)")
                
                # 0.5초 대기 (카메라 안정화)
                time.sleep(0.5)
                
                # 비전으로 위치 찾기
                tool_pos = self.get_vision_pos(tool_name)
                
                if tool_pos:
                    self.get_logger().info(f"🗑️ '{tool_name}' 발견! 수거하러 갑니다.")
                    
                    # 3. 집기
                    self.pick_action(tool_pos)
                    
                    # 4. 정리함 위로 이동
                    dest_over = list(CLEANUP_BIN_POS)
                    # dest_over[2] += 10.0 # 박스 위 10cm
                    movej(tool_clean, vel=VELOCITY, acc=ACC)
                    # mwait()
                    movej(dest_over, vel=VELOCITY, acc=ACC)
                    # mwait()
                    
                    # 5. 버리기 (오픈)
                    self.gripper.open_gripper()
                    time.sleep(1.0) # 떨어질 시간 주기
                    
                    # 6. 다시 스캔 위치로 복귀해서 또 있는지 확인
                    self.get_logger().info("🔄 다시 스캔 위치로 복귀...")
                    movej(tool_clean, vel=VELOCITY, acc=ACC)
                    # mwait()
                    movej(scan_pose, vel=VELOCITY, acc=ACC)
                    # mwait()
                else:
                    # 더 이상 이 도구는 안 보임 -> 다음 도구 종류로 넘어감
                    self.get_logger().info(f"✅ '{tool_name}' 더 이상 없음. 다음 도구로.")
                    break
        
        self.get_logger().info("✨ 모든 정리 완료! DB 상태를 Available로 변경합니다.")
        movej(tool_clean, vel=VELOCITY, acc=ACC)
        # 3. DB 상태 업데이트 (Cleanup -> Available)
        try:
            self.supabase.table("rooms").update({
                "availability": "available",
                "surgery_type": None, # 수술 종류 초기화
                "updated_at": datetime.now().isoformat()
            }).eq("room", self.current_room_id).execute()
            self.get_logger().info("💾 DB Update: available")
        except Exception as e:
            self.get_logger().error(f"DB Update Fail: {e}")

        # 4. 초기 위치 복귀 (퇴근 대기)
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        self.is_busy = False
    # ★★★ [수정] supabase_uploader.py의 inc_tool_counts 방식 적용 ★★★
    def increment_tool_usage(self, tool_name):
        if not self.supabase: return
        if tool_name not in ALL_TOOLS:
            return

        try:
            # 1. 기존 값 읽기 (Select)
            response = self.supabase.table("tool_count").select("*").eq("room", self.current_room_id).execute()
            
            # 기존 데이터가 있으면 가져오고, 없으면 0으로 된 딕셔너리 생성
            if response.data:
                row = response.data[0]
            else:
                row = {t: 0 for t in ALL_TOOLS}

            # 2. 값 증가 (Calculate)
            current_val = int(row.get(tool_name, 0))
            new_val = current_val + 1
            
            # 3. 전체 데이터 덮어쓰기 (Upsert)
            payload = {
                "room": self.current_room_id,
                "updated_at": datetime.now().isoformat()
            }
            # 기존 다른 도구들의 개수도 유지하면서
            for t in ALL_TOOLS:
                payload[t] = int(row.get(t, 0))
            # 이번에 쓴 도구만 +1
            payload[tool_name] = new_val

            self.supabase.table("tool_count").upsert(payload, on_conflict="room").execute()
            
            self.get_logger().info(f"💾 DB[{self.current_room_id}] '{tool_name}' 증가: {current_val} -> {new_val}")
            
        except Exception as e:
            self.get_logger().error(f"❌ 도구 카운팅 실패: {e}")
    #---------------------------------------------------------------------------    
    
    # ★ 추가: 콜백 함수
    def web_cmd_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"📱 명령 수신: {cmd}")

        if self.is_busy:
            self.get_logger().warn("⛔ 로봇 작업 중! 명령 무시.")
            return

        self.is_busy = True # 바쁨 표시

        try:
            # 1. 수술 준비 명령 (surgery:appendicitis)
            # if "surgery:" in cmd:
            #     surgery_name = cmd.split(":")[1].strip()
            #     # 룸 정보가 뒤에 따로 올 수 있으니 일단 수술부터 처리
            #     self.perform_surgery_prep(surgery_name)
            if "surgery:" in cmd:
                parts = {}
                for section in cmd.split('/'):
                    if ':' in section:
                        key, val = section.split(':')
                        parts[key.strip()] = val.strip()
                
                surgery_name = parts.get("surgery", "").strip()
                if not surgery_name and ":" in cmd: 
                     surgery_name = cmd.split(":")[1].strip()
                
                # 방 번호 파싱 및 저장
                self.current_room_id = parts.get("room", "").strip()
                if not self.current_room_id: 
                     self.current_room_id = "room1"

                self.perform_surgery_prep(surgery_name, self.current_room_id)
            elif "cmd:end" in cmd:
                self.get_logger().info(f"🏁 수술 종료 명령 수신! (방: {self.current_room_id})")
                
                # [수정 포인트] 방 이름과 Bool 정보를 합쳐서 전송 (예: "room3:True")
                msg_data = f"{self.current_room_id}:True"
                
                msg = String()
                msg.data = msg_data
                self.vision_pub.publish(msg)
                
                self.get_logger().info(f"📤 비전 노드로 전송: {msg_data}")
                
                # 2) DB 상태 업데이트 (Cleanup)
                if self.supabase:
                    try:
                        self.supabase.table("rooms").update({
                            "availability": "Cleanup",
                            "updated_at": datetime.now().isoformat()
                        }).eq("room", self.current_room_id).execute()
                    except: pass
                return
            # 2. 단일 도구 명령 (tool:hammer)
            elif "tool:" in cmd:
                tool_name = cmd.split(":")[1].strip()
                self.single_tool_delivery(tool_name)

            # 3. 기타 테스트
            elif "open" in cmd: self.gripper.open_gripper()
            elif "close" in cmd: self.gripper.close_gripper()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        
        finally:
            self.is_busy = False
            self.get_logger().info("✅ 대기 상태 (Ready)")
 
    def perform_surgery_prep(self, surgery_name, room_id):
        if surgery_name not in SURGERY_RECIPES: 
            self.get_logger().warn(f"❌ '{surgery_name}' 레시피 없음")
            return

        tool_list = SURGERY_RECIPES[surgery_name]
        self.get_logger().info(f"🏥 [{surgery_name}] 수술 세팅 시작 (방: {room_id})")

        # ★ 1. DB 초기화 (수술 시작 알림 & 카운트 리셋)
        self.init_surgery_db(room_id, surgery_name)
        # ★★★ 탐색할 위치 리스트 (관절 각도) ★★★
        # 1. 정면 (0도)
        # 2. 왼쪽 (20도)
        # 3. 오른쪽 (-20도)
        search_poses = [
            [15.03, -4.65, 98.04, -0.05, 86.6, 14.67],   # 정면
            [9.38, 23.17, 65.27, 0.13, 91.55, 9.05],  # 왼쪽으로 고개 돌림
        ]

        for i, tool in enumerate(tool_list):
            self.get_logger().info(f"🔍 '{tool}' 탐색 시작...")
            found_pos = None
            
            # --- 탐색 루프: 여러 위치를 돌면서 찾음 ---
            for pose_idx, pose in enumerate(search_poses):
                mwait()
                
                # 탐색 위치로 이동
                if pose_idx == 0: self.get_logger().info(f"👀 1. 번위치 확인")
                else: self.get_logger().info(f"👀 2.번 위치 확인")
                

                movej(pose, vel=VELOCITY, acc=ACC)
                mwait()
                time.sleep(0.5) # 카메라 초점 대기

                # 찾기 시도
                found_pos = self.get_vision_pos(tool)
                
                if found_pos:
                    self.get_logger().info(f"✨ 발견! ({pose_idx+1}번째 위치에서 '{tool}' 찾음)")
                    break # 찾았으면 탐색 루프 중단
                else:
                    self.get_logger().warn(f"💨 여기엔 '{tool}' 없음.")
            
            # --- 결과 처리 ---
            if found_pos:
                # 찾았으면 집어서 옮기기
                self.pick_action(found_pos)
                self.smart_place_action(i)
                self.get_logger().info(f"✅ {tool} 배치 완료!")
            else:
                # 다 돌아봐도 없으면 포기
                self.get_logger().error(f"❌ 모든 곳을 찾아봤지만 '{tool}'을 찾을 수 없습니다. (건너뜀)")

    def smart_place_action(self, index):
        self.get_logger().info(f"📉 {index+1}번째 도구: 정밀 착륙 시도...")

        # 1. 나열할 위치(공중)로 이동
        self.get_logger().info("🏠 자세 정렬을 위해 홈(Home) 경유")
        JReady = [0, 0, 90, 0, 90, 0] 
        movej(JReady, vel=VELOCITY, acc=ACC)
        mwait()

        # 3. 나열할 위치(공중)로 직선 이동
        # 홈에서 출발하므로 직선 경로가 확보되어 안전함
        target_pos = list(TABLE_START_POS)
        target_pos[0] += (ITEM_GAP * index) 
        
        self.get_logger().info(f"🚚 나열 위치로 이동: {target_pos}")
        movel(target_pos, vel=VELOCITY, acc=ACC) # movejx 대신 movel 사용
        mwait()
        
        # 2. 거리 측정 (Raw Distance)
        req = SrvDepthPosition.Request()
        req.target = "measure"

        floor_dist = 0.0
        # [수정 후] get_vision_pos 처럼 'spin' 방식으로 변경 (성공률 높음)
        future = self.cli_vision.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            res = future.result() # 결과 받기
            if res and res.depth_position[2] > 0:
                floor_dist = res.depth_position[2]
                self.get_logger().info(f"📏 바닥까지 남은 거리: {floor_dist}mm")
            elif res.depth_position[2]>400:
                self.get_logger().warn("⚠️ 바닥 감지 실패 (거리 0)! 기본 높이 적용.")
                floor_dist = 400.0 # 안전한 기본값
        except Exception as e:
            self.get_logger().error(f"비전 서비스 호출 에러: {e}")
            floor_dist = 400.0
        # ====================================================
        # ★★★ [핵심] Z축 거리만 좌표변환하기 ★★★
        # ====================================================
        # 1. 카메라 기준 좌표 생성 (x=0, y=0, z=거리)
        cam_point = [0.0, 0.0, floor_dist] 
        
        # 2. 현재 로봇 자세 가져오기
        cur_pos_data = get_current_posx()
        if not cur_pos_data: return
        robot_pose = cur_pos_data[0]
        
        # 3. 변환 실행 (카메라 좌표 -> 로봇 베이스 좌표)
        # "이 점이 로봇 바닥(0,0,0) 기준으로 어디에 있는가?"
        floor_point_base = self.transform_to_base(cam_point, self.calib_path, robot_pose)
        
        # 4. 바닥의 절대 높이(Z) 추출
        floor_abs_z = floor_point_base[2]
        self.get_logger().info(f"바닥의 절대 높이(Base Z): {floor_abs_z:.1f}mm")

        # ====================================================
        # 4. 목표 높이 계산 및 이동
        # ====================================================
        # 목표 높이 = 바닥 절대 높이 + 도구 길이(여유분)
        if floor_abs_z <= 0:
            target_z = 25.0
        else:    
            safety_margin = 20.0 # 도구 길이 + 여유 (10cm)
            target_z = floor_abs_z + safety_margin
        
        # 현재 위치 복사 후 Z만 수정해서 이동 ( movel )
        dest_pos = list(robot_pose)
        dest_pos[2] = target_z 
        
        self.get_logger().info(f"목표 높이 Z={target_z:.1f}mm로 하강")
        movel(dest_pos, vel=VELOCITY/2, acc=ACC)
        mwait()
        
        # 5. 놓기
        self.gripper.open_gripper()
        time.sleep(1.0)
        
        # 6. 복귀
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()

    def single_tool_delivery(self, tool_name):
        # tool_start = [270, -280, 200, 0, 180, 0]
        search_poses = [
            [270, -280, 200, 0, 180, 0],   # 1. 정면 (높은 뷰)
            [370, -280, 200, 0, 180, 0],     # 2. 왼쪽으로 고개 돌림
        ]
        found_pos = None
        for i, pose in enumerate(search_poses):
            # 이동 전 대기 (안전)
            mwait()
            
            # 해당 관절 각도로 이동
            self.get_logger().info(f"{i+1}번 위치에서 탐색 중...")
            movel(pose, vel=VELOCITY, acc=ACC)
            mwait()
            
            # 카메라 초점이 잡힐 시간 부여
            time.sleep(0.5) 

            # 비전으로 찾기
            found_pos = self.get_vision_pos(tool_name)
            
            if found_pos:
                self.get_logger().info(f"{i+1}번 위치에서 '{tool_name}' 발견!")
                break # 찾았으면 루프 탈출
            else:
                self.get_logger().warn(f"{i+1}번 위치엔 없음. 다음 위치로...")

        # 3. 결과에 따른 행동
        if found_pos:
            # 찾았으면 집고 -> 전달하고 -> 카운트 증가
            self.pick_action(found_pos) # [주의] tool_name 인자 꼭 넣기
            self.handover_action()
            self.increment_tool_usage(tool_name)
        else:
            # 다 돌아봐도 없으면 에러 로그
            self.get_logger().error(f"모든 곳을 찾아봤지만 '{tool_name}'을 찾을 수 없습니다.")
        # movel(tool_start, vel=VELOCITY, acc=ACC)
        # tool_pos = self.get_vision_pos(tool_name)
        # if tool_pos:
        #     self.pick_action(tool_pos)
        #     self.handover_action()
        #     self.increment_tool_usage(tool_name)
        # else:
        #     self.get_logger().warn(f"❌ '{tool_name}' 못 찾음")

    def run_scenario(self):
        # 1. 초기 위치 이동
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        self.gripper.open_gripper()
        
        # 2. 음성 명령 대기
        self.get_logger().info("Waiting for voice command...")
        req = Trigger.Request()
        future = self.cli_voice.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if not res.success: return

        # 파싱: "hammer / hand" -> parts=["hammer", "hand"]
        cmd_text = res.message.replace("[", "").replace("]", "").replace('"', '').replace("'", "")
        parts = [p.strip() for p in cmd_text.split('/')]

        # 안전 장치
        if len(parts) < 4: parts = parts + [""] * (4 - len(parts))

        tool_names = parts[0] # "hammer"
        destination = parts[1] # "pos1" 또는 "" (비어있음)
        
        if not tool_names:
            self.get_logger().warn("도구 이름을 못 들었어요.")
            return

        target_tool = tool_names.split()[0] # 첫 번째 도구 선택
        self.get_logger().info(f"주문: {target_tool} -> {destination}")

        # ★★★ [핵심 수정] 목적지 말 안 했으면 '손(hand)'으로 설정 ★★★
        if not destination:
            self.get_logger().info("목적지 미지정 -> '내 손'으로 자동 설정")
            destination = "hand"

        # 3. 도구 집기 (Pick)
        tool_pos = self.get_vision_pos(tool_names)
        if tool_pos:
            self.get_logger().info(f"Picking up {tool_names} at {tool_pos}")
            self.pick_action(tool_pos)
        else:
            self.get_logger().error("Failed to find tool.")
            return

        # 4. 목적지로 이동 (Handover or Place)
        if "hand" in destination or "me" in destination:
            self.handover_action()
        elif destination:
            # 특정 위치(pos1 등)로 이동 (좌표가 정의되어 있다면 구현)
            self.get_logger().info(f"Placing at {destination} (Not implemented yet)")
        else:
            self.get_logger().info("No destination. Holding object.")

    def get_vision_pos(self, target_name):
        req = SrvDepthPosition.Request()
        req.target = target_name
        future = self.cli_vision.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if not res or sum(res.depth_position) == 0:
            return None
            
        # 좌표 변환 (Camera -> Robot Base)
        cam_coords = res.depth_position
        robot_pos = get_current_posx()[0] # [x,y,z,rx,ry,rz]
        
        base_coords = self.transform_to_base(cam_coords, self.calib_path, robot_pos)
        
        # 좌표 보정
        final_pos = list(base_coords) + list(robot_pos[3:]) # 자세(rx,ry,rz)는 현재 유지
        return final_pos

    def transform_to_base(self, cam_xyz, calib_path, robot_pose):
        # (기존 robot_control.py의 로직과 동일)
        try:
            gripper2cam = np.load(calib_path)
        except:
            # 파일 없으면 단위행렬(테스트용)
            gripper2cam = np.eye(4)
            
        cam_p = np.append(np.array(cam_xyz), 1)
        
        x, y, z, rx, ry, rz = robot_pose
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T_base2grip = np.eye(4)
        T_base2grip[:3, :3] = R
        T_base2grip[:3, 3] = [x, y, z]
        
        T_base2cam = T_base2grip @ gripper2cam
        p_base = T_base2cam @ cam_p
        return p_base[:3]

    def pick_action(self, target_pos):
        # 접근 (Z 높게) -> 내려가기 -> 잡기 -> 올라오기
        self.gripper.open_gripper()
        time.sleep(0.2) # 명령 들어갈 짬 주기
        while self.gripper.get_status()[0] == 1: # 다 닫힐 때까지 대기
             time.sleep(0.1)
        approach_pos = list(target_pos)
        approach_pos[2] += 110.0 # 10cm 위
        
        movel(approach_pos, vel=VELOCITY, acc=ACC)

        pick_pos = list(target_pos)
        # ★★★ [핵심 기능] 바닥 충돌 방지 (Smart Floor Guard) ★★★
        # 로봇 좌표계 기준, 바닥이라고 판단할 높이 (예: 10mm 이하)
        FLOOR_LIMIT = 10.0  # mm (이보다 낮으면 바닥으로 간주)
        
        current_obj_z = pick_pos[2]       # 비전이 찾은 물체 높이
        final_target_z = current_obj_z + DEPTH_OFFSET # 오프셋 적용 후 높이
        
        # 조건 1: "니퍼"처럼 얇은 도구는 무조건 오프셋 0

        # 조건 2: 오프셋을 줬더니 바닥을 뚫고 들어갈 것 같다면? -> 오프셋 취소!
        if 0.0 <= final_target_z < FLOOR_LIMIT:
            self.get_logger().warn(f"[바닥 충돌 방지] 목표 Z({final_target_z:.1f})가 너무 낮음! -> 오프셋 제거")
            # 바닥을 박지 않도록, 물체 높이 그대로 잡거나 최소 안전 높이로 설정
            # pick_pos[2] = max(current_obj_z, FLOOR_LIMIT)
            pick_pos[2] = current_obj_z  
            
        else:
            # 안전하면 원래대로 오프셋 적용 (-10mm 더 내려감)
            pick_pos[2] = final_target_z
        # 실제 잡는 위치 (Depth Offset 적용)
        # pick_pos = list(target_pos)
        # pick_pos[2] += DEPTH_OFFSET 
        
        movel(pick_pos, vel=VELOCITY/2, acc=ACC)
        mwait()
        
        self.gripper.close_gripper()
        time.sleep(0.2) # 명령 들어갈 짬 주기
        while self.gripper.get_status()[0] == 1: # 다 닫힐 때까지 대기
             time.sleep(0.1)
        
        movel(approach_pos, vel=VELOCITY, acc=ACC) # 다시 상승

    def handover_action(self):
        self.get_logger().info("Starting Handover sequence...")
        
        # 1. 안전한 중간 위치로 이동 (사람을 바라보는 위치)
        # 예: 로봇 정면, 높이 적당히
        safe_pose = [-52.1, -452.1, 486, 50.73, -115.52, 97] 
        try:
            movel(safe_pose, vel=VELOCITY, acc=ACC)
        except:
            self.get_logger().warn("Move failed (Sim mode?)")

        # 2. 손 위치 찾기
        # 2. 손 위치 찾기 (무한 대기 루프)
        hand_base_pos = None
        
        self.get_logger().info(" Waiting for hand detection... (손을 보여주세요)")
        
        while rclpy.ok(): # 프로그램이 종료되지 않는 한 계속 반복
            hand_base_pos = self.get_vision_pos("hand")
            
            if hand_base_pos is not None:
                # 손을 찾았으면 루프 탈출!
                self.get_logger().info(f"✅ Hand found! Coordinates: {hand_base_pos}")
                break
            else:
                # 못 찾았으면 1초 대기 후 다시 시도
                self.get_logger().info("❌ Hand not found yet. Retrying in 1s...")
                time.sleep(1.0)
        
        if hand_base_pos:
            self.get_logger().info(f"Hand detected at {hand_base_pos}. Approaching...")
            
            # 3. 접근 (손 위로) - 안전 오프셋 필수!
            target_pos = list(hand_base_pos)
            target_pos[2] += HANDOVER_Z_OFFSET # 손보다 15cm 위
            
            movel(target_pos, vel=VELOCITY/2, acc=ACC)
            mwait()
            
            # 4. 전달 (그리퍼 오픈)
            self.get_logger().info("Here you go!")
            time.sleep(2.0) # 사람이 잡을 시간 주기
            self.gripper.open_gripper()
            
            # 5. 복귀 (위로 빠지기)
            target_pos[2] += 100.0
            movel(target_pos, vel=VELOCITY, acc=ACC)

            self.get_logger().info("🏠 초기 위치로 복귀 중...")
            JReady = [0, 0, 90, 0, 90, 0] # 초기 자세 값
            movej(JReady, vel=VELOCITY, acc=ACC)
        else:
            self.get_logger().warn("Cannot see hand. Staying in safe pose.")

if __name__ == "__main__":
    main_logic()
