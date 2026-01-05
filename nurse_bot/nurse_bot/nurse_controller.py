import os
import time
import sys
import threading
import json
import numpy as np
from scipy.spatial.transform import Rotation
from supabase import create_client, Client
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from nurse_bot.onrobot import RG

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from std_msgs.msg import String
from dsr_msgs2.srv import SetRobotControl, GetRobotState, DrlPause, DrlResume, DrlStop

import DR_init

class MockGripper:
    def open_gripper(self): print("[Gripper] Open")
    def close_gripper(self): print("[Gripper] Close")
    def get_status(self): return [0] 

# --- 설정 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 150, 100
DEPTH_OFFSET = -10.0 
HANDOVER_Z_OFFSET = 100.0 

GRIPPER_IP = "192.168.1.1" 
GRIPPER_PORT = 502
GRIPPER_TYPE = "rg2"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

TABLE_START_POS = [200.0, -180.0, 200.0, 0.0, 180.0, 0.0] 
CLEANUP_BIN_POS = [-15.5, 37.80, 57.0, -0.92, 81.8, -16.16]
ITEM_GAP = 70.0  

SURGERY_RECIPES = {
    "appendicitis": ["cutter", "scissors"],  
    "fracture": ["hammer","hammer", "driver","cutter","nipper"],       
    "suture": ["nipper", "scissors"]         
}
ALL_TOOLS = ["nipper", "scissors", "hammer", "driver", "cutter"]

def main_logic():
    rclpy.init()
    node = rclpy.create_node("nurse_controller", namespace=ROBOT_ID)
    
    DR_init.__dsr__node = node

    node = NurseController()
    
    
    # 버거 브릿지처럼 멀티스레드 실행기 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    print("✅ 로봇 준비 완료. (Burger-Bridge Style)")
    print("👉 '정지', '복구', '수술' 명령 대기 중...")

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

class NurseController(Node):
    def __init__(self):
        # super().__init__("nurse_controller", namespace="dsr01")
        super().__init__("nurse_controller")


        self.SUPABASE_URL = "https://oxbytxeozwwbctwrxhpk.supabase.co"
        self.SUPABASE_KEY = "sb_publishable_zzGU-pYBPq2AhGgEhd807g_gRDtz0to"   

        self.supabase = None
        try:
            self.supabase = create_client(self.SUPABASE_URL, self.SUPABASE_KEY)
            self.get_logger().info("✅ Supabase 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ Supabase 연결 실패: {e}")

        self.current_room_id = "room1"
        self.current_surgery_name = None
        
        # [핵심] ReentrantCallbackGroup: 병렬 실행 허용
        self.callback_group = ReentrantCallbackGroup()
        
        self.cli_vision = self.create_client(SrvDepthPosition, "/get_3d_position", callback_group=self.callback_group)
        
        # 웹/음성 명령 구독
        self.create_subscription(String, '/web_command', self.web_cmd_callback, 10, callback_group=self.callback_group)
        
        # DB 감시 타이머
        self.create_timer(3.0, self.monitor_db_cleanup, callback_group=self.callback_group)

        self.vision_pub = self.create_publisher(String, '/check', 10)
        self.status_pub = self.create_publisher(String, '/robot_status_update', 10)

        # [추가] 제어 서비스 클라이언트 (버거 브릿지 방식)
        self.cli_pause = self.create_client(DrlPause, '/dsr01/drl/drl_pause', callback_group=self.callback_group)
        self.cli_resume = self.create_client(DrlResume, '/dsr01/drl/drl_resume', callback_group=self.callback_group)
        self.cli_stop = self.create_client(DrlStop, '/dsr01/drl/drl_stop', callback_group=self.callback_group)
        self.cli_set_control = self.create_client(SetRobotControl, '/dsr01/system/set_robot_control', callback_group=self.callback_group)
        self.cli_get_state = self.create_client(GetRobotState, '/dsr01/system/get_robot_state', callback_group=self.callback_group)
        
        # 상태 모니터링 (1초 주기)
        self.create_timer(1.0, self.monitor_robot_state, callback_group=self.callback_group)
        self.create_subscription(String, '/dsr01/error', self.error_callback, 10, callback_group=self.callback_group)

        self.stop_event = threading.Event()
        self.mission_cancelled = False
        self.is_busy = False
        self.is_paused = False

        self.pkg_path = get_package_share_directory("nurse_bot")
        self.calib_path = os.path.join(self.pkg_path, "resource", "T_gripper2camera.npy")
        
        self.gripper = None
        if RG:
            try:
                self.gripper = RG(GRIPPER_TYPE, GRIPPER_IP, GRIPPER_PORT)
                self.get_logger().info("✅ OnRobot 그리퍼 연결 성공!")
            except: pass
        if self.gripper is None: self.gripper = MockGripper()

        # [핵심] 로봇 API 초기화 및 Wrapper 적용
        self.init_robot_api()

        if self.gripper:
            self.gripper.open_gripper = self.create_safe_wrapper(self.gripper.open_gripper)
            self.gripper.close_gripper = self.create_safe_wrapper(self.gripper.close_gripper)

    def init_robot_api(self):
        """두산 로봇 API 함수들에 안전장치(Wrapper)를 씌움"""
        try:
            from DSR_ROBOT2 import (
                movej, movel, movejx, mwait, 
                set_velj, set_accj, set_velx, set_accx,
                get_current_posx, get_last_alarm,
                drl_script_stop, drl_script_pause, drl_script_resume,
                DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE, DR_BASE
            )
            
            # 원본 함수 저장
            self.raw_movej = movej
            self.raw_movel = movel
            
            # Wrapper 적용 (이게 핵심입니다!)
            self.movej = self.create_safe_wrapper(movej)
            self.movel = self.create_safe_wrapper(movel)
            self.movejx = self.create_safe_wrapper(movejx)
            self.mwait = self.create_safe_wrapper(mwait)
            
            # 기타 함수 매핑
            self.drl_script_stop = drl_script_stop
            self.drl_script_pause = drl_script_pause
            self.drl_script_resume = drl_script_resume
            self.get_current_posx = get_current_posx
            self.get_last_alarm = get_last_alarm
            
            # 상수
            self.DR_MV_MOD_ABS = DR_MV_MOD_ABS
            self.DR_MV_MOD_REL = DR_MV_MOD_REL
            self.DR_MV_RA_DUPLICATE = DR_MV_RA_DUPLICATE
            self.DR_BASE = DR_BASE
            
        except ImportError:
            self.get_logger().error("DSR_ROBOT2 Import Error")

    def create_safe_wrapper(self, func):
        """로봇 동작 전 정지 상태를 체크하는 래퍼 함수"""
        def wrapper(*args, **kwargs):
            # 1. 멈춰야 하는지 확인 (Paused 상태면 여기서 무한 대기)
            if self.check_and_wait():
                return
            # 2. 문제 없으면 실행
            return func(*args, **kwargs)
        return wrapper

    def check_and_wait(self):
        """일시정지 중이면 대기, 취소면 True 반환"""
        if self.is_paused:
            self.get_logger().info("⏳ 일시정지! 복구 명령 대기 중...")
            self.stop_event.wait() # Resume 될 때까지 대기
            self.stop_event.clear()
            self.get_logger().info("🚦 대기 해제! 동작 재개")
            time.sleep(1.0)

        if self.mission_cancelled:
            self.get_logger().info("🛑 작업 취소됨.")
            return True
        return False

    # ==========================================================
    # 📡 웹 명령 처리 (멀티스레드 활용)
    # ==========================================================
    def web_cmd_callback(self, msg):
        """ReentrantCallbackGroup 덕분에 작업 중에도 호출됨"""
        cmd = msg.data
        self.get_logger().info(f"📱 명령 수신: {cmd}")

        # 1. 비상 정지 (즉시 처리)
        if any(x in cmd for x in ["stop", "정지", "멈춰"]):
            self.handle_stop_command()
            return

        # 2. 복구 명령 (즉시 처리)
        if any(x in cmd for x in ["recover", "home", "복구", "원위치"]):
            self.handle_recovery_command()
            return

        # 3. 일반 작업 (작업 중이면 무시)
        if self.is_busy:
            self.get_logger().warn("⛔ 로봇 작업 중! (명령 무시)")
            return
        self.is_busy = True
        try:
            self._process_task(cmd)
        finally:
            self.is_busy = False
            self.get_logger().info("✅ 작업 완료 (대기 중)")


    def handle_stop_command(self):
        self.get_logger().warn("🚨 [비상 정지] 로봇 일시 정지!")
        if self.cli_pause.service_is_ready():
            self.cli_pause.call_async(DrlPause.Request())
        self.is_paused = True
        self.update_status("paused")

    def handle_recovery_command(self):
        self.get_logger().info("🔄 [복구] 명령 수신")
        if self.is_paused:
            # 일시정지 해제 (Resume)
            if self.cli_resume.service_is_ready():
                self.cli_resume.call_async(DrlResume.Request())
            self.is_paused = False
            self.stop_event.set() # 대기 중인 스레드 깨움
        else:
            # 홈 복귀 (완전 초기화)
            self.mission_cancelled = True
            self.stop_event.set()
            # 복구 시퀀스는 별도 실행
            self.execute_home_sequence()

    def _process_task(self, cmd):
        """실제 작업 로직"""
        try:
            if "surgery:" in cmd:
                parts = {}
                for section in cmd.split('/'):
                    if ':' in section:
                        key, val = section.split(':')
                        parts[key.strip()] = val.strip()
                surgery_name = parts.get("surgery", "").strip()
                if not surgery_name and ":" in cmd: surgery_name = cmd.split(":")[1].strip()
                
                room_id = parts.get("room", "").strip()
                if not room_id: room_id = self.current_room_id if self.current_room_id else "room1"
                self.current_room_id = room_id

                self.perform_surgery_prep(surgery_name, self.current_room_id)
            
            elif "cmd:end" in cmd:
                msg_data = f"{self.current_room_id}:True"
                self.vision_pub.publish(String(data=msg_data))
                if self.supabase:
                    self.supabase.table("rooms").update({"availability": "Cleanup", "updated_at": datetime.now().isoformat()}).eq("room", self.current_room_id).execute()
            
            elif "tool:" in cmd:
                tool_name = cmd.split(":")[1].strip()
                self.single_tool_delivery(tool_name)

            elif "open" in cmd: self.gripper.open_gripper()
            elif "close" in cmd: self.gripper.close_gripper()

        except Exception as e:
            self.get_logger().error(f"작업 에러: {e}")

    # ==========================================================
    # 스마트 복구 & 상태 감시
    # ==========================================================
    def execute_home_sequence(self):
        self.get_logger().info("🛠️ 복구 시퀀스 시작...")
        self.update_status('recovering')
        
        # 상태 확인 및 리셋
        current_state = 0
        if self.cli_get_state.service_is_ready():
            future = self.cli_get_state.call_async(GetRobotState.Request())
            while not future.done(): time.sleep(0.01)
            try: current_state = future.result().robot_state
            except: pass
            
        if current_state in [3, 5, 6]:
            reset_cmd = 4
            if current_state == 5: reset_cmd = 2
            elif current_state == 3: reset_cmd = 3
            
            if self.cli_set_control.service_is_ready():
                req = SetRobotControl.Request()
                req.robot_control = reset_cmd
                self.cli_set_control.call_async(req)
                time.sleep(1.0)
                req.robot_control = 2 
                self.cli_set_control.call_async(req)
                time.sleep(2.0)

        # 홈 이동 (안전 래퍼 사용하지 않고 원본 함수 사용 권장 - 복구 중이니까)
        try:
            self.get_logger().info("🏠 홈 위치로 복귀...")
            JReady = [0, 0, 90, 0, 90, 0]
            if hasattr(self, 'raw_movej'):
                self.raw_movej(JReady, vel=50.0, acc=50.0)
        except: pass

        self.mission_cancelled = False
        self.is_busy = False
        self.stop_event.clear()
        self.update_status("ready")
        self.get_logger().info("✨ 복구 완료.")

    def error_callback(self, msg):
        self.get_logger().warn(f"🚨 에러 신호 감지: {msg.data}")
        self.mission_cancelled = True 
        self.stop_event.set()
        threading.Thread(target=self.handle_collision_recovery).start()

    def monitor_robot_state(self):
        if not self.cli_get_state.service_is_ready():
            self.get_logger().warn("⏳ 로봇 상태 서비스(/get_robot_state) 연결 대기 중...")
            return
        req = GetRobotState.Request()
        future = self.cli_get_state.call_async(req)
        future.add_done_callback(self._on_state_receive)

    def _on_state_receive(self, future):
        try:
            result = future.result()
            current_state = result.robot_state
            
            # ★★★ [추가] 1초마다 터미널에 현재 상태 출력하기 ★★★
            # 1: 대기(Normal), 2: 이동중, 3: 비상정지, 5: 안전정지
            state_msg = "알 수 없음"
            if current_state == 1: state_msg = "🟢 정상 대기 (Standby)"
            elif current_state == 2: state_msg = "🔵 이동 중 (Moving)"
            elif current_state == 3: state_msg = "🔴 비상 정지 (Emergency Stop)"
            elif current_state == 4: state_msg = "🟡 안전 정지 (Safe Stop)"
            elif current_state == 5: state_msg = "🟠 가르치기 모드 (Teaching)"
            elif current_state == 6: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 7: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 8: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 9: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 10: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 11: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 12: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 13: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 14: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 15: state_msg = "🔴 에러/충돌 (Error)"
            elif current_state == 16: state_msg = "🔴 에러/충돌 (Error)"
            # 너무 도배되는 게 싫으면 이 줄을 주석 처리하세요.
            self.get_logger().info(f"🤖 [상태 모니터] 현재 상태: {current_state} - {state_msg}")
            
            if current_state == 1 and self.mission_cancelled:
                self.get_logger().info("✨ 로봇 정상화 감지! 취소 모드를 해제하고 작업을 허용합니다.")
                self.mission_cancelled = False
                self.stop_event.clear()
                self.is_paused = False
            
            if result.robot_state in [3, 5, 6, 8]:
                if self.mission_cancelled: return
                self.get_logger().error(f"🚨 [비상 감지] 상태 이상! (State: {result.robot_state})")
                self.mission_cancelled = True 
                self.stop_event.set()
                threading.Thread(target=self.handle_collision_recovery).start()
        except Exception as e:
            # ★★★ [중요] 에러가 나면 뭔지 알려줘야 함! ★★★
            self.get_logger().error(f"❌ 상태 조회 콜백 에러: {e}")

    def handle_collision_recovery(self):
        self.get_logger().error("🚨 충돌/정지 발생! '복구' 명령을 내려주세요.")
        self.update_status("error_collision")

    def update_status(self, status):
        msg = String()
        msg.data = json.dumps({'status': status, 'timestamp': time.time()})
        self.status_pub.publish(msg)

    # (이하 기존 기능들: DB, Cleanup, Vision - movej 등은 이제 self.movej로 호출됨)
    def init_surgery_db(self, room_id, surgery_name):
        if not self.supabase: return
        try:
            current_time = datetime.now().isoformat()
            room_payload = {"room": room_id, "surgery_type": surgery_name, "availability": "in_use", "updated_at": current_time}
            self.supabase.table("rooms").upsert(room_payload, on_conflict="room").execute()
            tool_payload = {"room": room_id, "updated_at": current_time}
            for t in ALL_TOOLS: tool_payload[t] = 0
            self.supabase.table("tool_count").upsert(tool_payload, on_conflict="room").execute()
            self.get_logger().info(f"💾 DB[{room_id}] 수술 시작: {surgery_name}")
        except Exception as e: self.get_logger().error(f"❌ DB 초기화 실패: {e}")
    
    def monitor_db_cleanup(self):
        if self.is_busy or not self.supabase: return
        try:
            room_res = self.supabase.table("rooms").select("availability").eq("room", self.current_room_id).execute()
            if not room_res.data: return
            if room_res.data[0]['availability'] == "Cleanup":
                check_res = self.supabase.table("check_count").select("check").eq("room", self.current_room_id).order("created_at", desc=True).limit(1).execute()
                if check_res.data and check_res.data[0]['check'] is True:
                    self.get_logger().info(f"🧹 [자동 감지] 정리 시작!")
                    self.perform_cleanup_sequence()
        except: pass

    def perform_cleanup_sequence(self):
        self.is_busy = True
        self.get_logger().info("🧹 --- 정리(Cleanup) 모드 시작 ---")
        try:
            # 주의: 이제 self.movej를 사용합니다 (wrapper 적용됨)
            JReady = [0, 0, 90, 0, 90, 0] 
            self.movej(JReady, vel=VELOCITY, acc=ACC) 
            time.sleep(2.0)

            scan_pose = [-90, 31.55, 36.7, 0.0, 110.3, 0.0]
            tool_clean = [-46.577, 3.021, 85.977, -0.002, 91.002, -46.942]
            
            self.movej(scan_pose, vel=VELOCITY, acc=ACC)
            time.sleep(3.0) 
            
            for tool_name in ALL_TOOLS:
                while True:
                    self.get_logger().info(f"🔍 '{tool_name}' 스캔 중...")
                    time.sleep(0.5)
                    tool_pos = self.get_vision_pos(tool_name)
                    if tool_pos:
                        self.get_logger().info(f"🗑️ '{tool_name}' 발견! 수거합니다.")
                        self.pick_action(tool_pos, tool_name=tool_name) 
                        self.movej(tool_clean, vel=VELOCITY, acc=ACC)
                        self.movej(CLEANUP_BIN_POS, vel=VELOCITY, acc=ACC)
                        self.gripper.open_gripper()
                        time.sleep(1.0) 
                        self.movej(tool_clean, vel=VELOCITY, acc=ACC)
                        self.movej(scan_pose, vel=VELOCITY, acc=ACC)
                        time.sleep(3.0)
                    else:
                        break
            
            self.get_logger().info("✨ 모든 정리 완료! DB 상태 변경.")
            try:
                self.supabase.table("rooms").update({"availability": "available", "surgery_type": None, "updated_at": datetime.now().isoformat()}).eq("room", self.current_room_id).execute()
            except: pass
            self.movej(JReady, vel=VELOCITY, acc=ACC)
        except: pass
        self.is_busy = False

    def increment_tool_usage(self, tool_name):
        if not self.supabase: return
        try:
            response = self.supabase.table("tool_count").select("*").eq("room", self.current_room_id).execute()
            row = response.data[0] if response.data else {t: 0 for t in ALL_TOOLS}
            payload = {"room": self.current_room_id, "updated_at": datetime.now().isoformat()}
            for t in ALL_TOOLS: payload[t] = int(row.get(t, 0))
            payload[tool_name] = int(row.get(tool_name, 0)) + 1
            self.supabase.table("tool_count").upsert(payload, on_conflict="room").execute()
        except: pass
 
    def perform_surgery_prep(self, surgery_name, room_id):
        if surgery_name not in SURGERY_RECIPES: return
        tool_list = SURGERY_RECIPES[surgery_name]
        self.init_surgery_db(room_id, surgery_name)
        
        search_poses = [
            [15.03, -4.65, 98.04, -0.05, 86.6, 14.67], 
            [9.38, 23.17, 65.27, 0.13, 91.55, 9.05],
        ]
        
        for i, tool in enumerate(tool_list):
            found_pos = None
            for pose in search_poses:
                self.movej(pose, vel=VELOCITY, acc=ACC)
                self.mwait()
                time.sleep(0.5)
                found_pos = self.get_vision_pos(tool)
                if found_pos: break 
            
            if found_pos:
                self.pick_action(found_pos, tool_name=tool)
                self.smart_place_action(i)

    def smart_place_action(self, index):
        JReady = [0, 0, 90, 0, 90, 0] 
        self.movej(JReady, vel=VELOCITY, acc=ACC)
        self.mwait()
        target_pos = list(TABLE_START_POS)
        target_pos[0] += (ITEM_GAP * index) 
        self.movel(target_pos, vel=VELOCITY, acc=ACC) 
        self.mwait()
        
        req = SrvDepthPosition.Request()
        req.target = "measure"
        floor_dist = 400.0
        
        future = self.cli_vision.call_async(req)
        while not future.done(): time.sleep(0.01)
        try:
            res = future.result()
            if res and res.depth_position[2] > 0:
                floor_dist = res.depth_position[2]
        except: pass

        cam_point = [0.0, 0.0, floor_dist] 
        cur_pos_data = self.get_current_posx()
        if not cur_pos_data: return
        robot_pose = cur_pos_data[0]
        floor_point_base = self.transform_to_base(cam_point, self.calib_path, robot_pose)
        
        floor_abs_z = floor_point_base[2]
        if floor_abs_z <= 0: target_z = 25.0
        else: target_z = floor_abs_z + 20.0
        
        dest_pos = list(robot_pose)
        dest_pos[2] = target_z
        self.movel(dest_pos, vel=VELOCITY/2, acc=ACC)
        self.mwait()
        self.gripper.open_gripper()
        time.sleep(1.0)
        self.movel(target_pos, vel=VELOCITY, acc=ACC)
        self.mwait()

    def single_tool_delivery(self, tool_name):
        search_poses = [[270, -280, 200, 0, 180, 0], [320, -280, 200, 0, 180, 0]]
        found_pos = None
        for i, pose in enumerate(search_poses):
            self.mwait()
            self.movel(pose, vel=VELOCITY, acc=ACC)
            self.mwait()
            time.sleep(0.5) 
            found_pos = self.get_vision_pos(tool_name)
            if found_pos: break 

        if found_pos:
            self.pick_action(found_pos, tool_name=tool_name)
            self.handover_action()
            self.increment_tool_usage(tool_name)

    def get_vision_pos(self, target_name):
        candidates = [] 
        for i in range(3):
            req = SrvDepthPosition.Request()
            req.target = target_name
            future = self.cli_vision.call_async(req)
            while not future.done(): time.sleep(0.01)
            res = future.result()
            if res and sum(res.depth_position) != 0:
                cam_coords = res.depth_position
                cur_pos_data = self.get_current_posx()
                if cur_pos_data:
                    robot_pos = cur_pos_data[0] 
                    base_coords = self.transform_to_base(cam_coords, self.calib_path, robot_pos)
                    candidates.append(list(base_coords) + list(robot_pos[3:]))
            time.sleep(0.1)
        if not candidates: return None
        return max(candidates, key=lambda p: p[2])

    def transform_to_base(self, cam_xyz, calib_path, robot_pose):
        try: gripper2cam = np.load(calib_path)
        except: gripper2cam = np.eye(4)
        cam_p = np.append(np.array(cam_xyz), 1)
        x, y, z, rx, ry, rz = robot_pose
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T_base2grip = np.eye(4)
        T_base2grip[:3, :3] = R
        T_base2grip[:3, 3] = [x, y, z]
        return (T_base2grip @ gripper2cam @ cam_p)[:3]

    def pick_action(self, target_pos, tool_name=""):
        self.gripper.open_gripper()
        time.sleep(0.2) 
        while self.gripper.get_status()[0] == 1: time.sleep(0.1)
        approach_pos = list(target_pos)
        approach_pos[2] += 110.0 
        self.movel(approach_pos, vel=VELOCITY, acc=ACC)

        pick_pos = list(target_pos)
        FLOOR_LIMIT = 10.0  
        current_obj_z = pick_pos[2]       
        final_target_z = current_obj_z + DEPTH_OFFSET 
        if 0.0 <= final_target_z < FLOOR_LIMIT: pick_pos[2] = current_obj_z  
        else: pick_pos[2] = final_target_z
        
        self.movel(pick_pos, vel=VELOCITY/2, acc=ACC)
        self.mwait()
        self.gripper.close_gripper()
        time.sleep(0.2) 
        while self.gripper.get_status()[0] == 1: time.sleep(0.1)
        self.movel(approach_pos, vel=VELOCITY, acc=ACC) 

    def handover_action(self):
        safe_pose = [-52.1, -452.1, 486, 50.73, -115.52, 97] 
        try: self.movel(safe_pose, vel=VELOCITY, acc=ACC)
        except: pass

        self.get_logger().info(" Waiting for hand detection...")
        while rclpy.ok(): 
            hand_base_pos = self.get_vision_pos("hand")
            if hand_base_pos: break
            time.sleep(1.0)
        
        if hand_base_pos:
            target_pos = list(hand_base_pos)
            target_pos[2] += HANDOVER_Z_OFFSET 
            self.movel(target_pos, vel=VELOCITY/2, acc=ACC)
            self.mwait()
            time.sleep(2.0) 
            self.gripper.open_gripper()
            target_pos[2] += 100.0
            self.movel(target_pos, vel=VELOCITY, acc=ACC)
            JReady = [0, 0, 90, 0, 90, 0]
            self.movej(JReady, vel=VELOCITY, acc=ACC)

if __name__ == "__main__":
    main_logic()