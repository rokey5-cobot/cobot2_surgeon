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
from dsr_msgs2.srv import SetRobotControl, GetRobotState

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
    node = NurseController()
    DR_init.__dsr__node = node
    
    try:
        global movej, movel, get_current_posx, mwait , movejx
        from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, movejx
    except ImportError:
        print("DSR_ROBOT2 Import Error!")
        return

    # 실행기 생성 (콜백 처리를 위해)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    print("✅ 로봇 준비 완료. Web/Voice 서버 명령(/web_command) 대기 중...")

    try:
        # ★ run_scenario 없이, spin_once로 계속 명령을 체크합니다.
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

class NurseController(Node):
    def __init__(self):
        super().__init__("nurse_controller", namespace="dsr01")

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
        
        self.callback_group = ReentrantCallbackGroup()
        self.cli_vision = self.create_client(SrvDepthPosition, "/get_3d_position", callback_group=self.callback_group)
        
        # ★ [수정] 직접 음성인식 클라이언트(cli_voice) 제거됨
        
        # 웹 명령 구독 (여기서 정지/복구/작업 다 처리)
        self.create_subscription(String, '/web_command', self.web_cmd_callback, 10, callback_group=self.callback_group)
        self.create_timer(3.0, self.monitor_db_cleanup, callback_group=self.callback_group)

        self.vision_pub = self.create_publisher(String, '/check', 10)
        self.status_pub = self.create_publisher(String, '/robot_status_update', 10)

        # 복구/취소 관련 설정
        self.cli_set_control = self.create_client(SetRobotControl, '/dsr01/system/set_robot_control', callback_group=self.callback_group)
        self.cli_get_state = self.create_client(GetRobotState, '/dsr01/system/get_robot_state', callback_group=self.callback_group)
        
        self.create_timer(1.0, self.monitor_robot_state, callback_group=self.callback_group)
        self.create_subscription(String, '/dsr01/error', self.error_callback, 10, callback_group=self.callback_group)

        self.stop_event = threading.Event()
        self.mission_cancelled = False
        self.is_busy = False

        self.pkg_path = get_package_share_directory("nurse_bot")
        self.calib_path = os.path.join(self.pkg_path, "resource", "T_gripper2camera.npy")
        
        self.gripper = None
        if RG:
            try:
                self.gripper = RG(GRIPPER_TYPE, GRIPPER_IP, GRIPPER_PORT)
                self.get_logger().info("✅ OnRobot 그리퍼 연결 성공!")
            except: pass
        if self.gripper is None: self.gripper = MockGripper()

    # ==========================================================
    # 📡 웹/음성(서버경유) 명령 통합 처리
    # ==========================================================
    def web_cmd_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"📱 명령 수신: {cmd}")

        # 1. [비상 정지] 명령 확인
        if any(x in cmd for x in ["stop", "정지", "멈춰"]):
            self.get_logger().warn("🚨 [비상 정지] 명령 수신!")
            self.mission_cancelled = True
            self.stop_event.set()
            threading.Thread(target=self.handle_collision_recovery).start()
            return

        # 2. [복구/홈] 명령 확인
        if any(x in cmd for x in ["recover", "home", "복구", "원위치"]):
            self.get_logger().info("🔄 [복구] 명령 수신!")
            threading.Thread(target=self.execute_home_sequence).start()
            return

        # 3. 일반 명령 (로봇이 바쁘면 무시)
        if self.is_busy:
            self.get_logger().warn("⛔ 로봇 작업 중! 일반 명령 무시.")
            return

        # 작업 스레드 시작
        threading.Thread(target=self._process_web_cmd_task, args=(cmd,)).start()

    def _process_web_cmd_task(self, cmd):
        self.is_busy = True
        try:
            # 1. 수술 준비 (surgery:...)
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
            
            # 2. 수술 종료 (cmd:end)
            elif "cmd:end" in cmd:
                msg_data = f"{self.current_room_id}:True"
                self.vision_pub.publish(String(data=msg_data))
                if self.supabase:
                    self.supabase.table("rooms").update({"availability": "Cleanup", "updated_at": datetime.now().isoformat()}).eq("room", self.current_room_id).execute()
            
            # 3. 도구 전달 (tool:...)
            elif "tool:" in cmd:
                tool_name = cmd.split(":")[1].strip()
                self.single_tool_delivery(tool_name)

            # 4. 테스트용 (open/close)
            elif "open" in cmd: self.gripper.open_gripper()
            elif "close" in cmd: self.gripper.close_gripper()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            self.is_busy = False
            self.get_logger().info("✅ 작업 완료. 대기 상태 (Ready)")

    # ==========================================================
    # 상태 감시 및 복구
    # ==========================================================
    def error_callback(self, msg):
        self.get_logger().warn(f"🚨 에러 신호 감지: {msg.data}")
        self.mission_cancelled = True 
        self.stop_event.set()
        threading.Thread(target=self.handle_collision_recovery).start()

    def monitor_robot_state(self):
        if not self.cli_get_state.service_is_ready(): return
        req = GetRobotState.Request()
        future = self.cli_get_state.call_async(req)
        future.add_done_callback(self._on_state_receive)

    def _on_state_receive(self, future):
        try:
            result = future.result()
            # 3:비상정지, 5:안전정지, 6:충돌, 8:알람
            if result.robot_state in [3, 5, 6, 8]:
                if self.mission_cancelled: return
                self.get_logger().error(f"🚨 [비상 감지] 상태 이상! (State: {result.robot_state})")
                self.mission_cancelled = True 
                self.stop_event.set()
                threading.Thread(target=self.handle_collision_recovery).start()
        except: pass

    def handle_collision_recovery(self):
        self.get_logger().error("🚨 충돌/정지 발생! '복구' 명령을 내려주세요.")
        self.update_status("error_collision")

    def execute_home_sequence(self):
        self.get_logger().info("🛠️ 복구 시퀀스 시작...")
        self.update_status("recovering")
        self.mission_cancelled = True 
        # 1. 현재 로봇 상태(State) 조회
        current_state = 0
        if self.cli_get_state.service_is_ready():
            req = GetRobotState.Request()
            future = self.cli_get_state.call_async(req)
            while not future.done(): time.sleep(0.01)
            try:
                current_state = future.result().robot_state
            except: pass
            
        # 에러 상태인지 체크 (3:빨강, 5:노랑, 6:비상버튼)
        is_error = current_state in [3, 5, 6]

        if is_error:
            self.get_logger().warn(f"🛠️ 비상 상태(State {current_state}) 감지! 맞춤형 리셋을 시작합니다.")
            success = False
            
            # 최대 3번 시도
            for attempt in range(1, 4):
                self.get_logger().info(f"🔄 복구 시도 {attempt}/3 ...")
                
                # 상태별 리셋 명령 선택 (중요!)
                reset_cmd = 4 # 기본값 (Safety Reset)
                
                if current_state == 5:
                    self.get_logger().info("🟡 상태: 노란불(Safe Stop) -> 명령: 2번 (Reset Safe Stop)")
                    reset_cmd = 2 
                elif current_state == 3:
                    self.get_logger().info("🔴 상태: 빨간불(Safe Off) -> 명령: 3번 (Reset Safe Off)")
                    reset_cmd = 3
                else:
                    self.get_logger().info("⚪ 기타 상태 -> 명령: 4번 (Safety Reset)")
                    reset_cmd = 4 

                # (1) 리셋 명령 전송
                if self.cli_set_control.service_is_ready():
                    req = SetRobotControl.Request()
                    req.robot_control = reset_cmd
                    future = self.cli_set_control.call_async(req)
                    while not future.done(): time.sleep(0.1)
                
                time.sleep(1.0) # 리셋 적용 대기

                # (2) 서보 켜기 (Servo On)
                if self.cli_set_control.service_is_ready():
                    self.get_logger().info("🔌 시동 켜기 (Servo On)...")
                    req = SetRobotControl.Request()
                    req.robot_control = 2 
                    future = self.cli_set_control.call_async(req)
                    while not future.done(): time.sleep(0.1)

                time.sleep(1.0)
                if self.cli_get_state.service_is_ready():
                    req = GetRobotState.Request()
                    future = self.cli_get_state.call_async(req)
                    while not future.done(): time.sleep(0.01)
                    try:
                        new_state = future.result().robot_state
                        if new_state == 1: # 1: Standby (정상)
                            success = True
                            self.get_logger().info("✅ 로봇 정상화 성공!")
                            break
                    except: pass
            
            if not success:
                 self.get_logger().error("❌ 복구 실패. 수동 조치가 필요합니다.")
                 self.update_status("error_collision")
                 return # 복구 실패 시 홈 이동 안 함

        # 2. 홈 복귀
        try:
            self.get_logger().info("🏠 홈 위치로 복귀...")
            from DSR_ROBOT2 import movej
            JReady = [0, 0, 90, 0, 90, 0]
            movej(JReady, vel=50.0, acc=50.0)
        except: pass

        self.mission_cancelled = False
        self.is_busy = False
        self.stop_event.clear()
        self.update_status("ready")
        self.get_logger().info("✨ 복구 완료.")

    def update_status(self, status):
        msg = String()
        msg.data = json.dumps({'status': status, 'timestamp': time.time()})
        self.status_pub.publish(msg)

    def check_cancellation(self):
        if self.mission_cancelled:
            self.get_logger().warn("🛑 작업 취소됨.")
            return True
        return False

    # (이하 기존 DB, Cleanup, Vision 로직 유지)
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
        if self.check_cancellation(): return 
        self.is_busy = True
        self.get_logger().info("🧹 --- 정리(Cleanup) 모드 시작 ---")
        
        try:
            from DSR_ROBOT2 import movej, movel, mwait
            JReady = [0, 0, 90, 0, 90, 0] 
            movej(JReady, vel=VELOCITY, acc=ACC) 
            time.sleep(2.0)

            scan_pose = [-90, 31.55, 36.7, 0.0, 110.3, 0.0]
            tool_clean = [-46.577, 3.021, 85.977, -0.002, 91.002, -46.942]
            
            movej(scan_pose, vel=VELOCITY, acc=ACC)
            time.sleep(3.0) 
            
            for tool_name in ALL_TOOLS:
                while True:
                    if self.check_cancellation(): return 

                    self.get_logger().info(f"🔍 '{tool_name}' 스캔 중...")
                    time.sleep(0.5)
                    tool_pos = self.get_vision_pos(tool_name)
                    
                    if tool_pos:
                        self.get_logger().info(f"🗑️ '{tool_name}' 발견! 수거합니다.")
                        self.pick_action(tool_pos, tool_name=tool_name) 
                        
                        if self.check_cancellation(): return 

                        movej(tool_clean, vel=VELOCITY, acc=ACC)
                        self.gripper.open_gripper()
                        time.sleep(1.0) 
                        
                        movej(scan_pose, vel=VELOCITY, acc=ACC)
                        time.sleep(3.0)
                    else:
                        break
            
            self.get_logger().info("✨ 모든 정리 완료! DB 상태 변경.")
            try:
                self.supabase.table("rooms").update({"availability": "available", "surgery_type": None, "updated_at": datetime.now().isoformat()}).eq("room", self.current_room_id).execute()
            except: pass
            movej(JReady, vel=VELOCITY, acc=ACC)
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
        if self.check_cancellation(): return 

        if surgery_name not in SURGERY_RECIPES: return
        tool_list = SURGERY_RECIPES[surgery_name]
        self.init_surgery_db(room_id, surgery_name)
        
        search_poses = [
            [15.03, -4.65, 98.04, -0.05, 86.6, 14.67], 
            [9.38, 23.17, 65.27, 0.13, 91.55, 9.05],
        ]
        
        from DSR_ROBOT2 import movej, mwait
        
        for i, tool in enumerate(tool_list):
            if self.check_cancellation(): return 

            found_pos = None
            for pose in search_poses:
                mwait()
                movej(pose, vel=VELOCITY, acc=ACC)
                mwait()
                time.sleep(0.5)
                found_pos = self.get_vision_pos(tool)
                if found_pos: break 
            
            if found_pos:
                self.pick_action(found_pos, tool_name=tool)
                self.smart_place_action(i)

    def smart_place_action(self, index):
        if self.check_cancellation(): return 
        from DSR_ROBOT2 import movej, movel, mwait
        
        JReady = [0, 0, 90, 0, 90, 0] 
        movej(JReady, vel=VELOCITY, acc=ACC)
        mwait()
        target_pos = list(TABLE_START_POS)
        target_pos[0] += (ITEM_GAP * index) 
        movel(target_pos, vel=VELOCITY, acc=ACC) 
        mwait()
        
        req = SrvDepthPosition.Request()
        req.target = "measure"
        floor_dist = 400.0
        
        future = self.cli_vision.call_async(req)
        while not future.done():
            if self.check_cancellation(): return 
            time.sleep(0.01)
        
        try:
            res = future.result()
            if res and res.depth_position[2] > 0:
                floor_dist = res.depth_position[2]
        except: pass

        cam_point = [0.0, 0.0, floor_dist] 
        cur_pos_data = get_current_posx()
        if not cur_pos_data: return
        robot_pose = cur_pos_data[0]
        floor_point_base = self.transform_to_base(cam_point, self.calib_path, robot_pose)
        
        floor_abs_z = floor_point_base[2]
        if floor_abs_z <= 0: target_z = 25.0
        else: target_z = floor_abs_z + 20.0
        
        dest_pos = list(robot_pose)
        dest_pos[2] = target_z
        movel(dest_pos, vel=VELOCITY/2, acc=ACC)
        mwait()
        self.gripper.open_gripper()
        time.sleep(1.0)
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()

    def single_tool_delivery(self, tool_name):
        search_poses = [
            [270, -280, 200, 0, 180, 0],   
            [320, -280, 200, 0, 180, 0],    
        ]
        
        from DSR_ROBOT2 import movel, mwait
        
        found_pos = None
        for i, pose in enumerate(search_poses):
            if self.check_cancellation(): return 
            mwait()
            movel(pose, vel=VELOCITY, acc=ACC)
            mwait()
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
            if self.check_cancellation(): return None 
            req = SrvDepthPosition.Request()
            req.target = target_name
            future = self.cli_vision.call_async(req)
            while not future.done(): time.sleep(0.01)
            res = future.result()
            if res and sum(res.depth_position) != 0:
                cam_coords = res.depth_position
                cur_pos_data = get_current_posx()
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
        from DSR_ROBOT2 import movel, mwait
        self.gripper.open_gripper()
        time.sleep(0.2) 
        while self.gripper.get_status()[0] == 1: time.sleep(0.1)
        approach_pos = list(target_pos)
        approach_pos[2] += 110.0 
        movel(approach_pos, vel=VELOCITY, acc=ACC)

        pick_pos = list(target_pos)
        FLOOR_LIMIT = 10.0  
        current_obj_z = pick_pos[2]       
        final_target_z = current_obj_z + DEPTH_OFFSET 
        
        if 0.0 <= final_target_z < FLOOR_LIMIT:
            pick_pos[2] = current_obj_z  
        else:
            pick_pos[2] = final_target_z
        
        movel(pick_pos, vel=VELOCITY/2, acc=ACC)
        mwait()
        self.gripper.close_gripper()
        time.sleep(0.2) 
        while self.gripper.get_status()[0] == 1: time.sleep(0.1)
        movel(approach_pos, vel=VELOCITY, acc=ACC) 

    def handover_action(self):
        from DSR_ROBOT2 import movel, movej, mwait
        safe_pose = [-52.1, -452.1, 486, 50.73, -115.52, 97] 
        try: movel(safe_pose, vel=VELOCITY, acc=ACC)
        except: pass

        self.get_logger().info(" Waiting for hand detection...")
        while rclpy.ok(): 
            if self.check_cancellation(): return 
            hand_base_pos = self.get_vision_pos("hand")
            if hand_base_pos: break
            time.sleep(1.0)
        
        if hand_base_pos:
            target_pos = list(hand_base_pos)
            target_pos[2] += HANDOVER_Z_OFFSET 
            movel(target_pos, vel=VELOCITY/2, acc=ACC)
            mwait()
            time.sleep(2.0) 
            self.gripper.open_gripper()
            target_pos[2] += 100.0
            movel(target_pos, vel=VELOCITY, acc=ACC)
            JReady = [0, 0, 90, 0, 90, 0]
            movej(JReady, vel=VELOCITY, acc=ACC)

if __name__ == "__main__":
    main_logic()