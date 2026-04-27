import socket
import struct
import threading
import time

# 兼容旧版本drive_dog
# 定义用于动作控制的命令常量
CMD_VOICECONTROL = 0x21010C0A
CMD_STOP_ALL = 0x21010130  # 停止所有动作
CMD_STOP_TURNING = 0x21010131  # 停止转向
CMD_SQUAT = 0x21010202  # 蹲下/站姿
CMD_DANCE = 0x21010204  # 跳舞动作
CMD_CONTROL = 0x21010C0A  # 点头/摇头
CMD_MUSIC = 0x2101030D  # 运动扬声器
CMD_AUDIO = 0x21010C0A  # 音频控制
CMD_SOPEAKER = 0x2101030D  # 扬声器控制
CMD_TWIST = 0x2101020D  # 扭转动作
CMD_NORMAL_GAIT = 0x21010300  # 正常步态（低速）
CMD_MEDIUM_GAIT = 0x21010307  # 中速步态
CMD_HIGH_GAIT = 0x21010303  # 高速步态
CMD_HIGH_STEP_GAIT = 0x21010407  # 高踏步步态
CMD_GENERAL_GAIT = 0x21010401  # 通用越障步态
CMD_GRASP_GAIT = 0x21010402  # 抓地越障步态
CMD_SPACE_WALK = 0x2101030C  # 太空步步态
CMD_MOVE = 0x21010D06  # 移动控制
CMD_stay = 0x21010D05  # 停留/站立指令
CMD_spin = 0x21010135  # 旋转指令

# 根据官方文档更新的完整指令集
CMD = {
    # 1.2.1 心跳
    "HEARTBEAT": 0x21040001,

    # 1.2.2 机器人基本状态转换指令
    "STAND_TOGGLE": 0x21010202,  # 在趴下状态和初始站立状态之间轮流切换
    "EMERGENCY_STOP": 0x21020C0E,  # 使机器人软急停
    "INIT_JOINTS": 0x21010C05,  # 初始化机器人关节

    # 1.2.3 轴指令
    "MOVE_Y": 0x21010131,  # y轴移动/翻滚
    "MOVE_X": 0x21010130,  # x轴移动/低头
    "BODY_HEIGHT": 0x21010102,  # 身体高度
    "TURN": 0x21010135,  # 旋转

    # 1.2.4 运动模式切换指令
    "MODE_STAY": 0x21010D05,  # 切换到原地模式
    "MODE_MOVE": 0x21010D06,  # 切换到移动模式

    # 1.2.5 步态切换指令
    "GAIT_LOW": 0x21010300,  # 低速步态
    "GAIT_MEDIUM": 0x21010307,  # 中速步态
    "GAIT_HIGH": 0x21010303,  # 高速步态
    "GAIT_CRAWL": 0x21010406,  # 匍匐/正常步态切换
    "GAIT_GRASP": 0x21010402,  # 抓地步态
    "GAIT_GENERAL": 0x21010401,  # 通用步态
    "GAIT_HIGH_STEP": 0x21010407,  # 高踏步步态

    # 1.2.6 动作指令
    "ACTION_TWIST": 0x21010204,  # 扭身体
    "ACTION_FLIP": 0x21010205,  # 翻身
    "ACTION_SPACEWALK": 0x2101030C,  # 太空步
    "ACTION_BACKFLIP": 0x21010502,  # 后空翻
    "ACTION_GREET": 0x21010507,  # 打招呼
    "ACTION_JUMP_FORWARD": 0x2101050B,  # 向前跳
    "ACTION_TWIST_JUMP": 0x2101020D,  # 扭身跳

    # 1.2.7 控制模式切换指令
    "MODE_AUTO": 0x21010C03,  # 切换到自主模式
    "MODE_MANUAL": 0x21010C02,  # 切换到手动模式

    # 1.2.8 保存数据指令
    "SAVE_DATA": 0x21010C01,  # 保存前100秒数据

    # 1.2.9 持续运动指令
    "CONTINUOUS_MOTION": 0x21010C06,  # 持续运动控制

    # 1.2.10 语音指令
    "VOICE_CONTROL": 0x21010C0A,  # 语音控制指令
    "SPEAKER_CONTROL": 0x2101030D,  # 扬声器控制

    # 1.2.11 感知设置类指令
    "AI_SETTINGS": 0x21012109,  # AI功能设置

    # 1.2.12 速度指令
    "SPEED_TURN": 0x0141,  # 旋转角速度
    "SPEED_FORWARD": 0x0140,  # 前后平移速度
    "SPEED_SIDE": 0x0145,  # 左右平移速度
}

# 语音指令参数
VOICE_COMMANDS = {
    "STAND": 1,  # 起立
    "SIT": 2,  # 坐下
    "FORWARD": 3,  # 前进
    "BACKWARD": 4,  # 后退
    "LEFT": 5,  # 向左平移
    "RIGHT": 6,  # 向右平移
    "STOP": 7,  # 停止
    "HEAD_DOWN": 8,  # 低头
    "HEAD_UP": 9,  # 抬头
    "LOOK_LEFT": 11,  # 向左看
    "LOOK_RIGHT": 12,  # 向右看
    "TURN_LEFT_90": 13,  # 向左转90°
    "TURN_RIGHT_90": 14,  # 向右转90°
    "TURN_180": 15,  # 向后转180°
    "GREET": 22,  # 打招呼
}


class Controller:
    """
    控制四足机器人动作的类，通过 UDP 协议发送指令给机器人。
    基于官方文档的完整指令集实现。

    初始化流程：
    1. 连接机器人 - 建立通信连接
    2. 启动心跳 - 维持稳定连接
    3. 初始化关节 - 初始化机器人关节
    4. 让机器人起立 - 使用语音指令起立
    5. 切换到移动模式 - 切换到移动模式
    6. 切换到自动模式 - 切换到自动模式
    7. 完成初始化 - 机器人准备就绪

    属性:
        lock (bool): 是否锁定操作，防止重复触发动作
        last_ges (str): 上一次执行的动作名称
        socket (socket.socket): UDP 套接字对象
        dst (tuple): 目标地址 (IP, Port)
    """
    # 语音控制指令常量
    STAND_UP = 1           # 起立
    STAND_DOWN = 2         # 蹲下
    MOVE_FORWARD = 3       # 前进
    MOVE_BACKWARD = 4      # 后退
    MOVE_LEFT = 5          # 左移
    MOVE_RIGHT = 6         # 右移
    STOP = 7               # 停止
    BOW = 8                # 鞠躬
    RAISE = 9              # 抬起
    JUMP = 10              # 跳跃
    STAND_TURN_LEFT = 11   # 原地左看
    STAND_TURN_RIGHT = 12  # 原地右看
    TURN_LEFT = 13         # 左转
    TURN_RIGHT = 14        # 右转
    TURN_BACK = 15         # 掉头
    TURN_TARGET_ANGLE = 16 # 转向指定角度
    
    def __init__(self, dst):
        """
        初始化控制器

        参数:
            dst (tuple): 服务器地址 (ip, port)
        """
        self.lock = False  # 动作锁，防止并发动作冲突
        self.last_ges = "stop"  # 记录上一个动作
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建 UDP 套接字
        self.dst = dst  # 设置目标地址（IP + 端口）
        
        # 心跳线程相关
        self.heartbeat_running = False
        self.heartbeat_thread = None
        
        # 持续移动线程相关
        self.move_running = False
        self.move_thread = None
        self.move_params = (0.0, 0.0, 0.0)

    def send(self, pack):
        """
        发送原始数据包到目标地址,带异常处理

        参数:
            pack (bytes): 打包好的二进制数据
        """
        try:
            self.socket.sendto(pack, self.dst)
        except socket.error as e:
            print(f"❌ 发送失败: {e}")
        except Exception as e:
            print(f"❌ 未知错误: {e}")

    def _send_command(self, command, val1=0, val2=0):
        """
        向机器人发送指定格式的命令包

        参数:
            command (int): 命令码
            val1 (int): 参数1
            val2 (int): 参数2
        """
        self.send(struct.pack('<3i', command, val1, val2))  # 打包成 '<3i' 格式：3个整数，小端序

    def _start_threaded_action(self, action_func):
        """
        在新线程中启动指定动作函数，避免阻塞主程序

        参数:
            action_func (function): 要运行的动作函数
        """
        thread = threading.Thread(target=action_func)
        thread.start()

    def heartbeat(self):
        """发送心跳信号以保持连接。必须以大于1Hz的频率发送。"""
        self._send_command(CMD["HEARTBEAT"])
    
    def start_heartbeat(self, frequency=2.0):
        """
        启动自动心跳线程
        
        参数:
            frequency (float): 心跳频率(Hz)，默认2Hz，文档要求>1Hz
        """
        if not self.heartbeat_running:
            self.heartbeat_running = True
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, args=(frequency,), daemon=True)
            self.heartbeat_thread.start()
            print(f"✓ 心跳线程已启动 (频率: {frequency}Hz)")
    
    def _heartbeat_loop(self, frequency):
        """心跳循环"""
        interval = 1.0 / frequency
        while self.heartbeat_running:
            self.heartbeat()
            time.sleep(interval)
    
    def stop_heartbeat(self):
        """停止心跳线程"""
        if self.heartbeat_running:
            self.heartbeat_running = False
            if self.heartbeat_thread:
                self.heartbeat_thread.join(timeout=2)
            print("✓ 心跳线程已停止")

    def init_robot(self):
        """初始化机器人关节"""
        print("初始化机器人关节...")
        self._send_command(CMD["INIT_JOINTS"])
        time.sleep(10)  # 等待初始化完成
    
    def initialize(self):
        """
        完整的机器人初始化流程:
        1. 启动心跳
        2. 初始化关节
        3. 起立
        4. 切换到移动模式
        5. 切换到自动模式
        """
        print("\n" + "="*50)
        print("🤖 开始初始化机器人")
        print("="*50)
        
        try:
            # 1. 启动心跳
            print("\n[1/4] 启动心跳...")
            self.start_heartbeat(frequency=2.0)
            time.sleep(1)
            
            # 2. 初始化关节
            # print("\n[2/4] 初始化关节...")
            # self.init_robot()
            
            # 3. 起立
            print("\n[3/4] 机器人起立...")
            self.voice_command("STAND")
            time.sleep(3)
            
            # 4. 切换到移动模式
            print("\n[4/4] 切换到移动模式...")
            self.switch_to_move_mode()
            time.sleep(1)

            # 5. 切换到自动模式
            print("\n[5/4] 切换到自动模式...")
            self.switch_to_auto_mode()
            time.sleep(1)

            print("\n" + "="*50)
            print("✓ 机器人初始化完成，准备就绪！")
            print("="*50 + "\n")
            return True
            
        except Exception as e:
            print(f"\n❌ 初始化失败: {e}")
            self.stop_heartbeat()
            return False

    def stand_toggle(self):
        """在趴下状态和站立状态之间切换"""
        print("切换站立状态")
        self._send_command(CMD["STAND_TOGGLE"])

    def emergency_stop(self):
        """紧急停止"""
        print("紧急停止！")
        self._send_command(CMD["EMERGENCY_STOP"])

    def switch_to_move_mode(self):
        """切换到移动模式"""
        print("切换到移动模式")
        self._send_command(CMD["MODE_MOVE"])

    def switch_to_stay_mode(self):
        """切换到原地模式"""
        print("切换到原地模式")
        self._send_command(CMD["MODE_STAY"])
        
    def switch_to_auto_mode(self):
        """切换到自动控制模式"""
        print("切换到自动控制模式")
        self._send_command(CMD["MODE_AUTO"], 0, -1)
        
    def switch_to_manual_mode(self):
        """切换到手动控制模式"""
        print("切换到手动控制模式")
        self._send_command(CMD["MODE_MANUAL"], 0, -1)

    def set_gait(self, gait_type):
        """
        设置步态

        参数:
            gait_type (str): 步态类型 ('low', 'medium', 'high', 'grasp', 'general', 'high_step', 'crawl')
        """
        gait_commands = {
            'low': CMD["GAIT_LOW"],
            'medium': CMD["GAIT_MEDIUM"],
            'high': CMD["GAIT_HIGH"],
            'grasp': CMD["GAIT_GRASP"],
            'general': CMD["GAIT_GENERAL"],
            'high_step': CMD["GAIT_HIGH_STEP"],
            'crawl': CMD["GAIT_CRAWL"]
        }

        if gait_type in gait_commands:
            print(f"设置步态: {gait_type}")
            self._send_command(gait_commands[gait_type])
        else:
            print(f"未知步态类型: {gait_type}")

    def voice_command(self, command_name):
        """
        发送语音指令

        参数:
            command_name (str): 语音指令名称
        """
        if command_name in VOICE_COMMANDS:
            print(f"发送语音指令: {command_name}")
            self._send_command(CMD["VOICE_CONTROL"], VOICE_COMMANDS[command_name])
        else:
            print(f"未知语音指令: {command_name}")

    def _start_threaded_action(self, action_func):
        """
        启动线程执行动作并释放锁

        参数:
            action_func (function): 动作函数
        """
        thread = threading.Thread(target=action_func)
        thread.start()

    def rotate_degrees(self, angle):
        """
        控制机器人顺时针旋转指定角度

        参数:
            angle (int): 需要旋转的角度（度）
        """

        def _rotate():
            self.lock = True
            # 发送旋转命令（假设 val=13000 表示最大速度）
            self._send_command(0x21010135, 13000, 0)
            # 根据角度计算旋转时间（假设 90 度需要 1.42 秒）
            rotation_time = (angle / 90.0) * 1.42
            time.sleep(rotation_time)
            # 停止旋转
            self._send_command(0x21010135, 0, 0)
            self.lock = False

        thread = threading.Thread(target=_rotate)
        thread.start()
        thread.join()  # 等待旋转完成再继续执行

    def send_voice_command(self, cmd_value):
        """
        发送语音指令到机器狗
        
        参数:
            cmd_value (int): 指令值
        """
        cmd = struct.pack('<3i', CMD_VOICECONTROL, cmd_value, 0)
        self.send(cmd)
        print(f"发送语音指令: {cmd_value} ({self.get_voice_command_name(cmd_value)}) (0x21010C0A)")

    def stand_up(self, seconds=2):
        """
        起立 - 使机器狗从蹲下状态变为站立状态
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.STAND_UP)
        time.sleep(seconds)
        self.stop()

    def stand_down(self, seconds=2):
        """
        蹲下 - 使机器狗从站立状态变为蹲下状态
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.STAND_DOWN)
        time.sleep(seconds)
        self.stop()

    def move_forward(self, seconds=1):
        """
        前进 - 使机器狗向前行走
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.MOVE_FORWARD)
        time.sleep(seconds)
        self.stop()

    def move_backward(self, seconds=1):
        """
        后退 - 使机器狗向后行走
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.MOVE_BACKWARD)
        time.sleep(seconds)
        self.stop()

    def move_left(self, seconds=1):
        """
        左移 - 使机器狗向左横向移动
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.MOVE_LEFT)
        time.sleep(seconds)
        self.stop()

    def move_right(self, seconds=1):
        """
        右移 - 使机器狗向右横向移动
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.MOVE_RIGHT)
        time.sleep(seconds)
        self.stop()

    def stop(self, seconds=0.1):
        """
        停止 - 停止当前所有动作
        """
        self.send_voice_command(self.STOP)
        time.sleep(seconds)

    def bow(self, seconds=1):
        """
        鞠躬 - 使机器狗执行鞠躬动作
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.BOW)
        time.sleep(seconds)
        self.stop()

    def raise_up(self, seconds=1):
        """
        抬起 - 使机器狗执行抬起动作
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.RAISE)
        time.sleep(seconds)
        self.stop()

    def jump(self, seconds=1):
        """
        跳跃 - 使机器狗执行跳跃动作
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.JUMP)
        time.sleep(seconds)
        self.stop()

    def stand_turn_left(self, seconds=1):
        """
        原地左看 - 使机器狗在原地向左旋转
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.STAND_TURN_LEFT)
        time.sleep(seconds)
        self.stop()

    def stand_turn_right(self, seconds=1):
        """
        原地右看 - 使机器狗在原地向右旋转
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.STAND_TURN_RIGHT)
        time.sleep(seconds)
        self.stop()

    def turn_left(self, seconds=2):
        """
        左转 - 使机器狗向左转向
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.TURN_LEFT)
        time.sleep(seconds)
        self.stop()

    def turn_right(self, seconds=2):
        """
        右转 - 使机器狗向右转向
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.TURN_RIGHT)
        time.sleep(seconds)
        self.stop()

    def turn_back(self, seconds=4):
        """
        掉头 - 使机器狗掉头180度
        
        参数:
            seconds (int): 执行时间(秒)
        """
        self.send_voice_command(self.TURN_BACK)
        time.sleep(seconds)
        self.stop()

    def nod(self):
        """
        点头 - 使机器狗点头
        """
        self.bow(0.5)
        self.raise_up(0.5)
        self.bow(0.5)
        self.raise_up(0.5)
    def shake(self):
        """
        摇头 - 使机器狗摇头
        """
        self.stand_turn_left(0.5)
        self.stand_turn_right(0.5)
        self.stand_turn_left(0.5)
        self.stand_turn_right(0.5)
    def turn_target_angle(self, angle=0):
        """
        转向指定角度 - 使机器狗转向指定角度
        
        参数:
            angle (int): 目标角度(度)
        """
        # 这里需要根据实际情况实现角度控制
        self.send_voice_command(self.TURN_TARGET_ANGLE)

    def stop_music(self):
        """
        停止机器人播放音乐
        示例用法: controller.stop_music()
        """
        print(f"[动作开始] 停止音乐 - CMD指令码: 0x2101030D")
        self._send_command(0x2101030D, 0, 0)
        time.sleep(0.5)
            

    @staticmethod
    def get_voice_command_name(cmd_value):
        """
        根据指令值获取指令名称
        
        参数:
            cmd_value (int): 指令值
            
        返回:
            str: 指令名称
        """
        commands = {
            Controller.STAND_UP: "STAND_UP",
            Controller.STAND_DOWN: "STAND_DOWN",
            Controller.MOVE_FORWARD: "MOVE_FORWARD",
            Controller.MOVE_BACKWARD: "MOVE_BACKWARD",
            Controller.MOVE_LEFT: "MOVE_LEFT",
            Controller.MOVE_RIGHT: "MOVE_RIGHT",
            Controller.STOP: "STOP",
            Controller.BOW: "BOW",
            Controller.RAISE: "RAISE",
            Controller.JUMP: "JUMP",
            Controller.STAND_TURN_LEFT: "STAND_TURN_LEFT",
            Controller.STAND_TURN_RIGHT: "STAND_TURN_RIGHT",
            Controller.TURN_LEFT: "TURN_LEFT",
            Controller.TURN_RIGHT: "TURN_RIGHT",
            Controller.TURN_BACK: "TURN_BACK",
            Controller.TURN_TARGET_ANGLE: "TURN_TARGET_ANGLE"
        }
        return commands.get(cmd_value, "UNKNOWN_COMMAND")

    def move(self, forward_speed=0.0, side_speed=0.0, turn_speed=0.0):
        """
        发送移动指令（移动模式下的轴指令）- 单次发送

        参数:
            forward_speed (float): 前后平移速度 (-1.0 到 1.0)，正值向前
            side_speed (float): 左右平移速度 (-1.0 到 1.0)，正值向右
            turn_speed (float): 左右转弯角速度 (-1.0 到 1.0)，正值向右转

        注意：
            - 轴指令取值范围为[-32767,32767]
            - 死区范围（低于此值机器人将停止移动）：
              * 前后平移: [-6553,6553] 对应速度约 ±0.2
              * 左右平移: [-12553,12553] 对应速度约 ±0.38
              * 左右转弯: [-9553,9553] 对应速度约 ±0.29
            - 建议设置速度参数时避免死区，确保有效控制
            - 轴指令频率应≥20Hz，超时250ms，建议使用start_continuous_move()
        """
        # 将-1.0到1.0的速度映射到指令值范围
        forward_val = int(forward_speed * 32767)
        side_val = int(side_speed * 32767)
        turn_val = int(turn_speed * 32767)

        # 移动模式下的轴指令：
        # MOVE_X (0x21010130): 前后平移，指定机器人x轴上的期望线速度，正值向前
        # MOVE_Y (0x21010131): 左右平移，指定机器人y轴上的期望线速度，正值向右
        # TURN (0x21010135): 左右转弯，指定机器人的期望角速度，正值向右转
        self._send_command(CMD["MOVE_X"], forward_val)
        self._send_command(CMD["MOVE_Y"], side_val)
        self._send_command(CMD["TURN"], turn_val)
    
    def start_continuous_move(self, forward_speed=0.0, side_speed=0.0, turn_speed=0.0, frequency=20):
        """
        启动持续移动模式，以指定频率持续发送轴指令
        文档要求：轴指令频率≥20Hz，超时时间250ms
        
        参数:
            forward_speed (float): 前后平移速度 (-1.0 到 1.0)
            side_speed (float): 左右平移速度 (-1.0 到 1.0)
            turn_speed (float): 转弯速度 (-1.0 到 1.0)
            frequency (int): 发送频率(Hz)，默认20Hz
        """
        if self.move_running:
            # 如果已经在运行，更新参数
            self.move_params = (forward_speed, side_speed, turn_speed)
            # print(f"✓ 更新移动参数: 前进={forward_speed:.2f}, 侧移={side_speed:.2f}, 转向={turn_speed:.2f}")
        else:
            self.move_running = True
            self.move_params = (forward_speed, side_speed, turn_speed)
            
            def move_loop():
                interval = 1.0 / frequency
                while self.move_running:
                    self.move(*self.move_params)
                    time.sleep(interval)
            
            self.move_thread = threading.Thread(target=move_loop, daemon=True)
            self.move_thread.start()
            print(f"✓ 持续移动已启动 (频率: {frequency}Hz)")
    
    def stop_continuous_move(self):
        """停止持续移动模式"""
        if self.move_running:
            self.move_running = False
            if self.move_thread:
                self.move_thread.join(timeout=1)
            time.sleep(0.1)
            self.stop()
            print("✓ 持续移动已停止")

    def stop(self):
        """停止所有移动"""
        print("停止移动")
        # 发送0值轴指令来停止运动
        # 根据文档：机器人在运动过程中，若下发轴指令为0，机器人将停止运动
        self._send_command(CMD["MOVE_X"], 0)
        self._send_command(CMD["MOVE_Y"], 0)
        self._send_command(CMD["TURN"], 0)

    def stop_voice_command(self,seconds = 0.1):
        """
              停止 - 停止当前所有语音控制动作
         """
        self.voice_command("STOP")
        time.sleep(seconds)

    #兼容旧版本drive_dog,会清除高速步态慎用
    def drive_dog(self, ges, val=10000, time_val=0.1):
        """
        控制机器人执行特定动作

        参数:
            ges (str): 动作名称（如 "squat", "nod", "shake"）
            val (int): 动作幅度参数，默认为 10000
        """

        if self.lock :
            return  # 如果处于锁定状态或重复执行相同动作则返回

        # 如果上一个动作为蹲下且当前不是蹲下，则先站立
        if self.last_ges == "squat" and ges != "squat":
            self._send_command(CMD_SQUAT)
        else:
            # 先停止所有动作
            self._send_command(CMD_STOP_ALL, 0, 0)
            self._send_command(CMD_STOP_TURNING, 0, 0)
            self._send_command(CMD_STOP_ALL, 0, 0)
            self._send_command(0x21010135, 0, 0)  # 特定停止命令

        # 根据不同的动作执行相应逻辑
        if self.last_ges != "squat" and ges == "squat":
            # 执行蹲下动作
            self._send_command(CMD_SQUAT)

        elif ges == "turning":
            # 转身360度动作，使用线程避免阻塞主线程
            def turn_360():
                self.lock = True
                self._send_command(0x21010135, 13000, 0)  # 开始转动
                time.sleep(2.84)  # 转动一圈所需时间
                self._send_command(0x21010135, 0, 0)  # 停止转动
                self.lock = False

            self._start_threaded_action(turn_360)

        elif ges == "twisting":
            # 跳舞动作，持续约22秒
            def dance():
                self.lock = True
                self._send_command(CMD_DANCE)
                time.sleep(22)  # 跳舞持续时间
                self.lock = False

            self._start_threaded_action(dance)

        elif ges == "forward":
            # 向前走
            self._send_command(CMD_STOP_ALL, val, 0)
            time.sleep(time_val)
        elif ges == "back":
            # 向后走
            self._send_command(CMD_STOP_ALL, -val, 0)
            time.sleep(time_val)
        elif ges == "right":
            # 向右转
            self._send_command(CMD_STOP_TURNING, val, 0)
            time.sleep(time_val)
        elif ges == "left":
            # 向左转
            self._send_command(CMD_STOP_TURNING, -val, 0)
            time.sleep(time_val)
        elif ges == "nod":
            # 点头动作：上下移动头部
            self.lock = True  # 设置锁
            self._send_command(CMD_CONTROL, 9, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 8, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 9, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 8, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 1, 0)
            time.sleep(0.5)
            self.lock = False  # 释放锁
        elif ges == "shake":
            # 摇头动作：左右摇头
            self.lock = True  # 设置锁
            self._send_command(CMD_CONTROL, 11, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 12, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 11, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 12, 0)
            time.sleep(0.25)
            self._send_command(CMD_CONTROL, 1, 0)
            time.sleep(0.5)
            self.lock = False  # 释放锁
        elif ges == 'colse_music':
            self._send_command(CMD_MUSIC, 0, 0)
            time.sleep(0.5)
        elif ges == 'open_music':
            self._send_command(CMD_MUSIC, 1, 0)
            time.sleep(0.5)
        self.last_ges = ges  # 更新最后执行的动作

    # 时间控制方法（保持兼容性）
    def forward_for_time(self, seconds):
        """向前行走指定时间"""
        print(f"前进 {seconds} 秒")

        def forward_action():
            self.lock = True
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(forward_speed=0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(forward_action)

    def backward_for_time(self, seconds):
        """向后行走指定时间"""
        print(f"后退 {seconds} 秒")

        def backward_action():
            self.lock = True
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(forward_speed=-0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(backward_action)

    def left_turn_for_time(self, seconds):
        """向左转指定时间"""
        print(f"左转 {seconds} 秒")

        def left_action():
            self.lock = True
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(turn_speed=-0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(left_action)

    def right_turn_for_time(self, seconds):
        """向右转指定时间"""
        print(f"右转 {seconds} 秒")

        def right_action():
            self.lock = True
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(turn_speed=0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(right_action)

    def nod_for_time(self, seconds):
        """点头指定时间"""
        print(f"点头 {seconds} 秒")

        def nod_action():
            start_time = time.time()
            self.lock = True
            while time.time() - start_time < seconds:
                self.voice_command("HEAD_DOWN")
                time.sleep(0.5)
                self.voice_command("HEAD_UP")
                time.sleep(0.5)
            self.lock = False

        self._start_threaded_action(nod_action)

    def shake_head_for_time(self, seconds):
        """摇头指定时间"""
        print(f"摇头 {seconds} 秒")

        def shake_action():
            start_time = time.time()
            self.lock = True
            while time.time() - start_time < seconds:

                self.voice_command("LOOK_LEFT")
                time.sleep(0.5)
                self.voice_command("LOOK_RIGHT")
                time.sleep(0.5)
            self.lock = False

        self._start_threaded_action(shake_action)

    # 步态方法
    def flat_gait_for_time(self, seconds, speed_mode='low'):
        """平底步态行走指定时间"""
        print(f"平底步态行走 {seconds} 秒，速度: {speed_mode}")

        def flat_gait_action():
            self.lock = True
            self.set_gait(speed_mode)
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(forward_speed=0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(flat_gait_action)

    def spacewalk_for_time(self, seconds):
        """太空步指定时间"""
        print(f"太空步 {seconds} 秒")

        def spacewalk_action():
            self.lock = True
            self._send_command(CMD["ACTION_SPACEWALK"])
            start_time = time.time()
            while time.time() - start_time < seconds:
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(spacewalk_action)

    def high_step_for_time(self, seconds):
        """高踏步指定时间"""
        print(f"高踏步 {seconds} 秒")

        def high_step_action():
            self.lock = True
            self.set_gait('high_step')
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(forward_speed=0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(high_step_action)

    def grasp_for_time(self, seconds):
        """抓地步指定时间"""
        print(f"抓地步 {seconds} 秒")

        def grasp_action():
            self.lock = True
            self.set_gait('grasp')
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(forward_speed=0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(grasp_action)

    def general_for_time(self, seconds):
        """通用越障步指定时间"""
        print(f"通用越障步 {seconds} 秒")

        def general_action():
            self.lock = True
            self.set_gait('general')
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(forward_speed=0.3)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(general_action)

    def turning(self, seconds):
        """转动指定时间"""
        print(f"转动 {seconds} 秒")

        def turning_action():
            self.lock = True
            start_time = time.time()
            while time.time() - start_time < seconds:
                self.move(turn_speed=0.5)
                time.sleep(0.1)
            self.stop()
            self.lock = False

        self._start_threaded_action(turning_action)

    def enable_continuous_motion(self):
        """开启持续运动"""
        print("开启持续运动")
        self._send_command(CMD["CONTINUOUS_MOTION"], -1)  # -1=开启

    def disable_continuous_motion(self):
        """关闭持续运动"""
        print("关闭持续运动")
        self._send_command(CMD["CONTINUOUS_MOTION"], 2)  # 2=关闭


    # 测试方法
    def simple_test(self):
        """简单的机器人响应测试"""
        print("=== 简单测试开始 ===")

        print("1. 测试心跳")
        self.heartbeat()
        time.sleep(1)

        print("2. 测试站立/蹲下")
        self.stand_toggle()
        time.sleep(3)

        print("3. 测试点头")
        self.voice_command("HEAD_DOWN")
        time.sleep(0.5)
        self.voice_command("HEAD_UP")
        time.sleep(0.5)

        print("=== 简单测试完成 ===")

    def test_movement_commands(self):
        """测试移动指令"""
        print("=== 开始测试移动指令 ===")

        print("测试1：前进")
        self.move(forward_speed=0.3)
        time.sleep(2)
        self.stop()

        time.sleep(1)

        print("测试2：转向")
        self.move(turn_speed=0.3)
        time.sleep(2)
        self.stop()

        print("=== 测试完成 ===")

    def stop_music(self):
        """
        停止机器人播放音乐
        示例用法: controller.stop_music()
        """
        print(f"[动作开始] 停止音乐 - CMD指令码: 0x2101030D")
        
        def stop_music_action():
            self.lock = True
            self._send_command(CMD_SOPEAKER, 0, 0)
            print("[动作结束] 停止音乐")
            self.lock = False
        
        self._start_threaded_action(stop_music_action)
    
    def close(self):
        """关闭控制器，释放所有资源"""
        print("\n关闭控制器...")
        
        # 停止所有线程
        self.stop_continuous_move()
        self.stop_heartbeat()
        
        # 发送紧急停止
        self.emergency_stop()
        time.sleep(0.5)
        
        # 关闭socket
        try:
            self.socket.close()
            print("✓ 控制器已关闭\n")
        except Exception as e:
            print(f"❌ 关闭socket失败: {e}\n")
    
    def __enter__(self):
        """支持上下文管理器 - 进入"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持上下文管理器 - 退出时自动清理"""
        self.close()
    
    def get_robot_state(self):
        pass

    def start_recording(self):
        pass

    def stop_recording(self):
        pass

    def save_recording(self, filename):
        pass
