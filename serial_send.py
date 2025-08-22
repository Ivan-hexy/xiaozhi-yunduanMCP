#!/usr/bin/python3
# coding=UTF-8
import serial
import threading
import time
from queue import Queue
import rospy
from std_msgs.msg import Bool, Int32

# 指令常量定义
SERVO_MOVE_TIME_WRITE = 1
SERVO_POS_READ = 28
SERVO_MOVE_TIME_DATA_LEN = 7
SERVO_POS_READ_DATA_LEN = 3
POS_ONE = 420 #小的放
POS_TWO = 550 #大的夹

class BusServoController:
    def __init__(self, port='/dev/ttyUSB2', baudrate=115200):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 0.1
        self.feedback_queue = Queue()
        self.running = False
        self.receive_thread = None
        self.expected_position = None  

    def open(self):
        """打开串口并启动接收线程"""
        try:
            if not self.ser.is_open:
                self.ser.open()
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False

    def close(self):
        """关闭串口和接收线程"""
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join()
        if self.ser.is_open:
            self.ser.close()

    def _calculate_checksum(self, data):
        """计算校验和"""
        checksum = sum(data) & 0xFF
        return (~checksum) & 0xFF

    def send_command(self, servo_id, cmd, prm1=0, prm2=0):
        """发送指令到舵机"""
        if not self.ser.is_open:
            print("串口未打开")
            return

        # 如果是移动指令，记录预期位置
        if cmd == SERVO_MOVE_TIME_WRITE:
            self.expected_position = prm1

        # 根据指令类型确定数据长度
        if cmd == SERVO_MOVE_TIME_WRITE:
            datalen = SERVO_MOVE_TIME_DATA_LEN
        elif cmd == SERVO_POS_READ:
            datalen = SERVO_POS_READ_DATA_LEN
        else:
            datalen = 3  # 默认数据长度

        # 构建指令帧
        frame = [0x55, 0x55]  # 帧头
        frame.append(servo_id & 0xFF)
        frame.append(datalen & 0xFF)
        frame.append(cmd & 0xFF)

        # 添加参数
        if cmd == SERVO_MOVE_TIME_WRITE:
            # 角度参数（prm1）
            frame.append(prm1 & 0xFF)
            frame.append((prm1 >> 8) & 0xFF)
            # 时间参数（prm2）
            frame.append(prm2 & 0xFF)
            frame.append((prm2 >> 8) & 0xFF)

        # 计算校验和
        checksum_data = frame[2:2 + datalen]  # ID到参数部分
        checksum = self._calculate_checksum(checksum_data)
        frame.append(checksum)

        # 发送数据
        try:
            self.ser.write(bytearray(frame))
            self.ser.flush()
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            return False

    def _receive_loop(self):
        """接收并解析舵机反馈"""
        buffer = bytearray()
        while self.running:
            # 读取串口数据
            data = self.ser.read(1)
            if data:
                buffer.extend(data)
                self._parse_buffer(buffer)
            else:
                time.sleep(0.001)

    def _parse_buffer(self, buffer):
        """解析接收缓冲区中的数据帧"""
        while len(buffer) >= 2:
            # 查找帧头
            if buffer[0] == 0x55 and buffer[1] == 0x55:
                if len(buffer) < 4:
                    break  # 数据不足，等待更多数据

                # 解析帧头信息
                servo_id = buffer[2]
                datalen = buffer[3]
                total_len = datalen + 3  # 总帧长

                if len(buffer) < total_len:
                    break  # 数据不足，等待更多数据

                # 提取完整帧
                frame = buffer[:total_len]
                del buffer[:total_len]

                # 验证校验和
                checksum_data = frame[2:2 + datalen]
                checksum = self._calculate_checksum(checksum_data)
                if checksum != frame[-1]:
                    print("校验和错误")
                    continue

                # 解析指令和参数
                cmd = frame[4]
                params = frame[5:5 + datalen - 1]  # 提取参数部分

                # 处理位置反馈
                if cmd == SERVO_POS_READ and len(params) >= 2:
                    angle = (params[1] << 8) | params[0]
                    self.feedback_queue.put({
                        'id': servo_id,
                        'cmd': cmd,
                        'angle': angle,
                        'raw_params': params
                    })
                else:
                    self.feedback_queue.put({
                        'id': servo_id,
                        'cmd': cmd,
                        'raw_params': params
                    })

            else:
                # 移除无效字节
                buffer.pop(0)

    def get_feedback(self, timeout=1):
        """获取舵机反馈信息"""
        try:
            return self.feedback_queue.get(timeout=timeout)
        except:
            return None

# 全局变量用于存储接收到的命令（带线程锁保证安全）
cmd_value = 0
cmd_lock = threading.Lock()

def cmd_callback(msg):
    """处理接收到的/cmdend_effector话题消息"""
    global cmd_value
    with cmd_lock:
        cmd_value = msg.data
    print(f"接收到命令: {cmd_value}")

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('servo_controller', anonymous=True)
    # 创建发布者，发布到end_effector话题
    pub = rospy.Publisher('end_effector_jiazhua', Bool, queue_size=10)
    # 创建订阅者，订阅/cmdend_effector话题
    rospy.Subscriber('/cmdeffector', Int32, cmd_callback)
    
    # 初始化控制器
    controller = BusServoController('/dev/ttyUSB2')
    if not controller.open():
        exit(1)

    try:
        while not rospy.is_shutdown():
            # 读取当前命令值（带锁访问）
            current_cmd = None
            with cmd_lock:
                current_cmd = cmd_value
                # 处理后重置命令值（避免重复执行）
                cmd_value = None

            # 抓取
            if current_cmd == 1:
                print(f"发送移动指令: 位置{POS_ONE}，时间1000ms")
                controller.send_command(1, SERVO_MOVE_TIME_WRITE, POS_ONE, 1000)
                time.sleep(1.5)
                print(f"发送移动指令: 位置{POS_TWO}，时间1000ms")
                controller.send_command(1, SERVO_MOVE_TIME_WRITE, POS_TWO, 1000)
                time.sleep(1.5)  
            # 释放
            elif current_cmd == 2:
                print(f"发送移动指令: 位置{POS_ONE}，时间1000ms")
                controller.send_command(1, SERVO_MOVE_TIME_WRITE, POS_ONE, 1000)
                time.sleep(1.5)  

            # 读取当前位置
            print("发送位置读取指令")
            controller.send_command(1, SERVO_POS_READ)
            time.sleep(0.2)

            # 获取反馈并处理
            feedback = controller.get_feedback()
            if feedback and feedback['cmd'] == SERVO_POS_READ:
                print(f"当前位置: {feedback['angle']}")
                
                # 计算预期位置与实际位置的差值
                if controller.expected_position is not None:
                    diff = abs(controller.expected_position - feedback['angle'])
                    print(f"位置差值: {diff}")
                    
                    current_result = diff < 15

                    pub.publish(current_result)
                    print(f"发布到end_effector: {current_result}")

                    if current_result:  
                        controller.expected_position = None
                        print("夹爪成功到达")

            # 循环间隔
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("程序退出")
    finally:
        controller.close()
