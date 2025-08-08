#!/usr/bin/python3
# coding=UTF-8
import rospy
from geometry_msgs.msg import PoseStamped,Quaternion
from navigation_msgs.msg import NavigationStatus
import math
from imrobot_msg.msg import ArmDrive, ArmStatus
import paho.mqtt.client as mqtt
import logging
import ast
import time
import socket
import threading  

class NavigationManager:
    def __init__(self):
        # 存储导航目标和命令
        self.nav_target = None  # 格式: {'x': x, 'y': y, 'yaw': yaw}
        self.command = 0
        self.nav_status = 0
        self.last_nav_status = None
        self.last_arm_running_status = True
        self.arm_running_status = False

        # 初始化ROS节点和发布者/订阅者
        self.init_ros()

        self.lock = threading.RLock() 

    def init_ros(self):
        # 初始化发布者
        self.pub_nav_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.pub_arm_drive = rospy.Publisher("/arm_drive", ArmDrive, queue_size=10)
        # 订阅导航状态
        rospy.Subscriber("/navigation_status", NavigationStatus, self.update_nav_status, queue_size=10)
        rospy.Subscriber("/arm_status", ArmStatus, self.update_arm_status, queue_size=10)

    def update_arm_status(self, data):
        """更新机械臂状态，重点关注running_status"""
        with self.lock:
            current_running_status = data.running_status
            if current_running_status != self.last_arm_running_status:
                self.arm_running_status = current_running_status
                self.last_arm_running_status = current_running_status  # 更新记录
                rospy.loginfo(f"机械臂运行状态更新: running_status={self.arm_running_status}")

    def update_nav_status(self, data):
        """更新导航状态"""
        with self.lock: 
            current_status = data.state
            if current_status != self.last_nav_status:
                self.nav_status = current_status
                self.last_nav_status = current_status                         
                rospy.loginfo(f"当前导航状态: {self.nav_status}")
                self.check_and_execute()

    def check_and_execute(self):
        """根据导航状态执行相应操作"""
        if self.nav_status == 2 and self.nav_target is not None:
            # 状态为2时发布目标位置
            self.publish_goal()
        elif self.nav_status == 3 and self.command != 0:
            # 状态为3时发布命令
            self.execute_command()

    def publish_goal(self):
        """发布导航目标"""
        if not self.nav_target:
            rospy.logwarn("没有可发布的导航目标")
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.nav_target['x']
        goal.pose.position.y = self.nav_target['y']
        goal.pose.position.z = 0.0

        # 将yaw转换为四元数（roll和pitch为0）
        q = rpy2elements(0, 0, self.nav_target['yaw'])
        goal.pose.orientation = q

        # 发布目标
        self.pub_nav_goal.publish(goal)
        rospy.loginfo(f"已发布导航目标: x={self.nav_target['x']}, y={self.nav_target['y']}, yaw={self.nav_target['yaw']}")

        # 发布后清空目标
        self.nav_target = None

    def execute_command(self):
        """执行命令（实现机械臂控制）"""
        rospy.loginfo(f"收到命令: {self.command}，准备执行机械臂操作...")
        with self.lock:
            # 检查机械臂是否处于可执行状态（running_status为False）
            if self.arm_running_status:
                rospy.logwarn("机械臂正在运行中（running_status=True），无法发布新指令")
                return

            # 根据命令类型设置机械臂参数
            arm_cmd = ArmDrive()
            if self.command == 0:
                # 搬运指令
                arm_cmd.x = -88.396
                arm_cmd.y = 39.867
                arm_cmd.z = -100.611
                arm_cmd.rx = -7.378
                arm_cmd.ry = 89.048
                arm_cmd.rz = 0
                rospy.loginfo("执行搬运指令")
            elif self.command == 1:
                # 夹取指令
                arm_cmd.x = -92.346
                arm_cmd.y = -50.122
                arm_cmd.z = -53.531
                arm_cmd.rx = 17.066
                arm_cmd.ry = 89.044
                arm_cmd.rz = 0
                rospy.loginfo("执行夹取指令")
            elif self.command == 2:
                # 释放指令
                arm_cmd.x = -92.346
                arm_cmd.y = -50.122
                arm_cmd.z = -53.531
                arm_cmd.rx = 17.066
                arm_cmd.ry = 89.044
                arm_cmd.rz = 0
                rospy.loginfo("执行释放指令")
            else:
                rospy.logwarn(f"未知命令: {self.command}，不执行任何操作")
                self.command = 0
                return

            # 发布机械臂控制指令
            self.pub_arm_drive.publish(arm_cmd)
            rospy.loginfo(f"已发布机械臂指令: {arm_cmd}")

            # 执行后清空命令
            self.command = 0

    def update_nav_target(self, x, y, yaw, command=0):
        """更新导航目标和命令"""
        with self.lock:
            self.nav_target = {'x': x, 'y': y, 'yaw': yaw}
            self.command = command
            print(self.nav_target,self.command)
            rospy.loginfo(f"更新导航目标: x={x}, y={y}, yaw={yaw}, command={command}")
            # 立即检查是否可以执行
            self.check_and_execute()

# 使用127.0.0.1连接本地Broker
MQTT_BROKER = "127.0.0.1"  # 改为回环地址连接本地Broker
MQTT_PORT = 1883
MQTT_TOPICS = [
    ("robot/navigation/gooffice", 1),
    ("robot/navigation/gorestroom", 1)
]

def get_local_ip():
    """获取本机IP地址"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "未知"

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        rospy.loginfo(f"成功连接到MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
        rospy.loginfo(f"本地IP地址: {get_local_ip()}")
        client.subscribe(MQTT_TOPICS)
        rospy.loginfo(f"已订阅主题: {[t[0] for t in MQTT_TOPICS]}")
    else:
        rospy.logerr(f"连接失败，错误码: {rc} - {mqtt.connack_string(rc)}")

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    rospy.logdebug(f"订阅确认: MID={mid}, QOS={granted_qos}")

def on_disconnect(client, userdata, rc, properties=None):
    if rc != 0:
        rospy.logwarn(f"意外断开连接，错误码: {rc}")
        rospy.loginfo("尝试重新连接...")
        try:
            client.reconnect()
        except Exception as e:
            rospy.logerr(f"重连失败: {str(e)}")

def on_message(client, userdata, msg):
    rospy.loginfo(f"收到消息 [主题: {msg.topic}]")
    rospy.logdebug(f"原始消息: {msg.payload}")
    
    try:
        payload_str = msg.payload.decode('utf-8')
        rospy.logdebug(f"解码内容: {payload_str}")
        
        # 尝试解析为Python字典
        try:
            payload = ast.literal_eval(payload_str)
            rospy.loginfo(f"解析后的导航指令: {payload}")
        except:
            # 如果不是Python字典格式，尝试作为纯文本处理
            payload = payload_str
            rospy.logwarn("消息不是字典格式，作为文本处理")
        
        # 处理导航指令
        if "gooffice" in msg.topic:
            handle_navigation("办公室", payload)
        elif "gorestroom" in msg.topic:
            handle_navigation("休息室", payload)
            
    except Exception as e:
        rospy.logerr(f"处理消息时出错: {str(e)}")

def rpy2elements(roll,pitch,yaw):
    """
    欧拉角转四元素
    """
    cy=math.cos(yaw * 0.5)
    sy=math.sin(yaw * 0.5)
    cp=math.cos(pitch * 0.5)
    sp=math.sin(pitch * 0.5)
    cr =math.cos(roll * 0.5)
    sr =math.sin(roll * 0.5)

    q=Quaternion()
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    q.w = cy * cp * cr + sy * sp * sr
    return q

def handle_navigation(destination, payload):
    """处理导航指令"""
    rospy.loginfo(f"开始处理{destination}导航指令")
    
    # 如果是字典类型，提取参数
    if isinstance(payload, dict):
        x = payload.get('x', 0)
        y = payload.get('y', 0)
        yaw = payload.get('yaw', 0)
        command = payload.get('command', 0)
        
        rospy.loginfo(f"目标坐标: ({x}, {y}), 朝向: {yaw}弧度")
        
        # 命令处理
        command_actions = {
            0: "无操作",
            1: "执行夹取动作",
            2: "执行释放动作"
        }
        action = command_actions.get(command, "未知命令")
        rospy.loginfo(f"附加命令: {action} (代码: {command})")

        # 更新导航目标
        if nav_manager:
            nav_manager.update_nav_target(x, y, yaw, command)
    else:
        rospy.loginfo(f"接收到的指令内容: {payload}")
    
    rospy.loginfo(f"{destination}导航指令处理完成\n")

def start_mqtt_client():
    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=f"nav_receiver_{get_local_ip()}"
    )
    
    # 设置回调函数
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.on_disconnect = on_disconnect
    
    # 设置连接参数
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        rospy.loginfo("启动MQTT监听循环...")
        client.loop_forever()  # 将在独立线程中运行，不阻塞ROS
    except Exception as e:
        rospy.logerr(f"连接异常: {str(e)}")

# 全局导航管理器实例
nav_manager = None

if __name__ == "__main__":
    rospy.loginfo("启动MQTT导航指令接收器")
    rospy.loginfo(f"目标Broker: {MQTT_BROKER}:{MQTT_PORT}")
    rospy.loginfo(f"本地IP地址: {get_local_ip()}")

    try:
        # 初始化ROS节点
        rospy.init_node('mqtt_navigation_receiver', anonymous=True)
        # 创建导航管理器
        nav_manager = NavigationManager()
        rospy.loginfo("导航管理器初始化完成")

        # 启动MQTT客户端线程  
        mqtt_thread = threading.Thread(target=start_mqtt_client, name='MQTTThread') 
        mqtt_thread.daemon = True  
        mqtt_thread.start()  
        rospy.loginfo("MQTT客户端线程已启动")  

        rospy.spin()  

    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序异常终止: {str(e)}")
