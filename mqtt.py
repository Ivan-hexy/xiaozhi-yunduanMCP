import paho.mqtt.client as mqtt
import logging
import ast
import time
import socket

# 配置日志
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('MQTT_Receiver')

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
        logger.info(f"成功连接到MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
        logger.info(f"本地IP地址: {get_local_ip()}")
        client.subscribe(MQTT_TOPICS)
        logger.info(f"已订阅主题: {[t[0] for t in MQTT_TOPICS]}")
    else:
        logger.error(f"连接失败，错误码: {rc} - {mqtt.connack_string(rc)}")

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    logger.debug(f"订阅确认: MID={mid}, QOS={granted_qos}")

def on_disconnect(client, userdata, rc, properties=None):
    if rc != 0:
        logger.warning(f"意外断开连接，错误码: {rc}")
        logger.info("尝试重新连接...")
        try:
            client.reconnect()
        except Exception as e:
            logger.error(f"重连失败: {str(e)}")

def on_message(client, userdata, msg):
    logger.info(f"收到消息 [主题: {msg.topic}]")
    logger.debug(f"原始消息: {msg.payload}")
    
    try:
        payload_str = msg.payload.decode('utf-8')
        logger.debug(f"解码内容: {payload_str}")
        
        # 尝试解析为Python字典
        try:
            payload = ast.literal_eval(payload_str)
            logger.info(f"解析后的导航指令: {payload}")
        except:
            # 如果不是Python字典格式，尝试作为纯文本处理
            payload = payload_str
            logger.warning("消息不是字典格式，作为文本处理")
        
        # 处理导航指令
        if "gooffice" in msg.topic:
            handle_navigation("办公室", payload)
        elif "gorestroom" in msg.topic:
            handle_navigation("休息室", payload)
            
    except Exception as e:
        logger.error(f"处理消息时出错: {str(e)}")

def handle_navigation(destination, payload):
    """处理导航指令"""
    logger.info(f"开始处理{destination}导航指令")
    
    # 如果是字典类型，提取参数
    if isinstance(payload, dict):
        x = payload.get('x', 0)
        y = payload.get('y', 0)
        yaw = payload.get('yaw', 0)
        command = payload.get('command', 0)
        
        logger.info(f"目标坐标: ({x}, {y}), 朝向: {yaw}弧度")
        
        # 命令处理
        command_actions = {
            0: "无操作",
            1: "执行夹取动作",
            2: "执行释放动作"
        }
        action = command_actions.get(command, "未知命令")
        logger.info(f"附加命令: {action} (代码: {command})")

    else:
        logger.info(f"接收到的指令内容: {payload}")
    
    logger.info(f"{destination}导航指令处理完成\n")

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
        logger.info("启动MQTT监听循环...")
        client.loop_forever()
    except Exception as e:
        logger.error(f"连接异常: {str(e)}")

if __name__ == "__main__":
    logger.info("启动MQTT导航指令接收器")
    logger.info(f"目标Broker: {MQTT_BROKER}:{MQTT_PORT}")
    logger.info(f"本地IP地址: {get_local_ip()}")
    
    try:
        start_mqtt_client()
    except KeyboardInterrupt:
        logger.info("程序被用户中断")
    except Exception as e:
        logger.error(f"程序异常终止: {str(e)}")