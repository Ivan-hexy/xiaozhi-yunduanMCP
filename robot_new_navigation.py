from mcp.server.fastmcp import FastMCP
import sys
import logging
import paho.mqtt.client as mqtt

logger = logging.getLogger('RobotController')

# 修复Windows控制台的UTF-8编码
if sys.platform == 'win32':
    sys.stderr.reconfigure(encoding='utf-8')
    sys.stdout.reconfigure(encoding='utf-8')

# MQTT配置（区分不同目标地点的主题）
MQTT_BROKER = "192.168.18.164"  # MQTT broker地址
MQTT_PORT = 1883                # MQTT端口
MQTT_TOPIC_GOOFFICE = "robot/navigation/gooffice"    # 前往办公室的指令主题
MQTT_TOPIC_GORESTROOM = "robot/navigation/gorestroom"  # 前往休息室的指令主题

# 创建MCP服务器
mcp = FastMCP("RobotController")

# 连接MQTT服务器（共用的连接逻辑）
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        logger.info("MQTT连接成功")
    else:
        logger.error(f"MQTT连接失败，错误码: {rc}")  # 错误码1为认证失败


def connect_mqtt():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set("user1", "axjIWLAMMXA.SAKFN5321ZASHV")
    client.on_connect = on_connect  # 绑定连接回调
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    return client

# 通用导航并附加命令工具

def _send_navigation(client, topic, x, y, yaw, command):
    payload = {
        "x": x,
        "y": y,
        "yaw": yaw,
        "command": command
    }
    result = client.publish(topic, str(payload), qos=1)
    result.wait_for_publish()
    logger.info(f"已发送导航指令: {payload}")
    return payload

# 前往办公室的导航工具
@mcp.tool(name="self.robot.gooffice")
def go_to_office(command: int = 0) -> dict:
    """机器人目标地点是办公室，通过MQTT协议发送办公室的坐标指令，并附加命令（0:无操作,1:夹取,2:释放）"""
    try:
        # 办公室坐标：x=10, y=10, 偏航角=1.2弧度
        x, y, yaw = 10, 10, 1.2
        # 连接MQTT并发送消息到办公室主题
        client = connect_mqtt()
        client.loop_start()
        payload = _send_navigation(client, MQTT_TOPIC_GOOFFICE, x, y, yaw, command)
        client.loop_stop()
        client.disconnect()
        return {
            "success": True,
            "message": "已发送前往办公室的指令",
            "payload": payload
        }
    except Exception as e:
        logger.error(f"发送办公室指令失败: {str(e)}")
        return {
            "success": False,
            "error": str(e)
        }

# 前往休息室的导航工具
@mcp.tool(name="self.robot.gorestroom")
def go_to_restroom(command: int = 0) -> dict:
    """机器人目标地点是休息室，通过MQTT协议发送休息室的坐标指令，并附加命令（0:无操作,1:夹取,2:释放）"""
    try:
        # 休息室坐标：x=25, y=15, 偏航角=0.8弧度
        x, y, yaw = 25, 15, 0.8
        # 连接MQTT并发送消息到休息室主题
        client = connect_mqtt()
        client.loop_start()
        payload = _send_navigation(client, MQTT_TOPIC_GORESTROOM, x, y, yaw, command)
        client.loop_stop()
        client.disconnect()
        return {
            "success": True,
            "message": "已发送前往休息室的指令",
            "payload": payload
        }
    except Exception as e:
        logger.error(f"发送休息室指令失败: {str(e)}")
        return {
            "success": False,
            "error": str(e)
        }

# 启动服务器
if __name__ == "__main__":
    mcp.run(transport="stdio")
