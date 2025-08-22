from mcp.server.fastmcp import FastMCP
import sys
import logging
import paho.mqtt.client as mqtt
import time

logger = logging.getLogger('RobotController')

# 修复Windows控制台的UTF-8编码
if sys.platform == 'win32':
    sys.stderr.reconfigure(encoding='utf-8')
    sys.stdout.reconfigure(encoding='utf-8')

# MQTT配置
MQTT_BROKER = "192.168.18.170"
MQTT_PORT = 1883
MQTT_TOPIC_GOOFFICE = "robot/navigation/gooffice"
MQTT_TOPIC_GORESTROOM = "robot/navigation/gorestroom"
MQTT_TOPIC_GOCORRIDOR = "robot/navigation/gocorridor"
MQTT_TOPIC_ARM_CONTROL = "robot/arm/control"

# 创建MCP服务器
mcp = FastMCP("RobotController")

# 连接回调函数
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        logger.info("✅ MQTT连接成功")
    else:
        logger.error(f"❌ MQTT连接失败，错误码: {rc}")

# 发布回调函数
def on_publish(client, userdata, mid, reason_code, properties):
    """发布回调函数 (VERSION2 API需要5个参数)"""
    logger.info(f"📤 消息已发送 (消息ID: {mid})")

def connect_mqtt():
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    return client

def _send_navigation(client, topic, x, y, yaw):
    """发送导航指令（不含command参数）"""
    payload = {
        "x": x,
        "y": y,
        "yaw": yaw
    }
    message = str(payload)
    logger.info(f"📨 发送消息到主题 '{topic}': {message}")
    result = client.publish(topic, message, qos=1)
    
    # 等待消息确认
    try:
        if result.wait_for_publish(timeout=5):
            logger.info("✅ 消息已确认送达")
        else:
            logger.warning("⚠️ 消息确认超时")
    except Exception as e:
        logger.error(f"❌ 消息发送错误: {str(e)}")
    
    return payload

def _send_arm_command(client, topic, command):
    """发送机械臂控制命令"""
    payload = {
        "command": command
    }
    message = str(payload)
    logger.info(f"📨 发送机械臂命令到主题 '{topic}': {message}")
    result = client.publish(topic, message, qos=1)
    
    # 等待消息确认
    try:
        if result.wait_for_publish(timeout=5):
            logger.info("✅ 机械臂命令已确认送达")
        else:
            logger.warning("⚠️ 机械臂命令确认超时")
    except Exception as e:
        logger.error(f"❌ 机械臂命令发送错误: {str(e)}")
    
    return payload

@mcp.tool(name="self.robot.arm_control")
def arm_control(command: int) -> dict:
    """机械臂控制命令
    仅机械臂操作（拿、放、搬、传递）：无需导航到目标点
    command参数说明:
    0: 机械臂回到原位
    1: 机械臂夹取（拿水）
    2: 机械臂释放（递给用户）
    3: 机械臂搬运
    """
    try:
        # 验证命令有效性
        if command not in [0, 1, 2, 3]:
            logger.error(f"❌ 无效的机械臂命令: {command}")
            return {"success": False, "error": "无效的命令，必须是0-3之间的整数"}
            
        client = connect_mqtt()
        client.loop_start()
        time.sleep(1)  # 等待连接建立
        
        if not client.is_connected():
            logger.error("❌ MQTT连接未建立，无法发送机械臂命令")
            return {"success": False, "error": "MQTT连接失败"}
        
        payload = _send_arm_command(client, MQTT_TOPIC_ARM_CONTROL, command)
        time.sleep(1)  # 确保消息发送完成
        client.loop_stop()
        client.disconnect()
        
        # 构建命令描述信息
        command_desc = {
            0: "回到原位",
            1: "夹取",
            2: "释放",
            3: "搬运"
        }[command]
        
        return {
            "success": True,
            "message": f"已发送机械臂{command_desc}命令（指令值: {command}）",
            "payload": payload
        }
    except Exception as e:
        logger.exception(f"❌ 发送机械臂命令失败")
        return {"success": False, "error": str(e)}


@mcp.tool(name="self.robot.complex_task")
def complex_task(location: str, arm_command: int) -> dict:
    """用户要求执行组合任务，使用complex_task函数
    执行组合任务：先导航到目标地点，再执行机械臂操作。
    参数说明:
    location: 目标地点，可选值为 "office"（办公室）或 "restroom"（休息室）或 "corridor"（走廊）
    arm_command: 机械臂命令，0-3之间的整数（0:回原位，1:夹取/拿水，2:释放，3:搬运）
    典型场景:
    - 去办公室拿一瓶水（需要发布导航目标点命令和机械臂夹取命令），去休息室拿一本书（需要发布导航目标点命令和机械臂夹取命令），将水放到休息室（需要发布导航目标点命令和机械臂搬运命令）
    - 仅导航（去办公室、去休息室、去走廊）：无需机械臂操作时，单独调用go_to_office/go_to_restroom/go_to_corridor
    - 仅机械臂操作（拿、放、搬、传递）：无需导航到目标点，单独调用arm_control
    """
    try:
        # 验证参数有效性
        if location not in ["office", "restroom", "corridor"]:
            return {"success": False, "error": "无效的目标地点，必须是'office'、'restroom'或'corridor'"}
        if arm_command not in [0, 1, 2, 3]:
            return {"success": False, "error": "无效的机械臂命令，必须是0-3之间的整数"}
        
        # 2. 执行导航
        logger.info(f"开始执行组合任务：前往{location}，执行机械臂命令{arm_command}")
        nav_result = None
        if location == "office":
            nav_result = go_to_office()
        elif location == "restroom":
            nav_result = go_to_restroom()
        elif location == "corridor":  
            nav_result = go_to_corridor()
        
        # 3. 如果导航失败，直接返回结果
        if not nav_result.get("success", False):
            return {
                "success": False,
                "error": f"导航失败：{nav_result.get('error', '未知错误')}",
                "navigation_result": nav_result
            }
        
        # 4. 导航成功后执行机械臂操作
        arm_result = arm_control(arm_command)
        
        # 5. 返回组合结果
        return {
            "success": arm_result.get("success", False),
            "message": f"已完成前往{location}并执行机械臂操作的任务",
            "navigation_result": nav_result,
            "arm_control_result": arm_result
        }
    except Exception as e:
        logger.exception(f"❌ 组合任务执行失败")
        return {"success": False, "error": str(e)}

@mcp.tool(name="self.robot.gooffice")
def go_to_office() -> dict:
    """机器人仅前往办公室（纯导航，不执行机械臂操作）
    注意：若用户要求需要执行拿水、搬运等操作，请使用complex_task函数
    """
    try:
        x, y, yaw = 74.814, 77.791, -1.598
        client = connect_mqtt()
        client.loop_start()
        time.sleep(1)  # 等待连接建立
        
        if not client.is_connected():
            logger.error("❌ MQTT连接未建立，无法发送消息")
            return {"success": False, "error": "MQTT连接失败"}
        
        payload = _send_navigation(client, MQTT_TOPIC_GOOFFICE, x, y, yaw)
        time.sleep(1)  # 确保消息发送完成
        client.loop_stop()
        client.disconnect()
        return {"success": True, "message": "已发送前往办公室的指令", "payload": payload}
    except Exception as e:
        logger.exception(f"❌ 发送办公室指令失败")
        return {"success": False, "error": str(e)}

@mcp.tool(name="self.robot.gorestroom")
def go_to_restroom() -> dict:
    """机器人仅前往休息室（纯导航，不执行机械臂操作）
    注意：若用户要求需要执行拿水、搬运等操作，请使用complex_task函数
    """
    try:
        x, y, yaw = 86.846, 92.542, 0.046
        client = connect_mqtt()
        client.loop_start()
        time.sleep(1)  # 等待连接建立
        
        if not client.is_connected():
            logger.error("❌ MQTT连接未建立，无法发送消息")
            return {"success": False, "error": "MQTT连接失败"}
        
        payload = _send_navigation(client, MQTT_TOPIC_GORESTROOM, x, y, yaw)
        time.sleep(1)  # 确保消息发送完成
        client.loop_stop()
        client.disconnect()
        return {"success": True, "message": "已发送前往休息室的指令", "payload": payload}
    except Exception as e:
        logger.exception(f"❌ 发送休息室指令失败")
        return {"success": False, "error": str(e)}

@mcp.tool(name="self.robot.gocorridor")
def go_to_corridor() -> dict:
    """机器人仅前往走廊（纯导航，不执行机械臂操作）
    注意：若用户要求需要执行拿水、搬运等操作，请使用complex_task函数
    """
    try:
        x, y, yaw = 97.407, 55.386, 1.7  # 走廊坐标
        client = connect_mqtt()
        client.loop_start()
        time.sleep(1)  # 等待连接建立
        
        if not client.is_connected():
            logger.error("❌ MQTT连接未建立，无法发送消息")
            return {"success": False, "error": "MQTT连接失败"}
        
        payload = _send_navigation(client, MQTT_TOPIC_GOCORRIDOR, x, y, yaw)
        time.sleep(1)  # 确保消息发送完成
        client.loop_stop()
        client.disconnect()
        return {"success": True, "message": "已发送前往走廊的指令", "payload": payload}
    except Exception as e:
        logger.exception(f"❌ 发送走廊指令失败")
        return {"success": False, "error": str(e)}
        
if __name__ == "__main__":
    mcp.run(transport="stdio")
