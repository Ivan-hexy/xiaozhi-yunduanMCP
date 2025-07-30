# xiaozhi-yunduanMCP 
    本次任务的目标是使用搭载小智ai智能体的esp32识别用户的意图，发布命令让机器人移动到相应的位置并且做出指定的动作。  
    小智ai是基于开源项目：https://github.com/78/xiaozhi-esp32。项目推荐使用MCP协议去控制物联网设备、接入外部工具（使用 https://xiaozhi.me ）  
    实现语音对话并且向服务器发布任务有两种方法：
1.在esp32的代码中直接注册MCP方法，调用MQTT协议向服务器发送坐标信息和控制指令。具体的功能实现和参数细节参考main/mcp_server.cc 中 McpServer::AddCommonTools。  
void RobotController::RegisterMcpTools(McpServer& mcp_server) {
    mcp_server.AddTool(
        "self.robot.gooffice",
        "机器人目标地点是办公室",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            bool result = SendGoOfficeCommand();
            if (result) {
                return "机器人已收到前往办公室的指令";
            } else {
                return "发送前往办公室指令失败";
            }
        }
    );
}
然后需要在相应开发板的物联网初始化，添加对 AI 可见设备；注意初始化顺序，因为mqtt的初始化在物联网的初始化顺序之前。  
2.通过小智ai控制台的MCP接入点来让智能体使用自己编写的Python脚本方案更为方便快捷。文件robot_new_navigation.py实现了两个MCP工具：去办公室抓取/释放、去休息室抓取/释放。后续可以继续增添
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
if __name__ == "__main__":
    mcp.run(transport="stdio")

