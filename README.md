# xiaozhi-yunduanMCP 

## 任务目标与实现方法

本次任务的目标是使用搭载小智AI智能体的ESP32识别用户意图，发布命令控制机器人移动到指定位置并执行动作。  

小智AI基于开源项目：[https://github.com/78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)，推荐通过 **MCP协议** 控制物联网设备或接入外部工具（工具平台：[https://xiaozhi.me](https://xiaozhi.me)）。  


### 实现方式：两种任务发布方法  
#### 方法1：ESP32代码中直接注册MCP方法（C++）  
在ESP32的代码里注册MCP方法，通过MQTT协议向服务器发送坐标与控制指令。核心逻辑参考 `main/mcp_server.cc` 中的 `McpServer::AddCommonTools` 函数。  

```cpp
void RobotController::RegisterMcpTools(McpServer& mcp_server) {
    mcp_server.AddTool(
        "self.robot.gooffice",          // 工具名称
        "机器人目标地点是办公室",      // 工具描述
        PropertyList(),                // 属性列表（此处为空）
        [this](const PropertyList& properties) -> ReturnValue {
            bool result = SendGoOfficeCommand();  // 发送指令的核心函数
            if (result) {
                return "机器人已收到前往办公室的指令";
            } else {
                return "发送前往办公室指令失败";
            }
        }
    );
}
```
在开发板的物联网初始化流程中，需添加对 AI 可见设备的配置。注意初始化顺序：在注册MCP工具前需要进行MQTT的初始化
#### 2. 通过小智 AI 控制台的 MCP 接入点

通过小智 AI 控制台的 MCP 接入点来让智能体使用自己编写的 Python 脚本方案更为方便快捷。文件 `robot_new_navigation.py` 实现了两个 MCP 工具：去办公室抓取/释放、去休息室抓取/释放。后续可以继续增添。

```python
@mcp.tool(name="self.robot.gooffice")
def go_to_office(command: int = 0) -> dict:
    """
    机器人目标地点是办公室，通过 MQTT 协议发送办公室的坐标指令，
    并附加命令（0: 无操作, 1: 夹取, 2: 释放）
    """
    try:
        # 办公室坐标：x=10, y=10, 偏航角=1.2 弧度
        x, y, yaw = 10, 10, 1.2
        # 连接 MQTT 并发送消息到办公室主题
        client = connect_mqtt()
        client.loop_start()
        payload = _send_navigation(
            client,
            MQTT_TOPIC_GOOFFICE,
            x, y, yaw,
            command
        )
        client.loop_stop()
        client.disconnect()
        return payload
    except Exception as e:
        return {"error": str(e)}

if __name__ == "__main__":
    mcp.run(transport="stdio")
```
## 本次任务学习内容
### esp32环境配置 
参考[https://blog.csdn.net/suoxd123/article/details/147766118](https:\\blog.csdn.net/suoxd123/article/details/147766118)和[https://ccnphfhqs21z.feishu.cn/wiki/F5krwD16viZoF0kKkvDcrZNYnhb](https:\\icnynnzcwou8.feishu.cn/wiki/JEYDwTTALi5s2zkGlFGcDiRknXf)    
配置、编译、调试、以及如何进行MCP接入可以参考我的csdn的博客： [https://blog.csdn.net/2301_76785393/article/details/149715130?spm=1001.2014.3001.5501](https:\\blog.csdn.net/2301_76785393/article/details/149715130?spm=1001.2014.3001.5501)
