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
### MQTT传输协议的学习    
MQTT是一种轻量级的发布（小智ai发布指令）/订阅消息（机器人）协议，由客户端（Publisher/Subscriber）通过 TCP/TLS 与中央 Broker 建立连接，以主题（Topic）为“地址”进行消息的发布和订阅，支持三种服务质量（QoS 0 “至多一次”、QoS 1 “至少一次”、QoS 2 “仅一次”）以平衡传输可靠性与开销。需要配置MQTT broker地址、端口号、发布的主题、用户名称、客户端id（client_id）、密码等等。
### MCP协议的学习
MCP是轻量级的点对点消息协议，主要用于大模型与外部服务之间的调用与数据交换。它以 JSON 格式封装请求与响应，通常包含“method”（方法名）、“params”（参数列表）和“id”（请求标识）等字段，支持调用远程功能、传递参数及异步接收返回结果；在实际应用中，MCP 消息可通过 WebSocket（本次任务小智ai通过此方式接入）、HTTP 或 MQTT 载体传输，使模型能够通过简单、统一的接口与机器人控制、数据库查询或其他微服务无缝对接，从而实现功能扩展与系统解耦。
### 导航的学习
##[https://blog.csdn.net/2301_76785393/article/details/148263876?sharetype=blogdetail&sharerId=148263876&sharerefer=PC&sharesource=2301_76785393&spm=1011.2480.3001.8118](https:\\blog.csdn.net/2301_76785393/article/details/148263876?sharetype=blogdetail&sharerId=148263876&sharerefer=PC&sharesource=2301_76785393&spm=1011.2480.3001.8118)
##[https://blog.csdn.net/2301_76785393/article/details/148366890?spm=1001.2014.3001.5501](https:\\blog.csdn.net/2301_76785393/article/details/148366890?spm=1001.2014.3001.5501)    
### 如何建图（3D）
1.连接机器人WiFi（密码：robot888.），使用ubuntu打开终端输入指令 ssh  kelo@uvc4 回车输入密码 robot20 进入机器人（如果使用机器人本机则不用）    
2.输入指令 service joypadWorker stop 回车输入密码停止机器人服务（不用卸载，如果不小心卸载了重新安装命令：rosrun robot_upstart install --job joypadWorker --user kelo --logdir ~/log/joypad --setup /etc/ros/setup.bash robot_boot/boot/robot_boot.launch，然后sudo systemctl daemon-reload && sudo systemctl start joypadWorker，最后service joypadWorker restart重启服务）        
3.输入指令 roslaunch robot_boot uvc4_3Dcreatmap_ls16_double_wj716.launch 回车进入扫图程序；此时可使用遥控手柄操控机器人在扫描区域内移动，扫描是由顶部多线雷达+底部左前单线雷达同时完成；为了确保单线雷达扫描质量完整，稳定速度控制机器人左前雷达贴近墙壁1.3米内进行移动，且移动时不能同时操控手柄的线速度和角速度摇杆；扫描期间所有物体保持静止状态，面积较大的场地需要分区域来回扫描，扫描完成后按键盘 Ctrl+c 结束。    
4.地图怎么使用参见导航建图文件夹       
5.使用地图进行导航参见第五章        
6.确定机器人的初始位置    
7.设定目标点，开始导航测试    
## 注意事项
小智ai联网：    
小智ai重新配网参考：[https://ccnphfhqs21z.feishu.cn/wiki/KGvIwjTQxiTxPCkRbbjcPnsLnMb](https:\\ccnphfhqs21z.feishu.cn/wiki/KGvIwjTQxiTxPCkRbbjcPnsLnMb)    
查看ubuntu系统的mosquitto是否开启（或者是安装mosquitto），使能、开启mosquitto，然后设置防火墙（没有则不用），查看wlan的ip（ubuntu）    
windows发送端记得修改ip地址、用户名、密码    
## 机器人操作步骤    
先连上机器人WiFi：turtle 密码：robot888. 机器人ssh连接：ssh kelo@uvc4 密码:robot20    
如果想使用rviz，需要ssh -Y kelo@uvc4，然后export ROS_IP=192.168.1.101（控制电脑的ip，使用ifconfig查询inet的地址），然后export  ROS_MASTER_URI=http://uvc4:11311，rviz -d rviz/nav.rviz
1.建图参见上面    
2.将小智ai连上网（参见上面）    
3.启动智能体的MCP功能，在小智ai控制台找mcp接入点，依次输入：    
cd C:\Users\ivanhe\Desktop\mcp_mqtt（改为代码文件夹）      
set MCP_ENDPOINT=<your_mcp_endpoint>（windows）/export MCP_ENDPOINT=<your_mcp_endpoint>（ubuntu）    
python mcp_pipe.py my_testtry.py（改为自己文件名）    
4.启动ros节点：mqtt_navigation_receiver和servo_controller（因为需要调试没有写.launch）    
rosrun task test_xiaozhi.py （文件名）        
rosrun task serial_demo.py （文件名）    
5.更改地图，cd noetic/src/robot_boot/maps；创建地图文件夹（名字都要一样）；cd noetic/src/robot_boot/robots；输入指令nano map.yaml回车打开后把map:后面修改为地图文件名，修改后按键盘Ctrl+O保存，按键盘Ctrl+X退出。        
6.校准位置：在rivz里面使用2D Pose Estimate校准底盘在地图的所在位置    
7.启动节点观测变量    
rostopic echo /cmdeffector     
rostopic echo /end_effector_jiazhua    
rostopic echo /move_base_simple/goal    
watch -n 0.1 "rostopic echo /arm_status -n 1"    
watch -n 0.1 "rostopic echo /navigation_status -n 1"    
rostopic echo /arm_drive     
