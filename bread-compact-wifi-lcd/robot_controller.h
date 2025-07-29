#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "mcp_server.h"
#include "protocol.h"  // 改为基类头文件
#include <string>

class RobotController {
public:
    RobotController(Protocol* protocol);  // 接收基类指针
    ~RobotController() = default;

    bool SendGoOfficeCommand();
    void RegisterMcpTools(McpServer& mcp_server);

private:
    Protocol* protocol_;  // 基类指针
    const std::string GO_OFFICE_TOPIC = "robot/command/go";
    const std::string OFFICE_COORDINATES = "{\"x\":100,\"y\":100,\"yaw\":1.12}";
};

#endif