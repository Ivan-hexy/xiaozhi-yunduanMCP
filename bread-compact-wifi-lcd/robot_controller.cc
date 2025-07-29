#include "robot_controller.h"
#include <esp_log.h>

static const char* TAG = "RobotController";

RobotController::RobotController(Protocol* protocol) : protocol_(protocol) {  // 改为基类指针
    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol instance is null");
    }
}

bool RobotController::SendGoOfficeCommand() {
    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol is not initialized");
        return false;
    }

    // 构造符合协议要求的消息格式（参考 protocol.cc 中的 SendMcpMessage）
    std::string message = "{\"type\":\"command\",\"coordinates\":" + OFFICE_COORDINATES + "}";
    ESP_LOGI(TAG, "Sending go to office command: %s", message.c_str());
    return protocol_->SendText(message);  // 调用基类公共接口
}

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