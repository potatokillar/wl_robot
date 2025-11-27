/*
 * @Author: 唐文浩
 * @Date: 2025-05-20
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-23
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <std_srvs/srv/empty.hpp>

#include "httplib.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_base/timer_tools.hpp"
#include "ws_server.hpp"
#include "http_api.hpp"
#include "update_service.hpp"
#include "version_info.hpp"
#include "version_switch.hpp"
#include "version_cleanup.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std;

class dev_server_node : public rclcpp::Node
{
public:
    dev_server_node() : Node("dev_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "dev_server_node start");

        server_.set_open_call([this](const net_client_info &info)
                              { on_open(info); });
        server_.set_close_call([this](const net_client_info &info)
                               { on_close(info); });
        server_.set_message_call([this](const net_client_info &info, const std::string &data)
                                 { on_message(info, data); });
        server_.set_message_call([this](const net_client_info &info, const std::vector<uint8_t> &data)
                                 { on_message(info, data); });

        ws_thread_ = thread([this]()
                            { server_.loop(); });

        // 使用一次性定时器延迟初始化 HTTP 路由，以便可以安全使用 shared_from_this()
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]() {
                http_route_init();
                http_thread_ = thread([this]() { http_srv_.listen("0.0.0.0", 8080); });
                RCLCPP_INFO(this->get_logger(), "HTTP server started on port 8080");
                init_timer_->cancel();  // 一次性定时器，触发后取消
            });

        TimerTools::SleepForMs(100); // 保证线程均已启动
        srv_ready_ = this->create_service<std_srvs::srv::Empty>(
            "/dev_server/ready",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                RCLCPP_INFO(this->get_logger(), "dev_server ready");
                (void)request;
                (void)response;
            });
        RCLCPP_INFO(this->get_logger(), "dev_server_node init complete");
    }
    ~dev_server_node() {}

private:
    void on_open(const net_client_info &info)
    {
        lock_guard<mutex> lock(mutex_);
        clients_.insert(info);
    }
    void on_close(const net_client_info &info)
    {
        lock_guard<mutex> lock(mutex_);
        clients_.erase(info);
    }

    void on_message(const net_client_info &info, const std::string &data)
    {
        lock_guard<mutex> lock(mutex_);
        RCLCPP_INFO(this->get_logger(), "[WS_FORWARD] Received from role=%s, method=%s, data=%s",
                    info.role.c_str(), info.method.c_str(), data.substr(0, 100).c_str());
        // 只在用户侧和设备侧之间进行通信
        int forward_count = 0;
        if (info.role == "usr")
        {
            for (auto &client : clients_)
            {
                if ((client.role == "dev") && (client.method == info.method))
                {
                    server_.send(client, data);
                    forward_count++;
                    RCLCPP_INFO(this->get_logger(), "[WS_FORWARD] Forwarded usr->dev, method=%s, count=%d",
                                info.method.c_str(), forward_count);
                }
            }
        }
        else if (info.role == "dev")
        {
            for (auto &client : clients_)
            {
                if ((client.role == "usr") && (client.method == info.method))
                {
                    server_.send(client, data);
                    forward_count++;
                    RCLCPP_INFO(this->get_logger(), "[WS_FORWARD] Forwarded dev->usr, method=%s, count=%d",
                                info.method.c_str(), forward_count);
                }
            }
        }
        if (forward_count == 0)
        {
            RCLCPP_WARN(this->get_logger(), "[WS_FORWARD] No matching clients found for role=%s, method=%s",
                        info.role.c_str(), info.method.c_str());
        }
    }

    void on_message(const net_client_info &info, const std::vector<uint8_t> &data)
    {
        lock_guard<mutex> lock(mutex_);
        //  RCLCPP_INFO(this->get_logger(), "dev_server_node recv message bin %s, %s", info.role.c_str(), info.method.c_str());
        // 只在用户侧和设备侧之间进行转发
        if (info.role == "usr")
        {
            for (auto &client : clients_)
            {
                if ((client.role == "dev") && (client.method == info.method))
                {
                    // RCLCPP_INFO(this->get_logger(), "dev_server_node send message bin %s, %s", client.role.c_str(), client.method.c_str());
                    server_.send(client, data);
                }
            }
        }
        else if (info.role == "dev")
        {
            for (auto &client : clients_)
            {
                if ((client.role == "usr") && (client.method == info.method))
                {
                    server_.send(client, data);
                }
            }
        }
    }

    /**
     * @description: 获取静态文件目录路径
     * @return 静态文件目录的绝对路径
     */
    std::string get_static_directory()
    {
        namespace fs = std::filesystem;

        // 尝试 1: 部署环境 - ROS2 share 目录
        try {
            std::string share_dir = ament_index_cpp::get_package_share_directory("dev_server");
            std::string static_path = share_dir + "/static";
            if (fs::exists(static_path)) {
                RCLCPP_INFO(this->get_logger(), "Using static files from: %s", static_path.c_str());
                return static_path;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to get package share directory: %s", e.what());
        }

        // 尝试 2: 开发环境 - 当前目录的 static 文件夹
        std::string dev_path = "static";
        if (fs::exists(dev_path)) {
            RCLCPP_INFO(this->get_logger(), "Using static files from: %s (development mode)", dev_path.c_str());
            return dev_path;
        }

        // 如果都找不到，返回默认值并记录警告
        RCLCPP_ERROR(this->get_logger(), "Static directory not found! Frontend will not be accessible.");
        return "static";  // 回退到默认值
    }

    /**
     * @description: http服务器路由初始化
     * @return {}
     */
    void http_route_init()
    {
        // 初始化 HTTP API 处理器（需要在这里而不是构造函数中调用，因为需要 shared_from_this()）
        http_api_handler_ = std::make_unique<dev_server::HttpApiHandler>(shared_from_this());

        // 初始化版本信息处理器
        version_info_handler_ = std::make_unique<dev_server::VersionInfoHandler>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "VersionInfoHandler initialized");

        // 初始化版本切换处理器
        version_switch_handler_ = std::make_unique<dev_server::VersionSwitchHandler>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "VersionSwitchHandler initialized");

        // 初始化版本清理处理器
        version_cleanup_handler_ = std::make_unique<dev_server::VersionCleanupHandler>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "VersionCleanupHandler initialized");

        // 初始化 OTA 更新服务
        update_service_ = std::make_unique<dev_server::UpdateService>(shared_from_this());
        update_service_->setStatusCallback(
            [this](const std::string& taskId, const json& status) {
                this->broadcastUpdateStatus(taskId, status);
            }
        );
        RCLCPP_INFO(this->get_logger(), "UpdateService initialized");

        // 静态文件服务（Vue 前端）- 必须在其他路由之前设置
        std::string static_dir = get_static_directory();
        http_srv_.set_mount_point("/", static_dir);

        // 更新名称
        http_srv_.Post("/update-name", [this](const httplib::Request &req, httplib::Response &res)
                       { update_parameter(req, res, "name"); });

        // 更新描述
        http_srv_.Post("/update-description", [this](const httplib::Request &req, httplib::Response &res)
                       { update_parameter(req, res, "description"); });

        // 更新序列号
        http_srv_.Post("/update-serial-number", [this](const httplib::Request &req, httplib::Response &res)
                       { update_parameter(req, res, "serialNumber"); });

        // 启动电机初始化
        http_srv_.Post("/start-motor-init", [this](const httplib::Request &req, httplib::Response &res)
                       {
            (void)req;
            start_motor_init(res); });

        // 停止电机初始化
        http_srv_.Post("/stop-motor-init", [this](const httplib::Request &req, httplib::Response &res)
                       {
            (void)req;
            stop_motor_init(res); });

        // API: 获取设备 UUID
        http_srv_.Get("/api/device/uuid", [this](const httplib::Request &, httplib::Response &res)
                      {
            std::string json = http_api_handler_->getDeviceUUIDJson();
            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // API: 获取设备完整信息
        http_srv_.Get("/api/device/info", [this](const httplib::Request &, httplib::Response &res)
                      {
            std::string json = http_api_handler_->getDeviceInfoJson();
            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // ===== 版本信息 API =====

        // API: 获取 ROS2 集群版本
        http_srv_.Get("/api/version/ros2", [this](const httplib::Request &, httplib::Response &res)
                      {
            std::string json = version_info_handler_->getROS2VersionJson();
            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // API: 获取 QR 控制进程版本
        http_srv_.Get("/api/version/qr", [this](const httplib::Request &, httplib::Response &res)
                      {
            std::string json = version_info_handler_->getQRVersionJson();
            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // API: 获取版本历史列表
        http_srv_.Get("/api/version/history", [this](const httplib::Request &, httplib::Response &res)
                      {
            std::string json = version_info_handler_->getVersionHistoryJson();
            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // API: 获取当前激活版本
        http_srv_.Get("/api/version/current", [this](const httplib::Request &, httplib::Response &res)
                      {
            std::string json = version_info_handler_->getCurrentVersionJson();
            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // API: 版本切换
        http_srv_.Post("/api/version/switch", [this](const httplib::Request &req, httplib::Response &res)
                       {
            try {
                auto json_req = nlohmann::json::parse(req.body);
                std::string type = json_req["type"];
                std::string targetVersion = json_req["targetVersion"];

                RCLCPP_INFO(this->get_logger(), "[VERSION_SWITCH] Switching %s to %s",
                           type.c_str(), targetVersion.c_str());

                auto result = version_switch_handler_->switchVersion(type, targetVersion);
                std::string json = version_switch_handler_->getJsonResponse(result);

                res.set_content(json, "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");
            } catch (const std::exception& e) {
                nlohmann::json error_json;
                error_json["success"] = false;
                error_json["message"] = "Request parsing error";
                error_json["errorDetails"] = e.what();
                res.set_content(error_json.dump(), "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");
                res.status = 400;
            } });

        // API: 分析版本清理
        http_srv_.Get("/api/version/cleanup/analyze", [this](const httplib::Request &req, httplib::Response &res)
                      {
            std::string type = req.get_param_value("type");
            int keepCount = req.has_param("keepCount") ? std::stoi(req.get_param_value("keepCount")) : 5;

            RCLCPP_INFO(this->get_logger(), "[VERSION_CLEANUP] Analyzing cleanup: type=%s, keepCount=%d",
                       type.c_str(), keepCount);

            auto analysis = version_cleanup_handler_->analyzeCleanup(type, keepCount);
            std::string json = version_cleanup_handler_->getAnalysisJson(analysis);

            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // API: 执行版本清理
        http_srv_.Delete("/api/version/cleanup", [this](const httplib::Request &req, httplib::Response &res)
                         {
            std::string type = req.get_param_value("type");
            int keepCount = req.has_param("keepCount") ? std::stoi(req.get_param_value("keepCount")) : 5;

            RCLCPP_INFO(this->get_logger(), "[VERSION_CLEANUP] Starting cleanup: type=%s, keepCount=%d",
                       type.c_str(), keepCount);

            auto result = version_cleanup_handler_->cleanupOldVersions(type, keepCount);
            std::string json = version_cleanup_handler_->getJsonResponse(result);

            res.set_content(json, "application/json");
            res.set_header("Access-Control-Allow-Origin", "*"); });

        // API: 删除指定版本
        http_srv_.Delete("/api/version/delete", [this](const httplib::Request &req, httplib::Response &res)
                         {
            try {
                auto json_req = nlohmann::json::parse(req.body);
                std::string type = json_req["type"];
                std::string versionPath = json_req["versionPath"];

                RCLCPP_INFO(this->get_logger(), "[VERSION_CLEANUP] Deleting version: %s",
                           versionPath.c_str());

                auto result = version_cleanup_handler_->deleteVersion(type, versionPath);
                std::string json = version_cleanup_handler_->getJsonResponse(result);

                res.set_content(json, "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");
            } catch (const std::exception& e) {
                nlohmann::json error_json;
                error_json["success"] = false;
                error_json["message"] = "Request parsing error";
                error_json["errorDetails"] = e.what();
                res.set_content(error_json.dump(), "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");
                res.status = 400;
            } });

        // ===== OTA 更新 API =====

        // OTA: 文件上传
        http_srv_.Post("/api/update/upload", [this](const httplib::Request &req, httplib::Response &res)
                       {
            try {
                RCLCPP_INFO(this->get_logger(), "[OTA] Received upload request");
                RCLCPP_DEBUG(this->get_logger(), "[OTA] Checking multipart fields: file=%d, sha256_file=%d, app_type=%d",
                             req.has_file("file"), req.has_file("sha256_file"), req.has_file("app_type"));

                // 检查是否有 multipart 数据
                // 注意：multipart/form-data 中的所有字段（包括文本字段）都存储在 files map 中
                if (!req.has_file("file") || !req.has_file("sha256_file") || !req.has_file("app_type")) {
                    json response = {
                        {"success", false},
                        {"message", "Missing required fields: file, sha256_file, app_type"}
                    };
                    res.set_content(response.dump(), "application/json");
                    res.status = 400;
                    return;
                }

                // 获取文件数据
                auto package_file = req.get_file_value("file");
                auto sha256_file = req.get_file_value("sha256_file");
                // multipart 文本字段的内容存储在 content 字段中
                std::string app_type = req.get_file_value("app_type").content;

                // 获取文件名
                std::string package_filename = package_file.filename;

                RCLCPP_INFO(this->get_logger(), "[OTA] Upload validation: appType=%s, filename=%s",
                           app_type.c_str(), package_filename.c_str());

                // 转换为 vector<uint8_t>
                std::vector<uint8_t> package_data(package_file.content.begin(), package_file.content.end());
                std::vector<uint8_t> sha256_data(sha256_file.content.begin(), sha256_file.content.end());

                // 处理上传（包含文件名验证）
                std::string taskId = update_service_->handleUpload(package_data, sha256_data, app_type, package_filename);

                if (taskId.empty()) {
                    json response = {
                        {"success", false},
                        {"message", "Upload failed: Package filename validation error. ROS2 packages must contain 'ros' or 'ROS', QR packages must contain 'qr' or 'QR'."},
                        {"appType", app_type},
                        {"filename", package_filename}
                    };
                    res.set_content(response.dump(), "application/json");
                    res.status = 400;  // Bad Request
                    return;
                }

                // 返回成功响应
                json response = {
                    {"success", true},
                    {"taskId", taskId},
                    {"message", "Upload successful"},
                    {"appType", app_type}
                };
                res.set_content(response.dump(), "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");

                RCLCPP_INFO(this->get_logger(), "[OTA] Upload successful: taskId=%s, appType=%s",
                           taskId.c_str(), app_type.c_str());

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[OTA] Upload failed: %s", e.what());
                json response = {
                    {"success", false},
                    {"message", std::string("Upload failed: ") + e.what()}
                };
                res.set_content(response.dump(), "application/json");
                res.status = 500;
            } });

        // OTA: 启动更新
        http_srv_.Post("/api/update/start", [this](const httplib::Request &req, httplib::Response &res)
                       {
            try {
                auto body = json::parse(req.body);

                if (!body.contains("taskId") || !body.contains("appType")) {
                    json response = {
                        {"success", false},
                        {"message", "Missing required fields: taskId, appType"}
                    };
                    res.set_content(response.dump(), "application/json");
                    res.status = 400;
                    return;
                }

                std::string taskId = body["taskId"];
                bool daemon = body.value("daemon", true);

                // 启动更新
                bool success = update_service_->startUpdate(taskId, daemon);

                if (!success) {
                    json response = {
                        {"success", false},
                        {"message", "Failed to start update process"}
                    };
                    res.set_content(response.dump(), "application/json");
                    res.status = 500;
                    return;
                }

                // 返回成功响应
                json response = {
                    {"success", true},
                    {"taskId", taskId},
                    {"message", "Update started"},
                    {"statusFile", "/var/run/update_status.json"}
                };
                res.set_content(response.dump(), "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");

                RCLCPP_INFO(this->get_logger(), "[OTA] Update started: taskId=%s", taskId.c_str());

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[OTA] Start update failed: %s", e.what());
                json response = {
                    {"success", false},
                    {"message", std::string("Start update failed: ") + e.what()}
                };
                res.set_content(response.dump(), "application/json");
                res.status = 500;
            } });

        // OTA: 查询状态（支持直接读取状态文件，即使 dev_server 重启也能获取状态）
        http_srv_.Get(R"(/api/update/status/(.+))", [this](const httplib::Request &req, httplib::Response &res)
                      {
            try {
                std::string taskId = req.matches[1];

                // 首先尝试从 UpdateService 获取状态（如果任务还在内存中）
                json status = update_service_->getStatus(taskId);

                // 如果状态为空或出错，尝试直接读取状态文件
                if (status.contains("error")) {
                    std::string statusFile = "/var/run/update_status.json";
                    std::ifstream file(statusFile);
                    if (file.is_open()) {
                        try {
                            file >> status;
                            file.close();
                            
                            // 验证 taskId 是否匹配
                            if (status.contains("taskId") && status["taskId"] == taskId) {
                                RCLCPP_INFO(this->get_logger(), "[OTA] Status loaded from file for taskId=%s", taskId.c_str());
                            } else {
                                status = {{"error", "Task ID mismatch or not found"}};
                            }
                        } catch (const std::exception& e) {
                            status = {{"error", "Failed to parse status file"}};
                        }
                    }
                }

                // 构造响应
                json response = {
                    {"success", !status.contains("error")},
                    {"status", status}
                };
                res.set_content(response.dump(), "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[OTA] Get status failed: %s", e.what());
                json response = {
                    {"success", false},
                    {"message", std::string("Get status failed: ") + e.what()}
                };
                res.set_content(response.dump(), "application/json");
                res.status = 500;
            } });

        // OTA: 查询当前更新状态（直接读取状态文件，用于前端轮询）
        http_srv_.Get("/api/update/current-status", [this](const httplib::Request &req, httplib::Response &res)
                      {
            try {
                std::string statusFile = "/var/run/update_status.json";
                std::ifstream file(statusFile);
                
                if (!file.is_open()) {
                    json response = {
                        {"success", false},
                        {"message", "No active update"},
                        {"statusFile", statusFile}
                    };
                    res.set_content(response.dump(), "application/json");
                    res.set_header("Access-Control-Allow-Origin", "*");
                    return;
                }

                json status;
                try {
                    file >> status;
                    file.close();
                    
                    json response = {
                        {"success", true},
                        {"status", status}
                    };
                    res.set_content(response.dump(), "application/json");
                    res.set_header("Access-Control-Allow-Origin", "*");
                    
                    RCLCPP_DEBUG(this->get_logger(), "[OTA] Current status polled: state=%s",
                                status.contains("state") ? status["state"].get<std::string>().c_str() : "unknown");
                } catch (const std::exception& e) {
                    json response = {
                        {"success", false},
                        {"message", "Failed to parse status file"},
                        {"error", e.what()}
                    };
                    res.set_content(response.dump(), "application/json");
                    res.set_header("Access-Control-Allow-Origin", "*");
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[OTA] Get current status failed: %s", e.what());
                json response = {
                    {"success", false},
                    {"message", std::string("Get current status failed: ") + e.what()}
                };
                res.set_content(response.dump(), "application/json");
                res.status = 500;
            } });

        // OTA: 取消更新
        http_srv_.Post(R"(/api/update/cancel/(.+))", [this](const httplib::Request &req, httplib::Response &res)
                       {
            try {
                std::string taskId = req.matches[1];

                // 取消更新
                bool success = update_service_->cancelUpdate(taskId);

                // 构造响应
                json response = {
                    {"success", success},
                    {"message", success ? "Update cancelled" : "Failed to cancel update"}
                };
                res.set_content(response.dump(), "application/json");
                res.set_header("Access-Control-Allow-Origin", "*");

                if (success) {
                    RCLCPP_INFO(this->get_logger(), "[OTA] Update cancelled: taskId=%s", taskId.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "[OTA] Failed to cancel: taskId=%s", taskId.c_str());
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[OTA] Cancel update failed: %s", e.what());
                json response = {
                    {"success", false},
                    {"message", std::string("Cancel update failed: ") + e.what()}
                };
                res.set_content(response.dump(), "application/json");
                res.status = 500;
            } });

        RCLCPP_INFO(this->get_logger(), "HTTP routes initialized with API endpoints, OTA API, and static file serving");
    }

    // 返回 HTML 页面
    void serve_html(httplib::Response &res)
    {
        std::string html_content = read_file("config/index.html");
        if (html_content.empty())
        {
            res.set_content("Failed to load HTML file", "text/plain");
            return;
        }
        res.set_content(html_content, "text/html");
    }

    // 读取文件内容
    std::string read_file(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            return ""; // 文件打开失败
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    // 更新参数（名称、描述、序列号）
    void update_parameter(const httplib::Request &req, httplib::Response &res, const std::string &param_name)
    {
        auto json = nlohmann::json::parse(req.body);
        std::string value = json[param_name];
        // std::cout << param_name << " 更新为: " << value << std::endl;

        if (param_name == "name")
        {
            // apiDev_->SetRobotName(value);
        }
        else if (param_name == "description")
        {
            // apiDev_->SetRobotDescription(value);
        }
        else if (param_name == "serialNumber")
        {
            // apiDev_->SetRobotSerialNo(value);
        }

        res.set_content(value, "text/plain");
    }

    // 启动电机初始化
    void start_motor_init(httplib::Response &res)
    {
        // apiDev_->SetMotorInit(true);
        // std::cout << "电机初始化已启动，将在下一次重启后生效" << std::endl;
        res.set_content("电机初始化已启动", "text/plain");
    }

    // 停止电机初始化
    void stop_motor_init(httplib::Response &res)
    {
        //  apiDev_->SetMotorInit(false);
        //  std::cout << "电机初始化已停止，将在下一次重启后生效" << std::endl;
        res.set_content("电机初始化已停止", "text/plain");
    }

    /**
     * @brief 广播更新状态到所有 WebSocket 客户端
     * @param taskId 任务ID
     * @param status 状态 JSON 对象
     */
    void broadcastUpdateStatus(const std::string& taskId, const json& status)
    {
        lock_guard<mutex> lock(mutex_);

        // 构造 WebSocket 消息
        json message = {
            {"type", "update_status"},
            {"data", status}
        };

        std::string messageStr = message.dump();

        // 广播给所有 usr 角色的客户端
        int broadcast_count = 0;
        for (auto& client : clients_) {
            if (client.role == "usr") {
                server_.send(client, messageStr);
                broadcast_count++;
            }
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "[OTA] Broadcast update status: taskId=%s, state=%s, clients=%d",
                     taskId.c_str(),
                     status.contains("state") ? status["state"].get<std::string>().c_str() : "unknown",
                     broadcast_count);
    }

private:
    WebsocketServer server_;
    httplib::Server http_srv_;
    thread ws_thread_, http_thread_;
    std::set<net_client_info> clients_;
    std::mutex mutex_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_ready_;
    std::unique_ptr<dev_server::HttpApiHandler> http_api_handler_;
    std::unique_ptr<dev_server::UpdateService> update_service_;
    std::unique_ptr<dev_server::VersionInfoHandler> version_info_handler_;
    std::unique_ptr<dev_server::VersionSwitchHandler> version_switch_handler_;
    std::unique_ptr<dev_server::VersionCleanupHandler> version_cleanup_handler_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dev_server_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
