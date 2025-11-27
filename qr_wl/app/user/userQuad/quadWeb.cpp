#include "quadWeb.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "baseline.hpp"
#include "httplib.h"

using json = nlohmann::json;

class RobotWebServer
{
public:
    RobotWebServer(std::shared_ptr<ApiDevice> apiDev) : apiDev_(apiDev)
    {
        // 初始化路由
        setup_routes();
    }

    void run(int port = 20080)
    {
        // std::cout << "Server is running at http://localhost:" << port << "\n";
        LOG_INFO("Server is running at http://localhost:{}", port);
        svr.listen("0.0.0.0", port);
    }

private:
    httplib::Server svr;
    std::shared_ptr<ApiDevice> apiDev_;

    // 设置路由
    void setup_routes()
    {
        // 根路径
        svr.Get("/", [this](const httplib::Request &, httplib::Response &res) { serve_html(res); });

        // 获取版本信息
        svr.Get("/get-versions", [this](const httplib::Request &, httplib::Response &res) { get_versions(res); });

        // 更新名称
        svr.Post("/update-name", [this](const httplib::Request &req, httplib::Response &res) { update_parameter(req, res, "name"); });

        // 更新描述
        svr.Post("/update-description", [this](const httplib::Request &req, httplib::Response &res) { update_parameter(req, res, "description"); });

        // 更新序列号
        svr.Post("/update-serial-number", [this](const httplib::Request &req, httplib::Response &res) { update_parameter(req, res, "serialNumber"); });

        // 启动电机初始化
        svr.Post("/start-motor-init", [this](const httplib::Request &req, httplib::Response &res) {
            (void)req;
            start_motor_init(res);
        });

        // 停止电机初始化
        svr.Post("/stop-motor-init", [this](const httplib::Request &req, httplib::Response &res) {
            (void)req;
            stop_motor_init(res);
        });
    }

    // 返回 HTML 页面
    void serve_html(httplib::Response &res)
    {
        std::string html_content = read_file("config/index.html");
        if (html_content.empty()) {
            res.set_content("Failed to load HTML file", "text/plain");
            return;
        }
        res.set_content(html_content, "text/html");
    }

    // 获取版本信息
    void get_versions(httplib::Response &res)
    {
        auto info = apiDev_->GetDeviceInfo();
        json versions = {{"softwareVersion", info.version.software},
                         {"hardwareVersion", info.version.hardware},
                         {"mechanicalVersion", info.version.mechanical},
                         {"serialNumber", info.serialNo},
                         {"robotName", info.name},
                         {"robotDescription", apiDev_->GetRobotDescription()}};
        // std::cout << versions.dump() << std::endl;
        res.set_content(versions.dump(), "application/json");
    }

    // 更新参数（名称、描述、序列号）
    void update_parameter(const httplib::Request &req, httplib::Response &res, const std::string &param_name)
    {
        auto json = nlohmann::json::parse(req.body);
        std::string value = json[param_name];
        // std::cout << param_name << " 更新为: " << value << std::endl;

        if (param_name == "name") {
            apiDev_->SetRobotName(value);
        } else if (param_name == "description") {
            apiDev_->SetRobotDescription(value);
        } else if (param_name == "serialNumber") {
            apiDev_->SetRobotSerialNo(value);
        }

        res.set_content(value, "text/plain");
    }

    // 启动电机初始化
    void start_motor_init(httplib::Response &res)
    {
        apiDev_->SetMotorInit(true);
        // std::cout << "电机初始化已启动，将在下一次重启后生效" << std::endl;
        res.set_content("电机初始化已启动", "text/plain");
    }

    // 停止电机初始化
    void stop_motor_init(httplib::Response &res)
    {
        apiDev_->SetMotorInit(false);
        //  std::cout << "电机初始化已停止，将在下一次重启后生效" << std::endl;
        res.set_content("电机初始化已停止", "text/plain");
    }

    // 读取文件内容
    std::string read_file(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            return "";  // 文件打开失败
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }
};

QuadWeb::QuadWeb(std::shared_ptr<ApiQuadruped> apiQuad, std::shared_ptr<ApiDevice> apiDev) : apiQuad_(apiQuad), apiDev_(apiDev) { thread_ = std::thread(&QuadWeb::Run, this); }

void QuadWeb::Run()
{
    RobotWebServer server(apiDev_);
    server.run();
}