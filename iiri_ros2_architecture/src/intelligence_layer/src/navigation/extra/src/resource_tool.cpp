/*
 * @Author: 周健伟
 * @Date: 2025-04-29
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-06-15
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "extra/resource_tool.hpp"

ResourceTool::ResourceTool() {
    resource_path_ = std::filesystem::current_path().string() + "/" + "resource";
};

std::string ResourceTool::create_file(const std::string & file_parent_directory, const std::string & file_name, const bool & add_timestamp) {
    std::string file_path;
    file_path.clear();
    if (file_name.empty()) {
        std::cout << "Input file name is empty ! please input the correct file name !" << std::endl;
        return file_path;
    };
    if(add_timestamp) {
        // 获取时间戳
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        tm now_tm = *std::localtime(&now_time_t);
        // 文件名添加时间戳
        std::stringstream ss(file_name);
        std::string file_prefix, file_postfix;
        std::getline(ss, file_prefix, '.');
        std::getline(ss, file_postfix);
        if (ss.fail()) {
            std::cout << "Input file name is illegal! Please input correct file name with postfix!" << std::endl;
            return file_path;
        }
        ss.clear();
        ss << file_prefix << std::put_time(&now_tm, "-%F-%T") << "." << file_postfix;
        std::string file_name_stamped = ss.str();
        // 获取文件完整路径
        std::string file_parent_dir_path =  resource_path_ + "/" + file_parent_directory;
        std::filesystem::create_directories(file_parent_dir_path);
        std::string file_path = file_parent_dir_path + "/" + file_name_stamped;
        return file_path;
    } else {
        std::string file_parent_dir_path =  resource_path_ + "/" + file_parent_directory;
        std::filesystem::create_directories(file_parent_dir_path);
        std::string file_path = file_parent_dir_path + "/" + file_name;
        return file_path;        
    }
}

std::string ResourceTool::get_file_path(const std::string & file_parent_directory, const std::string & file_name) {
    std::string file_path = resource_path_ + "/" + file_parent_directory + "/" + file_name;
    if (!std::filesystem::exists(file_path)) {
        file_path.clear();
        return file_path;
    }
    return file_path;
}
