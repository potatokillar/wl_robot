/*
 * @Author: 周健伟
 * @Date: 2025-04-29
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-04-29
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#ifndef RESOURCE_TOOLS__HPP
#define RESOURCE_TOOLS__HPP

#include <iostream>
#include <filesystem>
#include <sstream>
#include <ostream>
#include <fstream>
#include <iomanip>
#include <optional>

class ResourceTool {
public:
    ResourceTool();
    std::string create_file(const std::string & file_parent_directory, const std::string & file_name, const bool & add_timestamp = true);
    std::string get_file_path(const std::string & file_parent_directory, const std::string & file_name);
    ~ResourceTool() = default;
    
private:
    std::string resource_path_;
};

#endif
