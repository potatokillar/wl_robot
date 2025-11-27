#!/usr/bin/env python3
"""
Jenkins 任务创建脚本
使用 Python 通过 Jenkins API 创建或更新任务
"""

import sys
import requests
from requests.auth import HTTPBasicAuth
import xml.etree.ElementTree as ET

# Jenkins 配置
JENKINS_URL = "http://192.168.1.93:8080"
JENKINS_USER = "admin"
JENKINS_PASSWORD = "westlake"
JOB_NAME = "iiri-layered-build-ci"
CONFIG_FILE = "job-config.xml"

class Colors:
    BLUE = '\033[0;34m'
    GREEN = '\033[0;32m'
    RED = '\033[0;31m'
    YELLOW = '\033[1;33m'
    NC = '\033[0m'

def print_info(msg):
    print(f"{Colors.BLUE}[INFO]{Colors.NC} {msg}")

def print_success(msg):
    print(f"{Colors.GREEN}[SUCCESS]{Colors.NC} {msg}")

def print_error(msg):
    print(f"{Colors.RED}[ERROR]{Colors.NC} {msg}")

def print_warn(msg):
    print(f"{Colors.YELLOW}[WARN]{Colors.NC} {msg}")

def get_crumb(session):
    """获取 Jenkins Crumb (CSRF token)"""
    try:
        response = session.get(
            f"{JENKINS_URL}/crumbIssuer/api/json",
            timeout=5
        )
        if response.status_code == 200:
            crumb_data = response.json()
            return crumb_data['crumb'], crumb_data['crumbRequestField']
        elif response.status_code == 404:
            print_warn("Jenkins 未启用 CSRF 保护")
            return None, None
        else:
            print_warn(f"无法获取 Crumb: HTTP {response.status_code}")
            return None, None
    except Exception as e:
        print_warn(f"获取 Crumb 失败: {e}")
        return None, None

def job_exists(session, job_name):
    """检查任务是否存在"""
    try:
        response = session.get(
            f"{JENKINS_URL}/job/{job_name}/api/json",
            timeout=5
        )
        return response.status_code == 200
    except Exception as e:
        print_error(f"检查任务失败: {e}")
        return False

def create_job(session, job_name, config_xml, crumb=None, crumb_field=None):
    """创建新任务"""
    headers = {'Content-Type': 'application/xml; charset=utf-8'}
    if crumb and crumb_field:
        headers[crumb_field] = crumb

    try:
        response = session.post(
            f"{JENKINS_URL}/createItem?name={job_name}",
            data=config_xml.encode('utf-8'),
            headers=headers,
            timeout=10
        )
        return response.status_code == 200, response.status_code, response.text
    except Exception as e:
        return False, 0, str(e)

def update_job(session, job_name, config_xml, crumb=None, crumb_field=None):
    """更新现有任务"""
    headers = {'Content-Type': 'application/xml; charset=utf-8'}
    if crumb and crumb_field:
        headers[crumb_field] = crumb

    try:
        response = session.post(
            f"{JENKINS_URL}/job/{job_name}/config.xml",
            data=config_xml.encode('utf-8'),
            headers=headers,
            timeout=10
        )
        return response.status_code == 200, response.status_code, response.text
    except Exception as e:
        return False, 0, str(e)

def main():
    import os

    print_info(f"Jenkins 服务器: {JENKINS_URL}")
    print_info(f"任务名称: {JOB_NAME}")
    print_info(f"配置文件: {CONFIG_FILE}")

    # 支持非交互模式
    force_update = os.environ.get('FORCE_UPDATE', 'false').lower() == 'true'

    # 读取配置文件
    try:
        with open(CONFIG_FILE, 'r') as f:
            config_xml = f.read()
    except FileNotFoundError:
        print_error(f"配置文件不存在: {CONFIG_FILE}")
        sys.exit(1)

    # 创建会话
    session = requests.Session()
    session.auth = HTTPBasicAuth(JENKINS_USER, JENKINS_PASSWORD)

    # 获取 Crumb
    print_info("获取 Jenkins Crumb...")
    crumb, crumb_field = get_crumb(session)
    if crumb:
        print_success("成功获取 Crumb")

    # 检查任务是否存在
    print_info("检查任务是否已存在...")
    if job_exists(session, JOB_NAME):
        print_warn(f"任务 '{JOB_NAME}' 已存在")
        if not force_update:
            response = input("是否要更新现有任务? (y/N): ")
            if response.lower() != 'y':
                print_info("取消操作")
                sys.exit(0)
        else:
            print_info("强制更新模式已启用")

        # 更新任务
        print_info("更新任务配置...")
        success, status, message = update_job(session, JOB_NAME, config_xml, crumb, crumb_field)

        if success:
            print_success("任务更新成功!")
        else:
            print_error(f"任务更新失败! HTTP {status}")
            if message:
                print(f"响应: {message[:200]}")
            sys.exit(1)
    else:
        # 创建新任务
        print_info("创建新任务...")
        success, status, message = create_job(session, JOB_NAME, config_xml, crumb, crumb_field)

        if success:
            print_success("任务创建成功!")
        else:
            print_error(f"任务创建失败! HTTP {status}")
            if message:
                print(f"响应: {message[:200]}")
            sys.exit(1)

    # 显示结果
    print_success("任务已配置完成!")
    print()
    print(f"任务 URL: {JENKINS_URL}/job/{JOB_NAME}")
    print()
    print_info("后续步骤:")
    print("1. 访问 Jenkins 任务页面查看配置")
    print("2. 确保 Git 凭据 'git-cred' 已配置")
    print("3. 点击 'Build with Parameters' 测试构建")
    print("4. 配置邮件或其他通知方式（可选）")
    print()
    print_info("提示: 首次构建可能需要下载 Docker 镜像，耗时较长")

if __name__ == "__main__":
    main()
