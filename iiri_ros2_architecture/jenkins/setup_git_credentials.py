#!/usr/bin/env python3
"""
Jenkins Git 凭据配置脚本
使用 Jenkins API 创建或更新 Git 凭据
"""

import sys
import requests
from requests.auth import HTTPBasicAuth
import xml.etree.ElementTree as ET

# Jenkins 配置
JENKINS_URL = "http://192.168.1.93:8080"
JENKINS_USER = "admin"
JENKINS_PASSWORD = "westlake"

# Git 凭据配置
GIT_CREDENTIAL_ID = "git-cred"
GIT_USERNAME = "root"
GIT_PASSWORD = "westlake"
GIT_DESCRIPTION = "GitLab credentials for iiri_ros2_architecture"

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
    """获取 Jenkins Crumb"""
    try:
        response = session.get(
            f"{JENKINS_URL}/crumbIssuer/api/json",
            timeout=5
        )
        if response.status_code == 200:
            crumb_data = response.json()
            return crumb_data['crumb'], crumb_data['crumbRequestField']
        else:
            return None, None
    except:
        return None, None

def create_credential_xml():
    """创建凭据 XML 配置"""
    xml_template = f"""<com.cloudbees.plugins.credentials.impl.UsernamePasswordCredentialsImpl>
  <scope>GLOBAL</scope>
  <id>{GIT_CREDENTIAL_ID}</id>
  <description>{GIT_DESCRIPTION}</description>
  <username>{GIT_USERNAME}</username>
  <password>{GIT_PASSWORD}</password>
</com.cloudbees.plugins.credentials.impl.UsernamePasswordCredentialsImpl>"""
    return xml_template

def credential_exists(session, cred_id):
    """检查凭据是否存在"""
    try:
        response = session.get(
            f"{JENKINS_URL}/credentials/store/system/domain/_/credential/{cred_id}/api/json",
            timeout=5
        )
        return response.status_code == 200
    except:
        return False

def create_credential(session, cred_xml, crumb=None, crumb_field=None):
    """创建新凭据"""
    headers = {'Content-Type': 'application/xml; charset=utf-8'}
    if crumb and crumb_field:
        headers[crumb_field] = crumb

    try:
        response = session.post(
            f"{JENKINS_URL}/credentials/store/system/domain/_/createCredentials",
            data=cred_xml.encode('utf-8'),
            headers=headers,
            timeout=10
        )
        return response.status_code == 200 or response.status_code == 302, response.status_code
    except Exception as e:
        return False, str(e)

def update_credential(session, cred_id, cred_xml, crumb=None, crumb_field=None):
    """更新现有凭据"""
    headers = {'Content-Type': 'application/xml; charset=utf-8'}
    if crumb and crumb_field:
        headers[crumb_field] = crumb

    try:
        response = session.post(
            f"{JENKINS_URL}/credentials/store/system/domain/_/credential/{cred_id}/config.xml",
            data=cred_xml.encode('utf-8'),
            headers=headers,
            timeout=10
        )
        return response.status_code == 200, response.status_code
    except Exception as e:
        return False, str(e)

def main():
    print_info(f"Jenkins 服务器: {JENKINS_URL}")
    print_info(f"凭据 ID: {GIT_CREDENTIAL_ID}")
    print_info(f"Git 用户名: {GIT_USERNAME}")

    # 创建会话
    session = requests.Session()
    session.auth = HTTPBasicAuth(JENKINS_USER, JENKINS_PASSWORD)

    # 获取 Crumb
    print_info("获取 Jenkins Crumb...")
    crumb, crumb_field = get_crumb(session)
    if crumb:
        print_success("成功获取 Crumb")

    # 创建凭据 XML
    cred_xml = create_credential_xml()

    # 检查凭据是否存在
    print_info("检查凭据是否已存在...")
    if credential_exists(session, GIT_CREDENTIAL_ID):
        print_warn(f"凭据 '{GIT_CREDENTIAL_ID}' 已存在")
        response = input("是否要更新现有凭据? (y/N): ")
        if response.lower() != 'y':
            print_info("取消操作")
            sys.exit(0)

        # 更新凭据
        print_info("更新凭据...")
        success, status = update_credential(session, GIT_CREDENTIAL_ID, cred_xml, crumb, crumb_field)

        if success:
            print_success("凭据更新成功!")
        else:
            print_error(f"凭据更新失败! 状态: {status}")
            sys.exit(1)
    else:
        # 创建新凭据
        print_info("创建新凭据...")
        success, status = create_credential(session, cred_xml, crumb, crumb_field)

        if success:
            print_success("凭据创建成功!")
        else:
            print_error(f"凭据创建失败! 状态: {status}")
            sys.exit(1)

    print()
    print_success("Git 凭据配置完成!")
    print()
    print_info("凭据信息:")
    print(f"  ID: {GIT_CREDENTIAL_ID}")
    print(f"  用户名: {GIT_USERNAME}")
    print(f"  描述: {GIT_DESCRIPTION}")
    print()
    print_info("此凭据将用于 Jenkins 拉取 GitLab 仓库")

if __name__ == "__main__":
    main()
