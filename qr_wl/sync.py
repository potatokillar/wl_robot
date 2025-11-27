#!/usr/bin/python3
'''
脚本使用格式
在本项目根目录中直接运行，不需要登录下载到机器人中。

./sync.py (IP地址) (启动文件)  [行为指令]

脚本参数
IP地址：必选，必须是第一个参数。指的是目标机器人的IP。可以是长IP地址，例如192.168.101.36。也可以是短IP，例如前面的可以缩写成36
注意，短IP的前缀是根据网卡名字提取的，若不可用，请用长IP或者检查网卡名字是否是eth或wlan，并反馈修改脚本NET_CARD_LIST参数。

启动文件：必选，必须是第二个参数，紧跟着脚本。即launch文件，不含launch.py，例如all.launch.py写成all

行为指令，必须存在0~1个。位置从第三个参数开启，行为指令之间的参数顺序可以改变。目前有如下参数
-r/--run：运行程序，添加即为运行
-i/--init：程序初始化，添加即为初始化
以上两个指令互斥。
-u/--update：下载程序


使用示例：
将可执行程序下载到机器人上，仅下载不运行。
./sync.py 192.168.101.36 qr -u

将可执行程序下载到机器人上，下载和运行。
./sync.py 192.168.101.36 qr -r -u

将可执行程序下载到机器人上，并设置为开机启动
./sync.py 192.168.101.36 qr -i

不下载，直接启动，适用于资源package已经存在的情况，若不存在，则报错
./sync.py 192.168.101.36 qr -r

'''

import subprocess
import argparse
import os
import time
import getpass
import socket
import psutil
import paramiko
import pexpect
from scp import SCPClient
import sys
import shutil
import signal


'''
常量定义
'''
# 长IP网卡优先级列表
NET_CARD_LIST = ("eno1","enp5s0", "wlan0")

# docker运行指令
DOCKER_RUN_SHELL="source install/setup.bash && ros2 launch bringup all.launch.py"

'''
全局变量定义
'''
target_arch = ""    # 目标机器人的架构
target_ip = ""      # 目标机器人的IP
target_port = "22"    # 目标端口
target_pwd = ""     # 目标机器人的ssh密码
target_user = "wl"
target_config = ".toml"

'''
自动交互命令
输入@1，命令行
'''
def rsync_shell(cmdStr):
    print(cmdStr)
    process = pexpect.spawn(cmdStr, timeout=120, encoding="utf-8")
    process.logfile = sys.stdout
    expect_list = [
        "yes/no",
        "password:",
        pexpect.EOF,
        pexpect.TIMEOUT,
        r'\[sudo\] password for [^:]+:',    #orin下，每次运行脚本还需要输入sudo提权密码，树莓派无此要求
    ]

    global target_pwd
    err_SSH = 0
    err_sudo = 0
    while True:
        if err_sudo >= 3 or err_sudo >= 3:
            # 密码错误太多
            print("多次输入错误密码，退出")
            exit()

        index = process.expect(expect_list)

        # 匹配yes/no
        if index == 0:
            process.sendline("yes")
        elif index == 1:
            if err_SSH == 0:
                process.sendline(target_pwd)
            elif  err_SSH == 2:
                target_pwd = "123"
                process.sendline(target_pwd)
            else:
                print("ssh密码错误，请重新输入")
                target_pwd = getpass.getpass("请输入ssh密码: ")
                process.sendline(target_pwd)
            err_SSH = err_SSH + 1
        elif index == 2:
            #print("执行完毕")
            break
        elif index == 3:
            print("TIMEOUT")
            exit()  # 匹配失败退出程序
        else:
            # 其他应该的是sudo提权密码了
            if err_sudo == 0:
                # 先用ssh密码尝试一下
                process.sendline(target_pwd)
            else:
                print("本次密码是本机sudo提权密码，由于你的密码和ssh远程密码不一致，请重新输入")
                sudo_pwd = getpass.getpass("请输入sudo提权密码: ")
                process.sendline(sudo_pwd)
            err_sudo = err_sudo + 1
    process.interact()  # 让出控制权



'''
shell命令运行
输入@1，命令行
'''
def shell_run(cmdStr):
    #cmdStr = "sudo " + cmdStr   #由于docker现在是root启动，因此所有命令均得提权
    print(cmdStr)
    try:
        ret = subprocess.call(cmdStr, shell=True)
        #if (ret != 0):
         #   print("执行命令>>" + cmdStr + "<<失败，请检查")
    except KeyboardInterrupt:
        print("\n用户强制退出") 


'''
description: 保存目标IP
param ip_addr
return {}
'''
def save_target_ip(ip_addr):
    global target_ip
    # 如果是完整IP，则直接返回，注意，这里不判断IP合法性
    if len(ip_addr) > 3:
        target_ip = ip_addr
        return

    # 获取所有网卡信息
    interface = psutil.net_if_addrs()

    # 按优先级匹配网卡
    for net_card in NET_CARD_LIST:
        if net_card in interface:
             for addr in interface[net_card]:
                if addr.family == socket.AF_INET:  # IPv4地址
                    ip_parts = addr.address.split('.')
                    if len(ip_parts) == 4: # 确保IP地址格式正确
                        target_ip =  f"{'.'.join(ip_parts[:3])}.{ip_addr}"
                        return
    raise Exception("未匹配到网卡")

'''
description: 保存目标架构
param ssh
return {}
'''  
def save_target_arch(ssh):
    stdin, stdout, stderr = ssh.exec_command('uname -m')
    # 获取命令结果
    global target_arch
    target_arch = stdout.read().decode().strip()
    print("目标机器人架构是:", target_arch)

'''
参数支持
'''
def ArgParse():
    parser = argparse.ArgumentParser(description="机器人同步调试软件")
    # 可选参数，未写该参数时，默认空
    #parser.add_argument("--id", help="domain_id的值，默认0", default="0")
    parser.add_argument("-u", "--update", help="将package的资源更新到机器中", action="store_true")
    parser.add_argument("-p", "--port", help="ssh端口号", default="22")

    # 必选参数，配置文件名字。当存在-f时，会首先更新配置文件
    parser.add_argument("ip_addr", help="目标机器人IP地址，第一个参数，必须指定")
    parser.add_argument("config", help="机器人运行配置文件，第二个参数，必须指定")

    # 互斥参数
    ctrType = parser.add_mutually_exclusive_group()
    # 存在-r时，为true，不存在时，为false
    ctrType.add_argument("-r", "--run", help="运行程序 添加即为运行", action="store_true")
    ctrType.add_argument("-i", "--init", help="程序初始化", action="store_true")
    

    args = parser.parse_args()
    return args

'''
复制文件或文件夹到对应目录，若存在则覆盖，若不存在则新建
'''
def CopyFile(src, dst):
    # dst是否存在，若不存在，则新建
    if not os.path.exists(dst):
        os.makedirs(dst)
    
    # 若src是文件则复制文件
    if os.path.isfile(src):
        shell_run("cp " + src + " " + dst)  
    else:
        # 不能判断是否为文件夹，不支持通配符
        shell_run("cp -r " + src + " " + dst)

'''
程序配置文件解析，根据里面的include字段下载新的文件
只解析一层
输入@1，需要解析的文件名，绝对路径
输出@1，需要额外下载的子配置文件，仅名字
'''
def GetSubCfgFile(fullname):
    with open(fullname, 'r') as fd:
        for line in fd.readlines():
            if line.startswith("include"):
                # 1-提取=后面的字符串。2-去除首尾空格。3-去除"字符
                newfile = line.split('=')[1].strip().replace('\"','')
                return newfile
    return ""

'''
description: 打包bin需要的资源
return {}
'''
def package_bin():
    pack_dir = "./build/packageV2"
    # 在build目录下新建一个package文件夹
    os.makedirs(pack_dir, exist_ok=True)

    if target_arch == "aarch64":
        if os.path.exists("./build/arm64/output/qr"):
            CopyFile("./build/arm64/output/qr", pack_dir)
    else:
        if os.path.exists("./build/x64/output/qr"): 
            CopyFile("./build/x64/output/qr", pack_dir)

    # 根据tomlFile文件，递归的将其父toml文件复制到对应位置
    absCfgFile = target_config
    CopyFile(absCfgFile, pack_dir)
    while True:
        subCfgFile = GetSubCfgFile(absCfgFile)
        if subCfgFile != "":
            CopyFile("./config/" + subCfgFile, pack_dir)
            absCfgFile = os.path.abspath("./config/" + subCfgFile)
        else:
            # 获取不到子配置了，就退出循环
            break     
    
    # AI
    CopyFile("./onnx_model/", pack_dir)


def download_bin(ssh):
    # 复制到目标机器中
    remote_file_path ="/home/wl/bin/"
    cmdStr = "rsync -av --delete " + "-e 'ssh -p " + target_port + "'"
    cmdStr = cmdStr + " ./build/packageV2 " + target_user + "@" + target_ip + ":" + remote_file_path
    rsync_shell(cmdStr)

    # 从目标机器中再复制到对应目录
    ssh.exec_command("cp -r " + remote_file_path + "/packageV2/* " +  remote_file_path)

'''
description: 打包autorun需要的资源
return {}
'''
def package_autorun(ssh):
    pack_dir = "./build/packageV2"
    # 在build目录下新建一个package文件夹
    os.makedirs(pack_dir, exist_ok=True)

    if target_arch == "aarch64":
        if os.path.exists("./build/arm64/output/qr"):
            CopyFile("./build/arm64/output/qr", pack_dir)
    else:
        if os.path.exists("./build/x64/output/qr"): 
            CopyFile("./build/x64/output/qr", pack_dir)

    # 根据tomlFile文件，递归的将其父toml文件复制到对应位置
    absCfgFile = target_config
    CopyFile(absCfgFile, pack_dir)
    while True:
        subCfgFile = GetSubCfgFile(absCfgFile)
        if subCfgFile != "":
            CopyFile("./config/" + subCfgFile, pack_dir)
            absCfgFile = os.path.abspath("./config/" + subCfgFile)
        else:
            # 获取不到子配置了，就退出循环
            break     
    
    # AI
    CopyFile("./onnx_model/", pack_dir)

    # 音频
    CopyFile("./resource", pack_dir)

    # 自启脚本
    CopyFile("./script/autorun/*", pack_dir)


def download_autorun(ssh):
    # 复制到目标机器中
    remote_file_path ="/home/wl/autorun/"
    cmdStr = "rsync -av --delete " + "-e 'ssh -p " + target_port + "'"
    cmdStr = cmdStr + " ./build/packageV2 " + target_user + "@" + target_ip + ":" + remote_file_path
    rsync_shell(cmdStr)

    # 从目标机器中再复制到对应目录
    ssh.exec_command("cp -r " + remote_file_path + "/packageV2/* " +  remote_file_path)

    # 设置自启
    ssh.exec_command("sudo cp /home/wl/autorun/qr.service /lib/systemd/system")
    ssh.exec_command("sudo systemctl enable qr.service")
    ssh.exec_command("sudo systemctl disable iiri-qr.service")



'''
description: 初始化
param ssh
return {}
'''
def init_autorun(ssh):
    package_autorun(ssh)    # 先打包

    # 再根据配置修改启动文件
    config  = os.path.basename(target_config) 

    # 修改qr_start.sh文件，使其启动对应的配置文件
    fileData = ""
    with open("./build/packageV2/qr_start.sh", "r") as fd:
        fileData = fd.read().replace("00-base.toml", config)
    with open("./build/packageV2/qr_start.sh", "w") as fd:
        fd.write(fileData)

    # cmake的install会修改权限，这里改掉
    shell_run("chmod 777 " + "./build/packageV2/qr_start.sh")
    shell_run("chmod 777 " + "./build/packageV2/qr_stop.sh")

    download_autorun(ssh)

    # 不需要修改回来，因为每次都会重新打包，都是新的
    pass


'''
description: 启动程序
return {}
'''
def run_bin(ssh):    
    print("请稍等，关闭旧程序，启动新程序，耗时约3~5秒")
    ssh.exec_command("sudo systemctl stop qr")
    ssh.exec_command("sudo killall qr")
    time.sleep(3)

    config  = os.path.basename(target_config)  # 获取文件名
    # 由于resource的问题，必须在bin下运行
    cmdStr = "cd /home/wl/bin && sudo /home/wl/bin/qr /home/wl/bin/" + config
    
    # 执行命令
    print(cmdStr)

    # 实时输出逻辑
    transport = ssh.get_transport()
    channel = transport.open_session()
    channel.get_pty(width=200)
    channel.exec_command(cmdStr)

      # 信号捕获
    def handle_interrupt(signum, frame):
        # 捕获到 Ctrl+C 信号后，发送中断信号到远程会话
        if channel:
            channel.send("\x03")  # 发送 Ctrl+C 到远程会话
        print("收到ctrl+c信号，发送给远程")
    signal.signal(signal.SIGINT, handle_interrupt)

    while True:
        while channel.recv_ready():
            output = channel.recv(512).decode('utf-8')
            print(output, end='', flush=True)
            
        while channel.recv_stderr_ready():
            error = channel.recv_stderr(512).decode('utf-8')
            print(error, end='', flush=True)
            
        if channel.exit_status_ready():
            channel.close()
            break
        time.sleep(0.1)

    # 获取退出状态码
    exit_status = channel.recv_exit_status()
    print(f"\n命令执行完成，退出码: {exit_status}")

    pass

'''
description: 目标机器人的相关处理
param ssh ssh对象
param args 所有参数
return {}
'''
def target_deal(ssh, args):
    
    # 下载程序
    if args.update:
        print("更新程序包到机器人中...")
        package_bin()
        download_bin(ssh)

    if args.run:
        print("运行程序...")
        run_bin(ssh)

    if args.init:
        print("初始化程序...")
        init_autorun(ssh)

    pass

'''
description: 保存配置文件的绝对路径，支持多种写法
param config
return {}
'''
def save_target_config(config):
    global target_config
    # 写法一，tab写法 ./config/qr-linkV2-3.toml
    if config.startswith("./config/") or config.startswith("config/"):
        target_config = os.path.join(os.getcwd(), config)
    elif config.endswith(".toml"):
        #写法二，直接写文件名 qr-linkV2-3.toml
        target_config = os.path.join(os.getcwd(), "config", config)
    else:
        # 写法3，没有后缀，也没有config前缀，尝试补充
        target_config = os.path.join(os.getcwd(), "config", config + ".toml")

def save_target_port(port):
    global target_port
    target_port = port

'''
本文件的路径需要注意，调试的时候，其路径是在~/bin下，同级可能存在目录package。
打包提供给客户时，其路径是任意目录的package下
需要先判断文件路径，不同目录的复制文件路径，略有不同
'''
if __name__ == "__main__":
    args = ArgParse()

    # 目标IP
    target_port = "10151" # 默认端口，相关树莓派已经需做好端口转发配置
    save_target_ip(args.ip_addr)
    save_target_config(args.config)
    print("目标机器人地址是, ip:", target_ip, " port:", target_port)
    print("目标机器人配置文件是:", target_config)

    # target_pwd = getpass.getpass("请务必确保已经编译，请输入目标机器人的SSH密码: ")
    target_pwd = "123456"

    #创建ssh对象
    ssh = paramiko.SSHClient()
    # 允许连接不在 know_hosts 文件中的主机
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        # 连接服务器
        ssh.connect(hostname=target_ip, port=target_port, username=target_user, password=target_pwd)
        save_target_arch(ssh)

        target_deal(ssh, args)

    except paramiko.AuthenticationException:
        print("Authentication failed, please verify your credentials.")
    except paramiko.SSHException as sshException:
        print(f"Could not establish SSH connection: {sshException}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 关闭连接
        exit()

    

