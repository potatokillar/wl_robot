#!/usr/bin/python3


import json
from operator import ge
import subprocess
import argparse
import os
import time
import platform
import pexpect
import getpass

'''
常量定义
'''
# 远程arm和x64路径
REMOTE_ARM64_DIR = "/build/arm64"
REMOTE_X64_DIR = "/build/x64"
# 远程可执行文件路径，相对工程 
REMOTE_BIN_NAME = "/output/qr"
# 远程配置文件路径
REMOTE_CFG_DIR = "/config/"
# 远程资源文件路径
REMOTE_RESOURCE_DIR = "/resource/"
# 本地调试路径
LOCAL_DEBUG_DIR = "/home/wl/bin/"
# 本地自启路径
LOCAL_AUTORUN_DIR = "/home/wl/autorun/"
# 本地资源文件路径
LOCAL_RESOURCE_DIR = "resource/audio"

# 脚本是否在bin下，脚本只能在package下或者bin下执行
underBin = False

# ssh密码
sshPasswd = ""

'''
SCP 自动交互命令
输入@1，命令行
'''
def SCPshellRun(cmdStr):
    global sshPasswd
    print(cmdStr)
    process = pexpect.spawn(cmdStr, timeout=30)
    expect_list = [
        "yes/no",
        "password:",
        pexpect.EOF,
        pexpect.TIMEOUT,
    ]
    index = process.expect(expect_list)
    # 匹配yes/no
    if index == 0:
        process.sendline("yes")
        expect_list = expect_list[1:]   # 不再匹配yes/no
        index = process.expect(expect_list)

        if index == 0:
            process.sendline(sshPasswd)
            process.interact()
        else:
            print("EOF or TIMEOUT")
            exit()
    elif index == 1:
        process.sendline(sshPasswd)
        index = process.expect(expect_list)
        if index == 1:
            sshPasswd = getpass.getpass("密码错误，请再次输入: ")
            process.sendline(sshPasswd)
            index = process.expect(expect_list)
            if index == 1:
                print("多次输入错误密码，退出")
                exit()

        process.interact()  # 让出控制权

    else:
        print("EOF or TIMEOUT")
        exit()  # 匹配失败退出程序



'''
shell命令运行
输入@1，命令行
'''
def ShellRun(cmdStr):
    print(cmdStr)
    try:
        ret = subprocess.call(cmdStr, shell=True)
        #if (ret != 0):
         #   print("执行命令>>" + cmdStr + "<<失败，请检查")
    except KeyboardInterrupt:
        print("\n用户强制退出") 
        KillRunningBin()


'''
清理自启程序，调试时需要关闭正在运行的自启程序
'''
def KillRunningBin():
    ShellRun("sudo killall qr")

'''
运行机器人调试程序
输入@2，机器人运行配置文件
'''
def RunBin(cfgFile):
    # 若脚本是在bin目录下，则cd到同级package目录
    if isUnderBin == False:
        print("请运行~/bin下的robotrun.py来启动程序")
        return
    print("三秒后运行程序……请稍等")
    if os.path.isfile(LOCAL_DEBUG_DIR + "qr") == False:
        print("文件不存在，请先用python3 robotrun.py -f [配置文件] 进行复制")
        return

    # 杀死旧程序
    KillRunningBin()
    time.sleep(3)
    # 运行目录必须在bin下
    os.chdir(LOCAL_DEBUG_DIR)
    cmdStr = "sudo " + LOCAL_DEBUG_DIR + "qr" + " " + LOCAL_DEBUG_DIR + cfgFile
    ShellRun(cmdStr)
  
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
description: 下载所有的配置文件
param downDir 当前配置文件所在的目录
param nowCfg 当前配置文件名
param serPath 服务器资源路径
return {}
'''
def DownloadAllConfig(downDir, nowCfg, serPath):
    # 读取已下载的配置文件，递归下载依赖的配置文件
    absCfgFile = os.path.abspath(downDir + nowCfg)
    while True:
        # 先查看有没有onnx模型
        onnxFile = GetOnnxPath(absCfgFile)[1:]
        if onnxFile != "":
            onnxPath = os.path.dirname(onnxFile)
            if not os.path.exists(downDir + onnxPath):
                os.makedirs(downDir + onnxPath)
            SCPshellRun("scp " + serPath + onnxFile + " " + downDir + onnxFile[1:])

        # 获取网页文件
        SCPshellRun("scp " + serPath + REMOTE_CFG_DIR + "index.html" + " " + downDir)

        # 再读取子配置文件
        subCfgFile = GetSubCfgFile(absCfgFile)
        if subCfgFile != "":
            SCPshellRun("scp " + serPath + REMOTE_CFG_DIR + subCfgFile + " " + downDir)
            absCfgFile = os.path.abspath(downDir  + subCfgFile)
        else:
            # 获取不到子配置了，就退出循环
            break    

def GetOnnxPath(fullname):
    with open(fullname, 'r') as fd:
        for line in fd.readlines():
            if line.startswith("onnxPath"):
                # 1-提取=后面的字符串。2-去除首尾空格。3-去除"字符。4-去掉开头的点
                newfile = line.split('=')[1].strip().replace('\"','')
                return newfile
    return ""

'''
配置文件解析
输入@1，配置文件名字，含路径
输出@1，服务器ssh路径，例如zyl@192.168.102.121:~/code/qr_wl
'''
def ParseJsonCfg(fullname):
    with open(fullname) as f:
        config = json.load(f)
    
        serCfgPath = config["serverUserName"] + "@" + \
                      config["serverIp"] + ":" + \
                      config["serverProject"]
        return serCfgPath

'''
参数支持
'''
def ArgParse():
    parser = argparse.ArgumentParser(description="四足机器人实体机调试脚本")
    # 可选参数，未写该参数时，默认空
    parser.add_argument("-f", "--filename", help="脚本配置文件名称，用于更新本机的程序和配置信息", default="")

    # 必选参数，配置文件名字。当存在-f时，会首先更新配置文件
    parser.add_argument("config", help="机器人运行配置文件，必须指定，无配置文件不能运行")

    # 互斥参数
    ctrType = parser.add_mutually_exclusive_group()
    # 存在-r时，为true，不存在时，为false
    ctrType.add_argument("-r", "--run", help="运行程序 添加即为运行", action="store_true")
    ctrType.add_argument("-i", "--init", help="程序初始化", action="store_true")
    ctrType.add_argument("-u", "--update", help="将package的资源更新到机器中", action="store_true")

    args = parser.parse_args()
    return args



'''
同步远程服务器内容到本地，保存两者一致
'''
def DownloadPackage(serverCfg):
    if len(serverCfg) == 0:
        return
    print("从服务器下载资源中...")
    # 当执行脚本处于子目录时，此方式可以找到正确的路径
    serPath = ParseJsonCfg(os.path.join(os.getcwd(), os.path.dirname(__file__), args.filename))
    cmdStr = "rsync -av --delete " + serPath + "/build/package ./"
    SCPshellRun(cmdStr)
    pass

'''
复制文件或文件夹到对应目录，若存在则覆盖，若不存在则新建
'''
def CopyFile(src, dst):
    # dst是否存在，若不存在，则新建
    if not os.path.exists(dst):
        os.makedirs(dst)
    
    # 若src是文件则复制文件
    if os.path.isfile(src):
        ShellRun("cp " + src + " " + dst)  
    else:
        # 不能判断是否为文件夹，不支持通配符
        ShellRun("cp -r " + src + " " + dst)

'''
更新package资源到autorun
'''
def UpdateAutorun(tomlFile): 
    # 若脚本是在bin目录下，则cd到同级package目录
    if isUnderBin == True:
        if os.path.exists("./package"):
            os.chdir("./package")
        else:
            print("package目录不存在，无法更新")
            return  
         
    # 备份autorun目录
    if os.path.exists(LOCAL_AUTORUN_DIR):
        CopyFile(LOCAL_AUTORUN_DIR + "*", "/home/wl/autorun_bak")


    # 判断当前系统架构是arm还是x86
    if os.uname().machine == "aarch64":
        CopyFile("./arm64/qr", LOCAL_DEBUG_DIR)
        CopyFile("./arm64/qr", LOCAL_AUTORUN_DIR)
    else:
        CopyFile("./x64/qr", LOCAL_DEBUG_DIR)
        CopyFile("./x64/qr", LOCAL_AUTORUN_DIR)
    
    # 复制其他资源
    CopyFile("./config/" + tomlFile, LOCAL_DEBUG_DIR)
    CopyFile("./config/" + tomlFile, LOCAL_AUTORUN_DIR)

    CopyFile("./resource/*", LOCAL_DEBUG_DIR + "resource/")
    CopyFile("./resource/*", LOCAL_AUTORUN_DIR + "resource/")

    CopyFile("./robotrun.py", LOCAL_DEBUG_DIR)
    CopyFile("./server.json", LOCAL_DEBUG_DIR)

    CopyFile("./autorun/*", LOCAL_AUTORUN_DIR)

    CopyFile("./onnx_model", LOCAL_DEBUG_DIR)
    CopyFile("./onnx_model", LOCAL_AUTORUN_DIR)

    CopyFile("./config/index.html", LOCAL_DEBUG_DIR)
    CopyFile("./config/index.html", LOCAL_AUTORUN_DIR)

    # 根据tomlFile文件，递归的将其父toml文件复制到对应位置
    absCfgFile = os.path.abspath("./config/" + tomlFile)
    while True:
        subCfgFile = GetSubCfgFile(absCfgFile)
        if subCfgFile != "":
            CopyFile("./config/" + subCfgFile, LOCAL_DEBUG_DIR)
            CopyFile("./config/" + subCfgFile, LOCAL_AUTORUN_DIR)
            absCfgFile = os.path.abspath("./config/" + subCfgFile)
        else:
            # 获取不到子配置了，就退出循环
            break    
    
    # 修改qr_start.sh文件，使其启动对应的配置文件
    fileData = ""
    with open(LOCAL_AUTORUN_DIR + "qr_start.sh", "r") as fd:
        fileData = fd.read().replace("00-base.toml", tomlFile)

    with open(LOCAL_AUTORUN_DIR + "qr_start.sh", "w") as fd:
        fd.write(fileData)

    # 修改自启脚本，开启自启功能
    ShellRun("sudo cp /home/wl/autorun/qr.service /lib/systemd/system")
    ShellRun("sudo systemctl enable qr.service")

    # 根据用户输入做处理
    user_input = input("是否将bin目录下的数据库复制到autorun中，请输入 y 或 n: ")
    wait_input = True
    while wait_input:
        if user_input.lower() == 'y':
            ShellRun("sudo cp " + LOCAL_DEBUG_DIR + "customConfig.db " + LOCAL_AUTORUN_DIR)
            wait_input = False
        elif user_input.lower() == 'n':
            wait_input = False
        else:
            user_input = input("无效的输入，请输入 y 或 n: ")

'''
更新package资源到bin目录
'''
def UpdateBin(tomlFile):
    # 若脚本是在bin目录下，则cd到同级package目录
    if isUnderBin == True:
        if os.path.exists("./package"):
            os.chdir("./package")
        else:
            print("package目录不存在，无法更新")
            return
        
    # 判断当前系统架构是arm还是x86
    if os.uname().machine == "aarch64":
        CopyFile("./arm64/qr", LOCAL_DEBUG_DIR)
    else:
        CopyFile("./x64/qr", LOCAL_DEBUG_DIR)
    
    # 复制其他资源
    CopyFile("./config/" + tomlFile, LOCAL_DEBUG_DIR)
    CopyFile("./onnx_model", LOCAL_DEBUG_DIR)

    # 根据tomlFile文件，递归的将其父toml文件复制到对应位置
    absCfgFile = os.path.abspath("./config/" + tomlFile)
    while True:
        subCfgFile = GetSubCfgFile(absCfgFile)
        if subCfgFile != "":
            CopyFile("./config/" + subCfgFile, LOCAL_DEBUG_DIR)
            absCfgFile = os.path.abspath("./config/" + subCfgFile)
        else:
            # 获取不到子配置了，就退出循环
            break    

'''
本文件的路径需要注意，调试的时候，其路径是在~/bin下，同级可能存在目录package。
打包提供给客户时，其路径是任意目录的package下
需要先判断文件路径，不同目录的复制文件路径，略有不同
'''
if __name__ == "__main__":
    args = ArgParse()

    # 获取脚本路径
    myDir = os.path.dirname(os.path.abspath(__file__))
    workDir = os.getcwd()
    if workDir != myDir:
        print("请确保脚本和工作目录一致！")
        print("脚本目录：" + myDir) 
        print("工作目录：" + workDir)
        exit()

    lastDir = os.path.basename(myDir)
    if lastDir == "bin":
        # 在bin目录下，可以执行下载操作
        isUnderBin = True
        if len(args.filename) != 0:
            sshPasswd = getpass.getpass("请务必确保远程主机已经仿真测试过，请输入远程主机的SSH密码: ")
            DownloadPackage(args.filename)
            if args.run == True:
                UpdateBin(args.config)  # 仅更新bin
            elif args.init == True:
                UpdateAutorun(args.config) # 仅更新autorun 
            else:
                print("请参考文件开头注释，输入正确的参数！")
                exit()
        if args.run == True:
            RunBin(args.config)
        elif args.update == True:
            print("-u 参数只能在package目录下使用！")
    elif lastDir == "package":
        isUnderBin = False
        if len(args.filename) != 0:
            print("在bin目录下，才能进行下载操作！(-f 参数)")
            exit()
        elif args.update == True:   # 更新包不需要下载资源
            UpdateAutorun(args.config)
        else:
            print("请参考文件开头注释，输入正确的参数！")
            exit()
    else:
        print("请确保脚本在bin或package目录下执行！")
        exit()


    



