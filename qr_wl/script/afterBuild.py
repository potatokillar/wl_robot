#!/usr/bin/python3

import os



'''
description: 打包编译后的产物到 iiri-qr 部署目录
return {}
'''
def Package():
    # iiri-qr 部署目录（项目根目录下）
    deploy_dir = "../iiri-qr"

    # 确保 iiri-qr 目录存在（应该已经存在）
    if not os.path.exists(deploy_dir):
        print(f"警告: {deploy_dir} 目录不存在，请先创建")
        return

    # 复制 qr 可执行文件到 iiri-qr 目录
    # x86 版本
    if os.path.exists("../build/x64/output/qr"):
        print("复制 x86 qr 可执行文件到 iiri-qr/")
        os.system(f"cp ../build/x64/output/qr {deploy_dir}/")
        os.system(f"chmod +x {deploy_dir}/qr")

    # ARM 版本
    if os.path.exists("../build/arm64/output/qr"):
        print("复制 ARM qr 可执行文件到 iiri-qr/")
        os.system(f"cp ../build/arm64/output/qr {deploy_dir}/")
        os.system(f"chmod +x {deploy_dir}/qr")

    print(f"✅ 编译产物已复制到 {deploy_dir}/")

    # 同时保留旧的 build/package 目录（兼容性）
    # 在build目录下新建一个package文件夹
    os.makedirs("../build/package", exist_ok=True)

    # 若存在qr，则复制到package目录下
    if os.path.exists("../build/x64/output/qr"):
        os.makedirs("../build/package/x64", exist_ok=True)
        os.system("cp ../build/x64/output/qr ../build/package/x64/")
    if os.path.exists("../build/arm64/output/qr"):
        os.makedirs("../build/package/arm64", exist_ok=True)
        os.system("cp ../build/arm64/output/qr ../build/package/arm64/")

    # 复制resource到package目录下
    os.makedirs("../build/package/resource", exist_ok=True)
    os.system("cp -r ../resource/audio ../build/package/resource/")
    # 复制config到package目录下
    os.system("cp -r ../config ../build/package/")
    # 复制script下的部分文件和文件夹到package目录下
    os.system("cp -r ../script/autorun ../build/package/")
    os.system("cp ../script/robotrun.py ../build/package/")
    os.system("cp ../script/server.json ../build/package/")
    # 复制onnx模型到package目录下
   # os.system("cp -r ../onnx_model ../build/package/")

    pass



if __name__ == "__main__":
    print("编译后处理")
    Package()

    pass