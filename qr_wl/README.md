# qr_wl

## 快速开始（新人必读）

### 首次使用

如果你是第一次从 Git 克隆此项目，请按以下步骤操作：

```bash
# 1. 克隆项目
git clone http://192.168.1.55/ontology/qr_wl.git
cd qr_wl

# 2. 配置环境（一键配置 Docker 和 Harbor）
./setup_environment.sh

# 3. 编译项目
./build_docker.sh        # 使用 Docker 编译（推荐）
# 或
./build.sh               # 传统编译方式

# 4. 编译特定架构
./build_docker.sh x86    # 编译 x86 版本
./build_docker.sh arm    # 编译 ARM 版本
```

**环境配置说明**：
- `setup_environment.sh` 会自动完成以下任务：
  - 检查并配置 Docker
  - 配置 Harbor 不安全仓库（192.168.1.93）
  - 自动登录 Harbor（账号密码已内置）
  - 下载编译所需的 Docker 镜像
  - 验证环境配置

## 介绍
通用机器人控制器

## 使用说明

1.  使用build.sh或者build.sh -c编译代码，-c选项代表清除编译临时文件，首次/调整cmake时建议加上。   
2.  使用start.sh可以直接在开发机上进行调试
3.  将script/robotrun.py存入机器人，参考server.json写好配置文件，可直接download到机器人上进行调试。  


# 参与贡献

