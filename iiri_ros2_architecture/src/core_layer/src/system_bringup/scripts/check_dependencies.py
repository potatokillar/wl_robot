#!/usr/bin/env python3
"""
依赖检查脚本
检查启动特定层级所需的所有依赖是否满足
"""

import sys
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def load_config():
    """加载配置文件"""
    try:
        config_path = Path(get_package_share_directory('system_bringup')) / 'config' / 'layer_nodes.yaml'
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"❌ 加载配置文件失败: {e}")
        sys.exit(1)


def check_package_exists(package_name):
    """检查ROS2包是否存在"""
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def check_layer_dependencies(layer_name, config):
    """检查指定层级的所有依赖"""
    if layer_name not in config:
        print(f"❌ 未知的层级: {layer_name}")
        return False

    layer_config = config[layer_name]
    missing_packages = []
    found_packages = []

    print(f"\n🔍 检查 {layer_name} 依赖...")
    print("=" * 60)

    for node_name, node_info in layer_config.items():
        package = node_info['package']
        print(f"  检查包: {package:30s} ", end='')

        if check_package_exists(package):
            print("✅ 已安装")
            found_packages.append(package)
        else:
            print("❌ 缺失")
            missing_packages.append(package)

    print("=" * 60)
    print(f"✅ 已找到: {len(found_packages)} 个包")
    print(f"❌ 缺失:   {len(missing_packages)} 个包")

    if missing_packages:
        print("\n⚠️  缺失的包:")
        for pkg in missing_packages:
            print(f"  - {pkg}")
        return False

    print("\n✅ 所有依赖已满足！")
    return True


def check_all_layers(config):
    """检查所有层级的依赖"""
    layers = ['hardware_layer', 'perception_layer', 'intelligence_layer', 'application_layer']
    results = {}

    print("\n" + "=" * 60)
    print("🔍 IIRI 系统依赖检查")
    print("=" * 60)

    for layer in layers:
        results[layer] = check_layer_dependencies(layer, config)

    print("\n" + "=" * 60)
    print("📊 检查结果汇总")
    print("=" * 60)

    for layer, result in results.items():
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {layer:30s} {status}")

    all_passed = all(results.values())
    print("=" * 60)

    if all_passed:
        print("\n✅ 所有层级依赖检查通过！")
        return 0
    else:
        print("\n❌ 部分层级依赖缺失，请安装缺失的包")
        return 1


def main():
    """主函数"""
    config = load_config()

    if len(sys.argv) > 1:
        # 检查特定层级
        layer_name = sys.argv[1]
        if not check_layer_dependencies(layer_name, config):
            sys.exit(1)
    else:
        # 检查所有层级
        sys.exit(check_all_layers(config))


if __name__ == '__main__':
    main()
