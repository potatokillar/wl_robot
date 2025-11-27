#!/bin/bash

echo "=== 诊断 vcstool 安装状态 ==="
echo ""

echo "1. 检查 vcs 命令是否存在:"
if command -v vcs &> /dev/null; then
    echo "   ✓ vcs 命令存在"
    echo "   路径: $(command -v vcs)"
else
    echo "   ✗ vcs 命令不存在"
fi

echo ""
echo "2. 直接运行 vcs --version:"
if vcs --version 2>&1; then
    echo "   ✓ vcs 可以执行"
else
    echo "   ✗ vcs 无法执行，返回码: $?"
fi

echo ""
echo "3. 检查 which vcs:"
which vcs

echo ""
echo "4. 检查 PATH:"
echo "   $PATH"

echo ""
echo "5. 检查 vcstool Python 包:"
python3 -c "import vcstool; print(f'   vcstool 包路径: {vcstool.__file__}')" 2>&1 || echo "   Python 包未安装"

echo ""
echo "=== 诊断完成 ==="
