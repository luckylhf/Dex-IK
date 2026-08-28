# Dex IK：Ubuntu 与 macOS 验收指南

本文用于验证 Dex IK 的安装、Python IK 精度以及原生动态库的生成和加载。
仓库 CI 工作流配置为在 Ubuntu 22.04 上覆盖 Python 3.10–3.13，并在
Python 3.10 上验证原生动态库。macOS 支持按本文步骤进行本机验收，但生成的
动态库不能跨操作系统或 CPU 架构使用。

Dex IK 不是 ROS 2 包，不需要 colcon、RViz 或 ROS 节点即可完成验收。

## 1. 通过标准

全部满足以下条件才算通过：

- Python 包可以正常导入。
- URDF 可以构建为仅包含双臂的 14 自由度模型。
- 三个回归姿态的最大位置误差不超过 0.5 mm。
- 三个回归姿态的最大姿态误差不超过 0.0005 rad。
- Ubuntu 能生成并加载 `.so`，或 macOS 能生成并加载 `.dylib`。
- 原生求解器达到与 Python 求解器相同的精度阈值。

## 2. 安装编译工具

Ubuntu 22.04：

```bash
sudo apt update
sudo apt install -y build-essential
```

macOS：

```bash
xcode-select --install
```

如果 macOS 提示 Command Line Tools 已安装，可以直接继续。

## 3. 创建环境

在包含 `pyproject.toml` 和 `environment.yml` 的仓库根目录运行：

```bash
conda env create --solver libmamba -f environment.yml
conda activate dex-ik
python -m pip install --no-deps -e .
```

确认关键依赖以及 Pinocchio CasADi 绑定：

```bash
python --version
python -c "import casadi; print('CasADi', casadi.__version__)"
python -c "import pinocchio; print('Pinocchio', pinocchio.__version__)"
python -c "from pinocchio import casadi; print('Pinocchio CasADi binding OK')"
```

四条命令必须全部成功。如果最后一条失败，不应继续 IK 或原生库验收。

## 4. 运行源码回归测试

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

资源、许可证范围和 Python IK 测试应通过。原生库测试默认显示 `skipped`，因为
这一步不编译动态库。

## 5. 一键生成并验证原生库

```bash
python scripts/validate_native.py
```

脚本会：

1. 用三个已知关节姿态通过正运动学生成可达目标。
2. 验证 Python IK 的精度并输出耗时。
3. 生成当前平台的原生动态库。
4. 创建新求解器实例并加载动态库。
5. 验证原生 IK 的精度并输出耗时。

默认输出为：

- Ubuntu：`.casadi_cache/dex_ik_solver.so`
- macOS：`.casadi_cache/dex_ik_solver.dylib`

指定输出路径：

```bash
python scripts/validate_native.py --output /path/to/dex_ik_solver.so
```

macOS 的指定路径必须使用 `.dylib` 后缀。

全部通过时，最后应输出：

```text
PASS: all Python and native Dex IK checks passed.
```

## 6. 使用单元测试验证原生库

也可以通过已有的可选测试生成、加载并验证动态库：

```bash
DEX_IK_TEST_SHARED_LIBRARY=1 \
PYTHONPATH=src \
python -m unittest tests.test_solver -v
```

## 7. 检查动态库

Ubuntu：

```bash
file .casadi_cache/dex_ik_solver.so
ldd .casadi_cache/dex_ik_solver.so
```

macOS：

```bash
file .casadi_cache/dex_ik_solver.dylib
otool -L .casadi_cache/dex_ik_solver.dylib
```

输出中不应出现依赖 `not found` 或 CPU 架构不匹配。

## 8. 常见问题

### Pinocchio 没有 CasADi 绑定

如果无法执行 `from pinocchio import casadi`，请使用 `environment.yml` 从
conda-forge 安装，不要使用 PyPI 上无机器人学功能的同名 `pinocchio` 包。

### 用户级包覆盖 Conda 环境

如果用户目录中的旧版 Pinocchio、coal 或 eigenpy 覆盖了 Conda 环境，可先运行：

```bash
export PYTHONNOUSERSITE=1
```

然后重新执行依赖检查和测试。

### 动态库无法加载

确认生成和加载动态库时使用同一个 Conda 环境：

```bash
echo "$CONDA_PREFIX"
which python
```

修改操作系统、CPU 架构、CasADi、IPOPT 或编译器环境后，应重新生成动态库。

## 9. 验收记录

```text
[ ] Conda 环境创建成功
[ ] Pinocchio CasADi 绑定可以导入
[ ] 源码回归测试通过
[ ] Python IK 三个姿态通过精度阈值
[ ] 原生动态库生成并加载成功
[ ] 原生 IK 三个姿态通过精度阈值
[ ] 最大位置误差 <= 0.5 mm
[ ] 最大姿态误差 <= 0.0005 rad
```

验收失败时，应保存完整输出，并同时记录：

```bash
python --version
python -c "import casadi; print(casadi.__version__)"
python -c "import pinocchio; print(pinocchio.__version__)"
echo "$CONDA_PREFIX"
```
