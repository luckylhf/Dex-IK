# Dex IK

[![CI](https://github.com/luckylhf/Dex-IK/actions/workflows/ci.yml/badge.svg)](https://github.com/luckylhf/Dex-IK/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Apache%202.0%20%2B%20OpenAtom%20OHL%201.0-green)](LICENSES/README.md)
[![Pinocchio](https://img.shields.io/badge/Pinocchio-3.6.0-orange)](https://github.com/stack-of-tasks/pinocchio)

Dex 双臂逆运动学求解器，使用 Pinocchio 建模、CasADi 自动微分和 IPOPT
非线性优化。求解器固定腿部、腰部和头部关节，只优化左右两条 7 自由度手臂，
目标末端为 URDF 自带的 `left_tcp_link` 和 `right_tcp_link`。

## 平台

正式运行基线：

- Ubuntu 22.04 x86_64
- Python 3.10
- Pinocchio 3.6.0
- CasADi 3.7.2

macOS 支持 Python 开发、测试和本机动态库生成。Ubuntu 生成 `.so`，macOS
生成 `.dylib`；两个平台的动态库不能互换。

## 安装

推荐使用 conda-forge：

```bash
conda env create -f environment.yml
conda activate dex-ik
python -m pip install --no-deps -e .
```

`--no-deps` 用于保留 `environment.yml` 中经过约束的 Pinocchio、CasADi 和
NumPy 版本，避免 pip 重装同名依赖。

如果 Python 导入到了 PyPI 上无机器人学功能的 `pinocchio` 0.1 包，请先卸载
该包，并使用 `environment.yml` 安装 conda-forge 的 Pinocchio。

如果系统中曾通过 pip/cmeel 安装过旧版 Pinocchio（路径形如
`~/.local/lib/python3.X/site-packages/cmeel.prefix/.../pinocchio/`），
会导致 conda 环境中的 Pinocchio 3.6.0 被覆盖。此时需要设置环境变量
`PYTHONNOUSERSITE=1` 跳过用户级 site-packages，或手动删除 `.local` 下的
`cmeel.prefix/lib/python3.X/site-packages/pinocchio*`。

## 常见问题

### 导入时报错 "extension class wrapper for base class coal::CollisionObject has not been created yet"

通常是 `.local` 下的旧版 Pinocchio 阴影包与 conda 环境中的 coal/eigenpy
版本不兼容所致。设置 `PYTHONNOUSERSITE=1` 或参考上方说明清理即可。

## Python 使用

```python
import numpy as np
from dex_ik import DexArmIK

ik = DexArmIK(jit=True)

left_target = np.eye(4)
right_target = np.eye(4)
left_target[:3, 3] = [0.30, 0.25, 0.10]
right_target[:3, 3] = [0.30, -0.25, 0.10]

q = ik.solve_ik(
    left_target,
    right_target,
    q_init=np.zeros(14),
    verbose=True,
)
errors = ik.pose_errors(q, left_target, right_target)
```

目标位姿均为相对于 URDF 根坐标系 `pelvis` 的 4×4 齐次变换矩阵。对于不可达
目标，`solve_ik` 返回限定迭代次数内得到的最优结果；调用方应使用
`pose_errors` 检查误差。

完整的可达目标示例：

```bash
python examples/solve_dual_arm.py
```

## 关节输出顺序

返回向量包含 14 个关节，顺序为：

```text
shoulder_pitch_l_joint
shoulder_roll_l_joint
shoulder_yaw_l_joint
elbow_pitch_l_joint
elbow_yaw_l_joint
wrist_pitch_l_joint
wrist_roll_l_joint
shoulder_pitch_r_joint
shoulder_roll_r_joint
shoulder_yaw_r_joint
elbow_pitch_r_joint
elbow_yaw_r_joint
wrist_pitch_r_joint
wrist_roll_r_joint
```

也可以通过 `ik.joint_names` 在运行时读取顺序，或使用
`get_arm_joint_positions(q, "left")` 和
`get_arm_joint_positions(q, "right")` 分别提取单臂结果。

## 生成原生动态库

激活 conda 环境后运行：

```bash
dex-ik-build-solver
```

默认输出：

- Ubuntu：`.casadi_cache/dex_ik_solver.so`
- macOS：`.casadi_cache/dex_ik_solver.dylib`

指定输出路径：

```bash
dex-ik-build-solver --output /path/to/dex_ik_solver.so
```

加载已生成的求解器：

```python
from dex_ik import DexArmIK

ik = DexArmIK(shared_library=".casadi_cache/dex_ik_solver.so")
```

动态库必须在目标操作系统和目标 conda 环境中生成。

## 测试

运行模型资源和 Python IK 测试：

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

额外生成、加载并调用原生动态库：

```bash
DEX_IK_TEST_SHARED_LIBRARY=1 \
PYTHONPATH=src python -m unittest tests.test_solver -v
```

也可以一键验证三个回归姿态的 Python IK、原生库生成与原生 IK：

```bash
python scripts/validate_native.py
```

Ubuntu 与 macOS 的完整验收步骤见
[`UBUNTU_MACOS_VALIDATION.md`](UBUNTU_MACOS_VALIDATION.md)。

## 项目结构

```text
src/dex_ik/solver.py                       IK 实现
src/dex_ik/generate_solver.py              动态库生成入口
src/dex_ik/assets/dex_description/         Dex URDF 与 Mesh
examples/solve_dual_arm.py                 双臂示例
scripts/validate_native.py                 Python 与原生库一键验收
tests/                                     回归测试
LICENSES/                                  许可证文本与适用范围
environment.yml                            conda-forge 环境
```

## 模型来源与许可证

Dex 模型派生自
[Open-X-Humanoid/TienKung_URDF](https://github.com/Open-X-Humanoid/TienKung_URDF/tree/main/tiangong2dex_urdf)，
固定到上游提交 `5c221783fb92fcc4af891ef1dc0502963caf2266`。重命名和删减说明见
[`NOTICE`](NOTICE)。

- IK 代码：Apache License 2.0，见
  [`LICENSES/Apache-2.0.txt`](LICENSES/Apache-2.0.txt)。
- 派生 URDF 与 Mesh：OpenAtom Open Hardware License 1.0，见
  [`LICENSES/OpenAtom-Open-Hardware-License-1.0.txt`](LICENSES/OpenAtom-Open-Hardware-License-1.0.txt)。
- 完整许可证范围见 [`LICENSES/README.md`](LICENSES/README.md)。

求解器不执行自碰撞或环境碰撞检测。
