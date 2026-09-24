# CM

双机械臂协同操作代码，包括轨迹规划、轨迹优化、分布式 DNMPC、Gazebo
仿真和实物控制。

## 功能包

- `cm_ball`：Ball 规划、仿真和控制入口。
- `cm_box`：Box 规划、仿真和控制入口。
- `cm_dnmpc`：双臂分布式 DNMPC 与实物参数配置。
- `cm_trajectory_optimization`：协同轨迹优化。

## 快速运行

```bash
source /opt/ros/noetic/setup.bash
source /home/mas-2/3rd_party/CM_ws/devel/setup.bash

roslaunch cm_ball sim.launch
roslaunch cm_box sim.launch
```

规划入口：

```bash
rosrun cm_ball plan.py
rosrun cm_box plan.py
```

所有控制程序支持 `--help` 查看运行参数。
