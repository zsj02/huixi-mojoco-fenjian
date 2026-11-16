# wheel_legged_proxy

项目原创几何代理：四条固定支腿、四个驱动轮、升降/伸缩机构、旋转腕、双指夹爪、RGB 相机、2D 激光、IMU 与里程计。尺寸和动力学为工程占位参数，不是 M20s 数字孪生，也没有主动抬腿、平衡或爬阶能力。

归档 `robot.sdf` / `robot.urdf` 保持原项目基础模型；生成源码为 `src/hx_grid_sim/hx_grid_sim/robot.py`。查看器使用已归档 SDF，不自动切换实验中的优化运动配置。无导航、抓取、开门或测温策略。
