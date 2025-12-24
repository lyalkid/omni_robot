# My Omni Robot - Stage 3

4-колесный омнинаправленный робот с 6-DOF манипулятором, Nav2 навигацией, SLAM и MoveIt2.

## Возможности

- Омни-кинематика + одометрия
- Nav2 автономная навигация
- MoveIt2 (планирование траекторий манипулятора)
- ros2_control контроллеры для базы и руки
- Симуляция в Gazebo (ros_gz)

## Требования

- ROS 2 Jazzy (Ubuntu 24.04)
- Gazebo (ros_gz)

Рекомендуется установить зависимости через rosdep.

## Быстрый старт

```bash
cd ~/ros2_ws/src
git clone git@github.com:lyalkid/omni_robot.git my_omni_robot
cd ~/ros2_ws
rosdep install --from-paths src -i -y
colcon build --packages-select my_omni_robot
source install/setup.bash
```

## Запуски

### 1. RViz (модель без симуляции)
```bash
ros2 launch my_omni_robot display.launch.py
```

### 2. Gazebo + управление базой
```bash
ros2 launch my_omni_robot gazebo.launch.py
```

Телеуправление:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Полная навигация (Nav2)
```bash
ros2 launch my_omni_robot sim_nav2.launch.py
```

В RViz:
1. "2D Pose Estimate" — начальная позиция
2. "Nav2 Goal" — цель

### 4. MoveIt2 (только планирование)
```bash
ros2 launch my_omni_robot moveit.launch.py
```

### 5. Gazebo + MoveIt2
```bash
ros2 launch my_omni_robot sim_moveit.launch.py
```

## Структура проекта

```
my_omni_robot/
├── src/
│   └── kinematics.cpp              # Кинематика + одометрия
├── launch/
│   ├── display.launch.py           # RViz
│   ├── gazebo.launch.py            # Gazebo + управление
│   ├── navigation.launch.py        # Nav2 (ядро)
│   ├── sim_nav2.launch.py          # Gazebo + Nav2
│   ├── moveit.launch.py            # MoveIt2 (планирование)
│   └── sim_moveit.launch.py        # Gazebo + MoveIt2
├── config/
│   ├── controllers.yaml            # ros2_control
│   ├── gz_bridge.yaml              # ros_gz bridge
│   ├── nav2_params.yaml            # Nav2 параметры
│   └── moveit/
│       ├── joint_limits.yaml
│       ├── kinematics.yaml
│       ├── moveit_controllers.yaml
│       ├── my_omni_robot.srdf
│       └── ompl_planning.yaml
├── urdf/                           # xacro-модули робота
├── rviz/
├── worlds/
├── meshes/
└── map/
```

## Основные топики

| Топик | Описание |
|-------|----------|
| `/cmd_vel` | Команды скорости |
| `/odom` | Одометрия |
| `/joint_states` | Состояния суставов |
| `/tf` | Трансформации |
| `/scan` | Лидар |
| `/imu` | IMU |
| `/map` | Карта |
| `/plan` | Глобальный путь |

## Параметры робота

В `src/kinematics.cpp`:
```cpp
#define WHEEL_RADIUS 0.03   // м
#define ROBOT_RADIUS 0.088  // м
```

## Лицензия

MIT
