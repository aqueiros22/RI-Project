# RI Project

## Compilar

```bash
source /opt/ros/jazzy/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

## Correr

```bash

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch swarm_follow swarm_launch.py
```

O argumento `mode` seleciona o comportamento dos cinco robôs:

| Modo | Comportamento |
| --- | --- |
| `no_following` | Mantém a velocidade e a curvatura inicial de cada robô (`BASE_MOTION`), sem reagir às deteções da câmara. |
| `weak_turning` | Following com PID angular: `(Kp, Ki, Kd) = (0.5, 0.02, 0.05)`. |
| `strong_turning` | Following com PID angular: `(Kp, Ki, Kd) = (2.0, 0.05, 0.15)`. Modo por defeito. |

```bash
ros2 launch swarm_follow swarm_launch.py mode:=no_following
ros2 launch swarm_follow swarm_launch.py mode:=weak_turning
ros2 launch swarm_follow swarm_launch.py mode:=strong_turning
```

## Limpar a compilacao

```bash
rm -rf build install log
```
