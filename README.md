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
| `weak_turning` | Following com ganho de viragem `0.5`. |
| `strong_turning` | Following com ganho de viragem `2.0`: comportamento anterior e modo por defeito. |

```bash
ros2 launch swarm_follow swarm_launch.py mode:=no_following
ros2 launch swarm_follow swarm_launch.py mode:=weak_turning
ros2 launch swarm_follow swarm_launch.py mode:=strong_turning
```

## Limpar a compilacao

```bash
rm -rf build install log
```
