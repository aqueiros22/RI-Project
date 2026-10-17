# RI Project


## Compilar (Ubuntu 24.04 + ROS 2 Jazzy)

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


# WSL do Lucas (Ubuntu 26.04 + ROS 2 Lyrical + Webots R2025a em `~/webots`)

## Abrir a simulação

```bash
cd ~/RI/RI-Project
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export WEBOTS_ROS2_LINUX_WEBOTS=1
ros2 launch swarm_follow swarm_launch.py
```

A janela do Webots abre via WSLg. `Ctrl+C` no terminal para fechar.

## Recompilar (só depois de mudar código em `src/`)

```bash
cd ~/RI/RI-Project
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to swarm_follow
source install/setup.bash
```

## Notas

- `src/webots_ros2` é um clone local de https://github.com/cyberbotics/webots_ros2 (commit `329486a`)
  com duas alterações locais para funcionar no Lyrical/WSL (não vão para o git deste repo):
  - `webots_ros2_driver/CMakeLists.txt`: substituto para `ament_target_dependencies` (removido no Lyrical).
  - `webots_ros2_driver/webots_ros2_driver/utils.py`: a variável `WEBOTS_ROS2_LINUX_WEBOTS=1` faz usar o
    Webots Linux em vez do `webots.exe` do Windows.
- Se apagares `src/webots_ros2`, tens de voltar a clonar e reaplicar estas alterações.
