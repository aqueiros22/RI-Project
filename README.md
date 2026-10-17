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



## Limpar a compilacao

```bash
rm -rf build install log
```
