# RI Project

<descrição curta do projeto>

Simulação em Webots de 5 robôs e-puck (`robot1` … `robot5`) controlados por ROS 2 através do
`webots_ros2_driver`. Cada robô usa só a sua câmara frontal para detetar outro e-puck e segui-lo.

## Ambientes testados

| | A — Linux nativo | B — Windows + WSL2 (recomendado em Windows) |
| --- | --- | --- |
| Sistema operativo | Ubuntu 24.04 `<versão exata, ex.: 24.04.x>` | Windows 11 (build 10.0.26300) + WSL 3.0.3.0 (WSLg 1.0.79) com Ubuntu 26.04.1 LTS |
| ROS 2 | Jazzy `<versão do ros-jazzy-ros-base>` | Lyrical — `ros-lyrical-ros-base 0.13.0-3` |
| Webots | `<versão do Webots, ex.: R2025a>` | R2025a, instalado **no Windows** |
| `webots_ros2` | pacote apt `ros-jazzy-webots-ros2` `<versão>` | código-fonte, commit `329486a` (2025.0.0) + [patch](patches/webots_ros2-lyrical-wsl.patch) |
| Python | `<versão, ex.: 3.12.x>` | 3.14.4 |
| GPU | `<GPU>` | NVIDIA GeForce RTX 3050 Laptop |

O mundo (`src/swarm_follow/worlds/arena.wbt`) está gravado em formato **Webots R2025a**.

## Dependências

Instaladas pelo `rosdep` a partir de `src/swarm_follow/package.xml` (pacotes apt, não pip):

| Dependência | Versão testada (ambiente B) |
| --- | --- |
| `rclpy` | 10.0.11 |
| `ament_cmake` | 2.8.8 |
| `webots_ros2_driver` | 2025.0.0 (commit `329486a`) |
| `python3-numpy` | 2.3.5 |
| `python3-opencv` | 4.10.0 |
| `python3-colcon-common-extensions` | 0.3.0 |
| `python3-rosdep` | 0.27.0 |
| `cmake` / `g++` | 4.2.3 / 15.2.0 |

No ambiente A: `<versões do ambiente A>`.

---

## Instalação — A: Ubuntu 24.04 + ROS 2 Jazzy

1. Instalar o ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
2. Instalar o Webots `<versão do Webots>`: https://cyberbotics.com
3. Clonar e compilar:

   ```bash
   git clone https://github.com/aqueiros22/RI-Project.git
   cd RI-Project
   git checkout <branch: main ou follow>

   source /opt/ros/jazzy/setup.bash
   sudo rosdep init   # só na primeira vez
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y

   colcon build --symlink-install
   source install/setup.bash
   ```

## Instalação — B: Windows + WSL2 (Ubuntu 26.04 + ROS 2 Lyrical)

O ROS corre no WSL e o Webots corre **no Windows** (GPU nativa); comunicam por TCP (porta 1234).
O Webots Linux dentro do WSL também funciona, mas é muito lento (ver [Problemas comuns](#problemas-comuns-e-soluções)).

1. **WSL:** no PowerShell, `wsl --install -d Ubuntu-26.04` `<confirmar o nome da distro>`.
2. **ROS 2 Lyrical** no WSL: https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html
   (`ros-lyrical-ros-base` chega), mais `sudo apt install python3-colcon-common-extensions python3-rosdep python3-dev`.
3. **Webots R2025a para Windows:** https://github.com/cyberbotics/webots/releases/tag/R2025a.
   O instalador por utilizador instala em `C:\Users\<utilizador Windows>\AppData\Local\Programs\Webots`.
4. **Clonar o projeto e o `webots_ros2`** (o Lyrical não tem `webots_ros2_driver` no apt, por isso é
   compilado a partir do código-fonte com um patch):

   ```bash
   git clone https://github.com/aqueiros22/RI-Project.git ~/RI/RI-Project
   cd ~/RI/RI-Project
   git checkout <branch: main ou follow>

   rm -rf src/webots_ros2
   git clone https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
   git -C src/webots_ros2 checkout 329486a
   git -C src/webots_ros2 submodule update --init --recursive
   git -C src/webots_ros2 apply ../../patches/webots_ros2-lyrical-wsl.patch
   ```

5. **Dependências e compilação:**

   ```bash
   source /opt/ros/lyrical/setup.bash
   sudo rosdep init   # só na primeira vez
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install --packages-up-to swarm_follow
   ```

## Variáveis de ambiente

| Variável | Valor | Quando |
| --- | --- | --- |
| `WEBOTS_HOME` | Ambiente A: `<pasta do Webots, ex.: /usr/local/webots>`<br>Ambiente B: `/mnt/c/Users/<utilizador Windows>/AppData/Local/Programs/Webots` | Sempre que o Webots não está na pasta padrão (`/usr/local/webots` ou `C:\Program Files\Webots`). |
| `WEBOTS_HOST_IP` | IP do Windows visto do WSL | Só se a deteção automática falhar (por defeito usa o gateway de `ip route`). |
| `WEBOTS_ROS2_LINUX_WEBOTS` | `1` | Só no ambiente B, para usar um Webots **Linux** dentro do WSL em vez do Windows. |

Se o teu `~/.bashrc` define `WEBOTS_HOME`, confirma que aponta para o Webots certo.

---

## Executar

Ambiente A:

```bash
cd RI-Project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch swarm_follow swarm_launch.py
```

Ambiente B (WSL):

```bash
cd ~/RI/RI-Project
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export WEBOTS_HOME=/mnt/c/Users/<utilizador Windows>/AppData/Local/Programs/Webots
ros2 launch swarm_follow swarm_launch.py
```

O launch abre o Webots com `arena.wbt` e liga um controlador ROS a cada robô. Deve aparecer 5 vezes
`Controller successfully connected to robot in Webots simulation.` Para fechar: `Ctrl+C` no terminal.

### Argumentos do launch

`mode` escolhe o comportamento dos cinco robôs:

| Modo | Comportamento |
| --- | --- |
| `no_following` | Usa o movimento inicial (`BASE_MOTION`) quando não há perigo visual; abranda e vira para longe de robôs próximos, sem os seguir. |
| `weak_turning` | Following com PID angular: `(Kp, Ki, Kd) = (0.5, 0.02, 0.05)`. |
| `strong_turning` | Following com PID angular: `(Kp, Ki, Kd) = (2.0, 0.05, 0.15)`. Modo por defeito. |

`webots_mode` escolhe a velocidade do Webots: `realtime` (defeito), `fast` ou `pause`.

No modo `no_following`, o desvio usa apenas a deteção da imagem atual: começa a
abrandar com elevação visual de 0.045 rad e pede velocidade linear zero a 0.09 rad
(ou se a deteção estiver cortada pela margem da imagem). Vira para o lado oposto
ao robô observado; num encontro exatamente frontal, vira à direita. Não guarda
alvos, direções de desvio, imagens anteriores ou temporizadores. O PID continua
a ser a única memória do controlador nos modos de following. A aceleração dos
motores continua limitada pelo Webots, pelo que a travagem não é instantânea.
Estes limiares precisam de validação na simulação: a câmara frontal não cobre
aproximações por trás e a deteção visual pode falhar, logo não há garantia de
ausência de colisões em todas as situações.

```bash
ros2 launch swarm_follow swarm_launch.py mode:=weak_turning webots_mode:=fast
ros2 launch swarm_follow swarm_launch.py --show-args   # lista os argumentos
```

### Métricas

Um robô extra, `metrics` (supervisor do Webots, em `arena.wbt`), regista cada execução. Lê as posições
reais dos robôs e o tópico `/<robô>/follow_state` que cada robô publica. Nunca envia nada aos robôs, por
isso o controlo continua puramente reativo.

| Argumento | Defeito | Efeito |
| --- | --- | --- |
| `gui` | `true` | `false`: Webots sem janela nem renderização, em modo `fast` (testes mais rápidos). |
| `duration` | `0` | Segundos simulados até gravar as métricas e fechar o Webots. `0` corre até `Ctrl+C`. |
| `metrics_dir` | `metrics` | Pasta dos CSV, relativa à pasta de onde se corre o launch. |
| `run_name` | `<modo>_<data-hora>` | Subpasta desta execução. |

```bash
# com janela, 60 s simulados
ros2 launch swarm_follow swarm_launch.py mode:=strong_turning duration:=60
# sem janela (mais rápido)
ros2 launch swarm_follow swarm_launch.py mode:=weak_turning gui:=false duration:=60
```

Cada execução cria `metrics/<run_name>/` com dois ficheiros:

- `timeseries.csv`: uma linha por robô e por passo, com
  `t, robot, x, y, yaw, following, detected, bearing, angular_height, v, w`.
- `summary.csv`: uma linha por robô e uma linha `all` para o grupo. É reescrito a cada segundo
  simulado, por isso existe mesmo depois de um `Ctrl+C`.

| Coluna do `summary.csv` | Significado |
| --- | --- |
| `following_time_s`, `following_pct` | Tempo a seguir um robô. No modo `no_following` é sempre 0. Na linha `all` é a média dos robôs. |
| `mean_abs_bearing_deg` | Erro angular médio (\|ângulo até ao alvo\|) enquanto segue. |
| `target_losses` | Passagens de "a seguir" para "sem alvo" que duram mais de 0,2 s. Falhas mais curtas são tremido da visão e não contam. |
| `robot_collisions` | Contactos entre robôs (centros a menos de 7,5 cm), contados uma vez por contacto. Na linha `all`, cada contacto conta uma vez. |
| `wall_collisions` | Contactos com as paredes da arena. |

### Recompilar

Só é preciso depois de mudar `package.xml`, `setup.py` ou adicionar ficheiros (com `--symlink-install`,
alterações a `.py`, `.wbt` e `.urdf` existentes são usadas diretamente):

```bash
colcon build --symlink-install --packages-up-to swarm_follow
source install/setup.bash
```

### Limpar a compilação

```bash
rm -rf build install log
```

---

## Problemas comuns e soluções

| Sintoma | Causa | Solução |
| --- | --- | --- |
| `src/webots_ros2` está vazia depois do clone | No branch `main`, `src/webots_ros2` é um submódulo sem `.gitmodules` | Ambiente A: apagar a pasta e usar o pacote apt. Ambiente B: clonar como no passo 4. |
| `Unknown CMake command "ament_target_dependencies"` ao compilar `webots_ros2_driver` | Comando removido no ROS 2 Lyrical | Aplicar `patches/webots_ros2-lyrical-wsl.patch` (passo 4). |
| `No such file or directory: '.../msys64/mingw64/bin/webots.exe'` | No WSL, o `webots_ros2` procura o Webots do Windows em `C:\Program Files\Webots` | Definir `WEBOTS_HOME` com a pasta real do Webots no Windows. |
| `The specified robot is not in the list of robots with <extern> controllers, retrying…` sem nunca ligar | Outro Webots já aberto no Windows ocupa a porta 1234, ou o IP do Windows está errado | Fechar todos os Webots abertos; se continuar, `export WEBOTS_HOST_IP=$(ip route show default \| awk '{print $3}')`. |
| Controladores ficam à espera e o Webots não recebe ligações | Firewall do Windows | Permitir o Webots nas redes privadas e públicas quando o Windows perguntar. |
| Webots dentro do WSL (WSLg) muito lento (~0.12x, mesmo em `fast`) | A GPU passa pela camada D3D12 do WSL | Usar o Webots do Windows (ambiente B). |
| Simulação a ~0.5x do tempo real no ambiente B | Cada robô lê e processa uma imagem 640x320 por passo, transferida do Windows para o WSL | `<decisão pendente: baixar a resolução da câmara em arena.wbt?>` Com 320x160 volta a ~0.96x. |
| `e-puck_bluetooth.dll remote control library initialisation failed` | O e-puck tenta carregar o plugin Bluetooth do robô real | Pode ignorar-se. |
| `SystemError: <method 'is_done' ...>` ao fazer `Ctrl+C` | Encerramento dos controladores no Python 3.14 | Pode ignorar-se. |
| `ModuleNotFoundError: No module named 'cv2'` ou `numpy` | Dependências não instaladas | `rosdep install --from-paths src --ignore-src -r -y` |
| `Package 'swarm_follow' not found` | Workspace não compilado ou não carregado | Compilar e fazer `source install/setup.bash`. |

## Notas sobre `src/webots_ros2` (ambiente B)

- É um clone de https://github.com/cyberbotics/webots_ros2 (commit `329486a`) e não faz parte deste
  repositório. As alterações locais estão em [`patches/webots_ros2-lyrical-wsl.patch`](patches/webots_ros2-lyrical-wsl.patch):
  - `webots_ros2_driver/CMakeLists.txt`: substituto de `ament_target_dependencies` e deteção de Python para o Lyrical.
  - `webots_ros2_driver/webots_ros2_driver/utils.py`: IP do Windows a partir do gateway (com DNS tunneling o
    `/etc/resolv.conf` tem `10.255.255.254`), `WEBOTS_HOST_IP` e `WEBOTS_ROS2_LINUX_WEBOTS`.
- Para atualizar o patch depois de mudar o `webots_ros2`:
  `git -C src/webots_ros2 diff > patches/webots_ros2-lyrical-wsl.patch`
