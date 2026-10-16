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

## PID como única memória do controlador

A viragem usa um PID por robô, com os ganhos em `TURNING_PID_GAINS` no ficheiro
`src/swarm_follow/swarm_follow/robot_driver.py`. Os três termos P, I e D estão
ativos nos modos de following. Apenas o PID guarda histórico: erro anterior,
integral e derivada filtrada. Não há histórico de imagens, alvos ou velocidades
comandadas fora do PID. O passo temporal é o passo de simulação do Webots.

A integral está limitada a ±1.0 rad·s e não acumula erro que agrave uma saída
angular saturada pelos limites físicos das rodas, incluindo saturação causada
pela combinação de avanço e viragem. O filtro D tem constante de tempo de 0.10 s.
Ao perder o alvo, o PID é reiniciado; a recuperação começa sem pico derivativo.
O modo `no_following` mantém a velocidade e curvatura inicial, sem usar o PID.
Os ganhos são pontos de partida que precisam de afinação visual no Webots.

A velocidade pedida segue esta regra, sem teto `MAX_SPEED`:

```text
v = max(0, SIZE_GAIN × (TARGET_ANGULAR_HEIGHT / altura_angular_atual − 1))
v = v × max(0, cos(ângulo_do_alvo))
```

A altura angular aparente é uma aproximação visual da distância, não uma
medição em metros. Para o mesmo ângulo do alvo, quanto menor a altura aparente
(mais longe), maior a velocidade pedida. Ao aproximar-se, a velocidade diminui,
chegando a zero na altura angular desejada. Um alvo cortado na margem da imagem
torna a estimativa pouco fiável: nesse caso pede-se avanço zero e apenas viragem.
Sem alvo, pede-se a velocidade e curvatura inicial do robô.

`MAX_ACCELERATION = 0.10`, em `src/swarm_follow/swarm_follow/robot_driver.py`,
define a aceleração/desaceleração tangencial de cada roda em **m/s²**.
O controlador converte-a em rad/s² dividindo pelo raio da roda e configura
`Motor.setAcceleration()`. O Webots faz a transição entre velocidades; o
controlador não guarda velocidades anteriores nem deteções passadas.
O histórico de erro fica exclusivamente dentro do PID. O limite aplica-se aos três modos, incluindo o arranque e a travagem.
Assim, pedir velocidade zero não provoca uma paragem instantânea.

Limitar aceleração implica estado físico ao longo do tempo: esse estado fica
nos motores/simulador, não no algoritmo de following. O limite de cada roda
corresponde a um limite cinemático de 0.10 m/s² para o avanço, sem deslizamento;
não é uma garantia da aceleração real em colisões ou com deslizamento.

Não existe um teto adicional de velocidade no algoritmo de following. Os
motores conservam os seus limites físicos, lidos com `getMaxVelocity()`.
Quando necessário, os comandos das duas rodas são escalados em conjunto para
respeitar esses limites e preservar a curvatura pedida; a velocidade realizável
pode, portanto, saturar mesmo que a velocidade pedida continue a aumentar.
Ver a [documentação do Motor do Webots](https://www.cyberbotics.com/doc/reference/motor).

## Limpar a compilacao

```bash
rm -rf build install log
```
