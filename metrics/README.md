# Métricas

Cada execução cria uma subpasta com dois ficheiros:

- `summary.csv`: o resumo, com uma linha por robô e uma linha `all` para o grupo;
- `timeseries.csv`: o registo passo a passo de cada robô (`t, robot, x, y, yaw, following, detected, bearing, angular_height, v, w`).

## Correr tudo com um comando

O script corre os 3 modos em vários **cenários**. Cada cenário é um `seed`, isto é, um número que gera
posições e direções iniciais aleatórias. O mesmo seed dá **sempre as mesmas posições**, por isso os
3 modos são comparados nas mesmas condições.

Corre-se da raiz do projeto, com o workspace carregado (`source install/setup.bash`):

```bash
# 5 cenários x 3 modos = 15 simulações, uma de cada vez, sem janela
python3 scripts/run_experiments.py --scenarios 5 --duration 60
# os 3 modos de cada cenário ao mesmo tempo, com as 3 janelas do Webots
python3 scripts/run_experiments.py --scenarios 3 --duration 60 --parallel --gui
```

| Opção | Defeito | Efeito |
| --- | --- | --- |
| `--scenarios N` | `3` | Número de cenários diferentes (seeds 1…N). |
| `--seeds 4 7 …` | | Escolher os seeds exatos em vez de 1…N. `0` usa as posições de `arena.wbt`. |
| `--modes …` | os 3 | Só alguns modos. |
| `--duration S` | `60` | Segundos simulados de cada simulação. |
| `--gui` | desligado | Mostra as janelas do Webots. |
| `--parallel` | desligado | Corre os modos de cada cenário ao mesmo tempo, em portas e domínios ROS diferentes. É mais rápido se o PC aguentar, porque cada Webots processa 5 câmaras. |
| `--name` | `exp_<data-hora>` | Nome da pasta da experiência. |

Os resultados ficam em `metrics/<name>/seed<N>_<modo>/`, com um `seed<N>_<modo>.log` ao lado.
No fim, o script diz quantas simulações chegaram ao fim e quais falharam.

Para uma só simulação:

```bash
# gui:=false para não renderizar a simulação
ros2 launch swarm_follow swarm_launch.py mode:=strong_turning seed:=3 gui:=false duration:=60
ros2 launch swarm_follow swarm_launch.py mode:=weak_turning seed:=3 gui:=false duration:=60
ros2 launch swarm_follow swarm_launch.py mode:=no_following seed:=3 gui:=false duration:=60
```

## De onde vêm os dados

- **Cada robô** publica, a cada passo da simulação (32 ms), o tópico `/<robô>/follow_state`.
  Diz se está a seguir alguém, o ângulo até ao alvo (`bearing`) e as velocidades `v` e `w`.
- **O robô `metrics`** é um supervisor do Webots, invisível.
  Lê esses tópicos e as **posições reais** dos robôs no simulador.
  Só observa: nunca envia nada aos robôs.

## Como cada métrica é calculada

| Coluna | Cálculo |
| --- | --- |
| `seed` | Cenário (posições iniciais). `0` são as posições de `arena.wbt`. |
| `duration_s` | Tempo simulado da execução: número de passos × 32 ms. |
| `following_time_s` | Soma dos passos em que o robô está **a seguir**: vê outro robô e o modo não é `no_following`. |
| `following_pct` | `following_time_s / duration_s × 100`. |
| `mean_abs_bearing_deg` | Média de \|ângulo até ao alvo\|, só nos passos em que segue. 0° quer dizer o alvo bem no centro da câmara. |
| `target_losses` | Conta uma perda quando o robô estava a seguir e deixa de ver o alvo **durante mais de 0,2 s**. Falhas mais curtas são tremidos da visão e não contam. |
| `robot_collisions` | Contacto quando a distância entre os centros de dois robôs fica abaixo de **7,5 cm** (dois raios de e-puck). Cada contacto conta uma vez. Só volta a contar depois de se afastarem mais 5 mm. |
| `wall_collisions` | Contacto quando o centro do robô chega a menos de **3,9 cm** da parede (raio do e-puck + 2 mm), na arena de 3×3 m. Conta uma vez por contacto, como acima. |

Na linha `all`:
- `following_time_s`, `following_pct` e `mean_abs_bearing_deg` são **médias**;
- `target_losses` e `wall_collisions` são **somas**;
- `robot_collisions` conta cada batida **uma vez**: robot1 contra robot2 conta 1, não 2.

O código das contas está em `src/swarm_follow/swarm_follow/metrics.py`.
O supervisor que grava os ficheiros está em `src/swarm_follow/swarm_follow/metrics_supervisor.py`.

## Limitações

- As posições aleatórias ficam a pelo menos 30 cm das paredes e 35 cm umas das outras (`src/swarm_follow/swarm_follow/scenarios.py`).
- As colisões vêm só da distância entre centros, não dos contactos da física do Webots.
S