# Métricas

Cada execução cria uma subpasta `<modo>_<data-hora>/` com dois ficheiros:

- `summary.csv`: o resumo, com uma linha por robô e uma linha `all` para o grupo;
- `timeseries.csv`: o registo passo a passo de cada robô (`t, robot, x, y, yaw, following, detected, bearing, angular_height, v, w`).


```bash
# false para não renderizar a simulação
ros2 launch swarm_follow swarm_launch.py mode:=strong_turning gui:=false duration:=60 
ros2 launch swarm_follow swarm_launch.py mode:=weak_turning gui:=false duration:=60 
ros2 launch swarm_follow swarm_launch.py mode:=no_turning gui:=false duration:=60
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

- As posições iniciais são sempre as de `arena.wbt`. Para comparar com outras posições é preciso mudar o mundo.
- As colisões vêm só da distância entre centros, não dos contactos da física do Webots.
