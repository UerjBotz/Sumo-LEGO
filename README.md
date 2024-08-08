# Sumo LEGO
### Escrito por Argos A. Maia

1. *Constantes*
   - **EDGE_THRESHOLD**:
   - **ENEMY_DISTANCE**: checa a distância do meu robô contra o robô adversário
   - **SPEED_RUN**: velocidade de combate
   - **SPEED_SEARCH**: velocidade de giro

2. *checarBordas()*:
   Verifica se o robô está próximo da borda da arena, se checar a borda branca, o robô para e vira a ré.
   - Só funciona se estiver com o **sensorColor** ligado

3. *Motores*
  ~~~python
  motor_L = Motor(Port.B, Direction.CLOCKWISE)#Esquerda
  motor_R = Motor(Port.A, Direction.CLOCKWISE)#Direita
  motor_tracao = Motor(Port.C, Direction.COUNTERCLOCKWISE)
  ~~~~
 A roda de tração deve ser invertida para o funcionamento do código

 