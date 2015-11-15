# localization
Monte Carlo Localization - Based on Particle Filters

## Instalação

1. Faça o clone do pacote *ocuppacy_grid_utils* do respositório do LCAD dentro da pasta $catkin_ws/src/

> $ git clone https://github.com/LCAD-UFES/occupancy_grid_utils

2. Agora, Faça o clone do nosso pacote *localization* dentro da pasta $catkin_ws/src/

> $ git clone https://github.com/LCAD-UFES/localization.git

3. Volte até a pasta do catkin $catkin_ws/ e compile o projeto:

> $ catkin_make

## Testar com o log

Para testar o programa com log feito no CT-7:

> $ roslaunch log_play_${beam or likelihood}.launch


## Testar o localization com o controle

Abra 6 terminais e em faça em sequência, um comando por terminal(Sugiro usar o Terminator):

> $ roscore

> $ rosrun map_server map_server ${diretório_que_contém_o_mapa_yaml}/map.yaml

> $ roslaunch p3dx_gazebo gazebo.launch

Espere o gazebo ser lançado e:

> $ roslaunch p3dx_description rviz.launch

Pode ser que ocorra algum problema do rviz ou do gazebo no caminho, sem stress, encerre o processo pelo terminal adequado com o comando CTRL+C e tente novamente.

Execute o launch com o localization:

> $ roslaunch localization_${beam or likelihood}.launch

Vá até a janela do RVIZ e adicione um mapa que ouça o tópico /map.

Ainda no RVIZ, mude o frame global de base_link para /map

Adicione um PoseArray, também no RVIZ. escolha o tópico /pose_array. 

Imediatamente você verá as párticulas espalhadas pelo mapa de forma uniforme e randômica. O algoritmo somente faz o sampling e resampling quando ocorre movimentos, então prossigamos para o último terminal (perdeu a conta?):

> $ rosrun p3dx_controller mover.py

Posicione as janelas de forma que você consiga ver o rviz enquanto controla o Pioneer pelo terminal. Caso prefira, use uma outra alternativa como controlar via joystick, daí não é necessário manter o terminal ativado.

O algoritmo parece convergir os samples, inicialmente, para áreas erradas mas com o tempo ele se recupera, passeie com o Pioneer por um tempinho, experimente dar uma volta.

Falta agora refinar o código para obter mais perfomance em algumas áreas, evitar algumas cópias de objetos desnecessárias e melhorar os parâmetros alphas dos modelos de medição e locomoção.
