# Localization
Monte Carlo Localization - Based on Particle Filters

### Instalação para o robô simulado

1. Faça o clone do nosso pacote *localization* dentro da pasta $catkin_ws/src/

    $ git clone https://github.com/LCAD-UFES/localization.git

2. Baixe os maps e os logs do link abaixo para executar o localization e coloque as pastas no diretório do localization $catkin_ws/src/localization:

    https://www.dropbox.com/sh/n70dto0qn0ab364/AACDTmBnz4Sz0EgIfhsDeDh_a?dl=0

3.  Faça o clone do nosso pacote *PioneerModel* dentro da pasta $catkin_ws/src/

    $ git clone https://github.com/LCAD-UFES/PioneerModel.git

4. Volte até a pasta do catkin $catkin_ws/ e compile o projeto:

    $ catkin_make

### Instalação para o robô real

1. Repita os passos 1 e 2 acima

2. Para usar no robô real será necessário o Rosaria para controlar o Pioneer, o controle via Joystick, drivers do Sensor Sick LMS.
    Os passos para instalar desses pacotes podem ser encontrados na Wiki do Lcad:
    
    http://www.lcad.inf.ufes.br/wiki/index.php/Constru%C3%A7%C3%A3o_de_um_Mapa_usando_o_Pionner,_JoyStick_e_Sensor_Sick_LMS
    
### Testar com o log

Para testar o programa com log feito no CT-7:

    $ roslaunch log_play_${beam or likelihood}.launch


### Testar o localization com o controle

Abra 6 terminais e em faça em sequência, um comando por terminal(Sugiro usar o Terminator):

    $ roscore

    $ rosrun map_server map_server ${diretório_que_contém_o_mapa_yaml}/map.yaml

    $ roslaunch p3dx_gazebo gazebo.launch

Espere o gazebo ser lançado e:

    $ roslaunch p3dx_description rviz.launch

Pode ser que ocorra algum problema do rviz ou do gazebo no caminho, sem stress, encerre o processo pelo terminal adequado com o comando CTRL+C e tente novamente.

Execute o launch com o localization:

    $ roslaunch localization localization_${beam or likelihood}.launch

Vá até a janela do RVIZ e adicione um mapa que ouça o tópico /map.

Ainda no RVIZ, mude o frame global de base_link para /map

Adicione um PoseArray, também no RVIZ. escolha o tópico /pose_array.

Imediatamente você verá as párticulas espalhadas pelo mapa de forma uniforme e randômica. O algoritmo somente faz o sampling e resampling quando ocorre movimentos, então prossigamos para o último terminal (perdeu a conta?):

    $ rosrun p3dx_controller mover.py
    
Caso queira controlar o robô com o joystick execute os comandos(O js0 define a porta USB onde está conectádo o robô):

    $ rosparam set joy_node/dev "/dev/input/js0"
    $ roslaunch localization joy.launch

Posicione as janelas de forma que você consiga ver o rviz enquanto controla o Pioneer pelo terminal.

O algoritmo parece convergir os samples, inicialmente, para áreas erradas mas com o tempo ele se recupera, passeie com o Pioneer por um tempinho, experimente dar uma volta.

