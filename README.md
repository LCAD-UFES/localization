# localization
Monte Carlo Localization - Based on Particle Filters

Para copiar o pojeto, faça o clone do respositório dentro da pasta $catkin_ws/src/

    git clone https://github.com/LCAD-UFES/localization.git

Falta ainda publicar o mapa por conta própria e estabelecer as relações de tf para o mapa.Para testar a implementação, desde já, siga os passos:

Volte até a pasta do catkin $catkin_ws/ e compile o projeto:

    catkin_make localization

Abra 7 terminais e em faça em sequẽncia, um comando por terminal:

    $ roscore
    
    $ rosrun map_server map_server ${diretório_que_contém_o_mapa_yaml}/map.yaml
    
    $ roslaunch p3dx_gazebo gazebo.launch

Espere o gazebo ser lançado e:

    $ rosrun amcl amcl
    
    $ roslaunch p3dx_description rviz.launch

Pode ser que ocorra algum problema do rviz ou do gazebo no caminho, sem stress, encerre o processo pelo terminal adequado com o comando CTRL+C e tente novamente.

Vá até a janela do RVIZ e adicione um mapa que ouça o tópico /map. Bom, estou precisando do amcl (comando acima) somente para visualizar o mapa corretamente. Logo mais estará resolvido.

Mude a frame global de base_link para /map.

Adicione um PoseArray, também no rviz, por padrão ele vai pegar o tópico do amcl padrão do ROS e vai exibir um conjunto de partículas ao redor do Pioneer.

Agora vamos iniciar o nosso nó:

    $ rosrun localization localization_node

Vá até o PoseArray, dentro do RVIZ, e altere o tópico de /particle_cloud para /pose_array. Estamos publicando nesse tópico.

Imediatamente você verá as párticulas espalhadas pelo mapa de forma uniforme e randômica. O algoritmo somente faz o sampling e resampling quando ocorre movimentos, então prossigamos para o último terminal (perdeu a conta?):

    $ rosrun p3dx_controller mover.py
  
Posicione as janelas de forma que você consiga ver o rviz enquanto controla o Pioneer pelo terminal. Caso prefira, use uma outra alternativa como controlar via joystick, daí não é necessário manter o terminal ativado.

O algoritmo parece convergir os samples, inicialmente, para áreas erradas mas com o tempo ele se recupera, passeie com o Pioneer por um tempinho, experimente dar uma volta.

Falta agora refinar o código para obter mais perfomance em algumas áreas, evitar algumas cópias de objetos desnecessárias e melhorar os parâmetros alphas dos modelos de medição e locomoção.
