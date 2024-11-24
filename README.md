# PSI3442 - P1: Questão Prática

Resposta da oitava questão da primeira prova da disciplina PSI3442 - Projeto de Sistemas Embarcados, em seu oferecimento do segundo semestre de 2024.

## Dados pessoais

Nome: Gabriel Guidi Pezati

NUSP: 12551578

## Instalação

1. Crie um ROS workspace e adicione o pacote `iris_sim/`. É necessário estar usando o ROS Noetic.
2. Copie a pasta `.gazebo/models/` para o diretório do gazebo.
3. Configure as variáveis de environment com:

```shell
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh
source ~/ros_workspaces/nome_do_workspace/devel/setup.bash
```

## Simulação

### Item a

Nesse item, o drone deve visitar sequencialmente três coordenadas espaciais determinadas por (Xa, Ya, Za), (Xb, Yb, Zb) e (Xc, Yc, Zc) e, em cada uma delas, permanecer por 3 minutos. Depois, ele deve retornar a coordenada de origem.

Para simular, abra um terminal e execute:

```shell
roslaunch iris_sim simulation.launch 
```

Depois, no terminal anterior:

```shell
rosrun iris_sim item_a.py
```

Para mudar as coordenadas, altere a variável `coordinates` em `item_a.py`.

### Item b

Diferentemente do item passado, caso o drone atinja 90% de bateria, ele deve voltar a sua base para carregamento.

O procedimento para simular é o mesmo, mas é necessário usar o arquivo `item_b.py` ao invés de `item_a.py`.

## Resultados

Os objetivos dos dois itens foram atingidos, sendo que eu gravei os dois vídeos na aula no Inova. Os vídeos estão no Google Drive, não nesse repositório.

No primeiro vídeo, eu ainda não tinha configurado esse repositório, então estava rodando o conteúdo do arquivo `item_a.py` em outro que chamei de `scripts.py`. O drone voa para o ponto A, espera um tempo `t` menor que três minutos para evitar esperar 9 minutos para a simulação, mas esse tempo pode ser configurado de acordo, e depois vai para os outros pontos, esperando um tempo `t` neles e retorna ao ponto inicial. No console, printei quando o drone chega nos pontos.

No segundo vídeo, rodei o arquivo `takeoff_land.py`, que tinha o conteúdo de `item_b.py`. No vídeo, o drone chega ao ponto A e sua bateria fica com carga menor que 90%, então ele retorna a base. 
