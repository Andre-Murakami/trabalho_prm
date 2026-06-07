# TRABALHO 1 - SSC0712 Programação de Robôs Móveis

**Disciplina SSC0712**  

## Como utilizar o pacote

### 1. Clonar o repositório

Acesse a pasta `src` do seu workspace ROS 2:

```bash
cd ~/ros2_ws/src/
git clone https://github.com/Andre-Murakami/trabalho_prm.git
````

### 2. Instalar dependências

Instale as dependências do pacote com:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> Certifique-se de ter rodado previamente `sudo rosdep init` e `rosdep update`, se for a primeira vez usando o `rosdep`.

### 3. Compilar o workspace

Certifique-se de estar na **raiz do seu workspace** (geralmente `~/ros2_ws`) antes de compilar:


cd ~/ros2_ws
rm -rf build install log
cd ~/ros2_ws && colcon build --symlink-install --packages-select missao_bandeira && source install/local_setup.bash



### 4. Atualizar o ambiente do terminal


source install/local_setup.bash


## Executando a simulação

### 1. Abra um terminal - Terminal 1:Iniciar o mundo no Gazebo

cd ~/ros2_ws && source install/local_setup.bash
ros2 launch missao_bandeira inicia_simulacao.launch.py

Espera o Gazebo abrir completamente, depois:

### 2. Abra outro terminal - Terminal 2: Carregar o robô no ambiente

cd ~/ros2_ws && source install/local_setup.bash
ros2 launch missao_bandeira carrega_robo.launch.py

Espera o robô aparecer no Gazebo, depois:


### 3. Abra um terceiro terminal: Terminal 3: Programa encontra bandeira azul e para em frente a ela

cd ~/ros2_ws && source install/local_setup.bash
ros2 run missao_bandeira missao_controle


# TRABALHO 1 - SSC0712 Programação de Robôs Móveis - Grupo X
