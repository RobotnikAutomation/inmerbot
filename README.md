# inmerbot_sim

Packages for the simulation of the RB-Vogui

<p align="center">
  <img src="rbvogui_sim/doc/rbvogui_base.png" height="275" />
  <img src="rbvogui_sim/doc/rbvogui_one_arm.png" height="275" />
</p>

## Paquetes

Este paquete contiene: 

### rbvogui_sim

Contiene todos los archivos necesarios para lanzar la simulación completa del modelo del RBVogui dedicado al proyecto INMERBOT

### rbvogui_common

Contiene los archivos necesarios para cargar los modelos del robot en el servidor de parámetros de ROS, además de las llamadas a los controladores.

## Requisitos

- Ubuntu 18.04
- ROS Melodic
- Python 2.7 o superior

## Simulando el RB-Vogui

### 1) Instalación de dependencias:

La simulación se ha testeadp usando Gazebo 11, para facilitar la instalación de los elementos que componen el repositorio usamos la herramienta ```vcstool```:

```bash
sudo apt-get install -y python3-vcstool
```

Instalamos ```catkin_tools``` para compilar el workspace

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

Instalamos ```rqt_joint_trajectory_controller``` para mover el brazo joint a joint y ```moveit_commander``` para moverlo vía script

```bash
sudo apt-get install ros-melodic-rqt-joint-trajectory-controller 
sudo apt-get install ros-melodic-moveit-commander
```

### 2) Crear un nuevo directorio de trabajo y clonar el repositorio:

Creamos un nuevo ws:

```bash
mkdir catkin_ws #Desde la carpeta que se quiera creamos el espacio de trabajo, como estándar catkin_ws
cd catkin_ws
```

Instalamos la versión estable del repositorio. Este repositorio se gestiona por medio de ramas. La rama principal, main, se considera la rama limpia y estable. Para nuevos desarrollos crearemos una nueva rama y realizaremos una pull request a la rama principal cuando hayamos terminado y comprobado los desarrollos.

**Instalamos la versión estable:**

```bash
vcs import --input https://raw.githubusercontent.com/RobotnikAutomation/inmerbot/main/repos/inmerbot.repos
rosdep install --from-paths src --ignore-src -y
``` 

### 3) Instalamos los paquetes propietarios de Robotnik, entre ellos: controladores, robotnik_msgs y rcomponent:


```bash
cd ~/catkin_ws
sudo dpkg -i src/inmerbot/rbvogui_common/libraries/*
```

### 4) Compilamos:

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

### 5) Lanzamos la simulación:

Hay diferentes configuraciones disponibles:

- Vogui con manipulador UR-10
- Varios Voguis sin brazo

### 5.1 RB-Vogui con manipulador UR10

Para lanzar el rbvogui con el manipulador UR deberemos lanzar en terminal la siguiente llamada:
```bash
  roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui robot_xacro:=rbvogui_std_ur10.urdf.xacro launch_arm:=true arm_manufacturer:=ur arm_model:=ur10
```

<p align="center">
  <img src="rbvogui_sim/doc/rbvogui_one_arm.png" height="275" />
</p>

You can play with the arm by using the rqt_joint_trajectory:
```bash
ROS_NAMESPACE=robot rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Or even use moveit to plan trajectories:
```bash
ROS_NAMESPACE=robot roslaunch rbvogui_moveit_ur10 demo.launch
```

<!-- 
### 6) Enjoy!

You can use the topic ```/robot/robotnik_base_control/cmd_vel ``` to control the RB-Vogui robot. -->

### 5.2 Varios Voguis sin brazo

*******************************
** **Zona en construcción** **
*******************************

## 6) Teleoperación

El robot se puede controlar de tres maneras:

- Rviz pad plugin
- Keyboard
- Joystick

### 6.1 Rviz pad plugin

Cuando lanzamos RViZ a través de la simulación, el plugin se carga automáticamente. Lo encontramos en la esquina inferior izquierda de la pantalla de RViZ.

<p align="center">
  <img src="rbvogui_sim/doc/rviz_pad_teleop_plugin.png" height="250" />
</p>

### 6.2 Teclado

Instalamos el nodo para control por teclado

```bash
sudo apt-get update
sudo apt-get install ros-melodic-teleop-twist-keyboard
```

Abrimos una nueva termianl y lanzamos el nodo:

```bash
ROS_NAMESPACE=robot rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

### 6.3 Joystick

El robot puede ser controlado con el mando de la PS4, este es el método habitual de control manual de los robots desarrollados por Robotnik, también disponemos de integración para simulación.

Seguir la guía de instalación: [guía de instalación del pad de robotnik](https://github.com/RobotnikAutomation/robotnik_pad)

Una vez tengamos todo instalado, lanzamos la simulación con ```launch_pad:=true```

Parámetro | Tipo | Descripción | Requisitos
------------ | -------------  | ------------- | -------------
launch_pad | booleano  | Lanza el paquete de robotnik pad | ds4drv installado, ps4 joystick, conexión bluetooth

Por ejemplo:

```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui launch_pad:=true
```

## 7) Mapeado, localización y navegación

Con cualquiera de las anteriores configuraciones del robot podemos usar las siguientes acciones: **Solo añadiendo los siguiente parámetros:**

Param | Tipo | Descripción | Requisitos
------------ | -------------  | ------------- | -------------
run_mapping | Booleano  | Lanza mapeado con gmapping | La localización no puede estar activa
run_localization | Booleano  | Lanzar localización con amcl | El mapeado no puede estar funcionando
map_file | String | Mapa a cargar por la localización | Formato: map_folder/map_name.yaml
run_navigation | Booleano  | Lanza navegación con TEB como planificador local | La localización debe estar corriendo. No es compatible con el mapeado

### 7.1 Quick start

Lanza un rbvogui con un world por defecto y su mapa para la localización y la navegación

```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui robot_xacro:=rbvogui_std.urdf.xacro run_localization:=true run_navigation:=true
```

<p align="center">
  <img src="rbvogui_sim/doc/rbvogui_navigation.png" height="400" />
</p>

### 7.2 Crea un mapa

Lanza rbvogui robot con gmapping:

```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui robot_xacro:=rbvogui_std.urdf.xacro run_mapping:=true rviz_config_file:=rviz/rbvogui_map.rviz
```

Mueve el robot usando el plugin del pad teleop de RViZ (por ejemplo):

<p align="center">
  <img src="rbvogui_sim/doc/rbvogui_mapping.png" height="400" />
</p>

Cuando el mapa esté cargado, abre un terminal y accede al paquete ```rbvogui_localization```

```bash
cd nombre_ws && source devel/setup.bash #Accede al ws, carga el setup.bash
roscd rbvogui_localization && cd maps #Accede a la carpeta maps del rospkg
```

Crea una nueva carpeta con el nombre del mapa. Por ejemplo:

``` bash
mkdir demo_map
cd demo_map
```

Finalmente, guarda eñ mapa dentro de la carpeta

```bash
ROS_NAMESPACE=robot rosrun map_server map_saver -f demo_map
```


### 7.3 Usar un mapa

Para navegar con el rbvogui usando el mapa por defecto:

```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui robot_xacro:=rbvogui_std.urdf.xacro run_localization:=true run_navigation:=true
```

O para cargar tu propio mapa:

```bash
roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui robot_xacro:=rbvogui_std.urdf.xacro run_localization:=true run_navigation:=true map_file:=demo_map/demo_map.yaml
```
## 8) Crear tu propio mundo de Gazebo

Existen varias maneras de crear tu propio mundo en Gazebo. 

***********
ADD CONTENT 
***********