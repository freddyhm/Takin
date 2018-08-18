# Takin
 Capra-Takin is a ROS-based solution for managing and operating Club Capra's rescue robot.

### Imu-Takin:
le imu n'est présentement pas fonctionnel dans Gazebo dans Takin. Il est fonctionnel uniquement sur l'ordinateur IBEX-1/2 dans Urial sur la branch Master.
La commande est: `~/urial/src/capra_imu roslaunch IMU.launch`

Toutefois, il est possible d'obtenir les données du IMU en invite de commande sur tout les ordis:

D'abord installer le nécessaire

1. sudo apt-get install ros-kinetic-octomap ros-kinetic-serial ros-kinetic-move-base-msgs ros-kinetic-joy

Ensuite

1. cd ~/Urial ou cd ~/
2. git clone https://github.com/dawonn/vectornav.git
3. catkin_make
4. source devel/setup.sh
5. roscore

Autre terminal
1. roslaunch vectornav vectornav.launch

Autre terminal
1. rosrun capra_imu listener.py
