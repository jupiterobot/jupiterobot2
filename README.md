# jupiterobot2
ROS packages for Jupiter2 / Juno2 

## Install dependencies
```
cd ~/catkin_ws/src
git clone https://github.com/jupiterobot/jupiterobot2.git
cd jupiterobot2/
sudo ./install_pkgs.sh
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
## Yolov5 Configuration
In the home directory
```
git clone -b v6.2 https://github.com/ultralytics/yolov5.git
cd ~/yolov5
pip3 install -r requirements.txt
```
## XFei Lib Configuration
```
roscd jupiterobot2_voice_xf && cd lib/
sudo cp libmsc.so /usr/lib
```
## Build the workspace
```
cd ~/catkin_ws
catkin_make
```
