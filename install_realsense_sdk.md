
# librealsense 

## preparations
```console

export REALSENSE_SOURCE_DIR=$HOME/projects/librealsense/
sudo apt-get install guvcview git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_SOURCE_DIR
mkdir $REALSENSE_SOURCE_DIR/build
cd $REALSENSE_SOURCE_DIR/build

```

## build
```console

export REALSENSE_INSTALL_PREFIX=/opt/realsense
sudo mkdir -p $REALSENSE_INSTALL_PREFIX; 
sudo chown $USER:$USER -R $REALSENSE_INSTALL_PREFIX # not relay needed -> you could run _make_ followed by _sudo make install_
cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=false -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DCMAKE_INSTALL_PREFIX=$REALSENSE_INSTALL_PREFIX
make install

```
## extend the ld search path
```console

sudo sh -c "echo $REALSENSE_INSTALL_PREFIX/lib > /etc/ld.so.conf.d/realsense.conf"
sudo ldconfig

```
## udev rules
```console

cd ~/projects/librealsense/
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger

```
## make cmake config avaliable
```console

echo "export realsense2_DIR=/opt/realsense/lib/cmake/realsense2" >> ~/.bashrc

```
### reboot
```console

reboot

```

# test
```console

/opt/realsense/bin/realsense-viewer

```

# ROS 
```console

export REALSENSE_ROS_WS=$HOME/projects/ros/ws00
sudo apt install ros-noetic-ddynamic-reconfigure
git clone https://github.com/IntelRealSense/realsense-ros.git $REALSENSE_ROS_WS/src/realsense-ros
cd $REALSENSE_ROS_WS
catkin_make

```

# UNISTALL
```console

rm -rf /opt/realsense
sudo rm /etc/ld.so.conf.d/realsense.conf
sudo ldconfig
sudo rm /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && udevadm trigger

```

#### &#8594; remove the line " ```console export realsense2_DIR=/opt/realsense/lib/cmake/realsense2 ```" from ~/.bashrc


## Source : https://answers.ros.org/question/363889/intel-realsens-on-ubuntu-2004-ros-noetic-installation-desription/
