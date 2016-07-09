# How to build DVO SLAM for Linux Mint Rosa 17.3 (based on Ubunty Trusty 14.04 LTS)

## Install ROS Indigo

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
```

## Checkout DVO SLAM
The DVO SLAM code is clone from jade-devel branch from DVO SLAM repository. 
This version uses catkin to build the code, which makes things much easier. 

The Catkin workspace is organized as follows: 

```
dvo_slam  (Catkin workspace root)
    src 
        dvo_core
            CMakeLists.txt
            src
        dvo_benchmark
            CMakeLists.txt
            src
    README.md
```

where dvo_slam is the root of the Catkin workspace and has 4 packages: dvo_core, dvo_ros, dvo_slam, and dvo_benchmark.

## Dependencies 

### sophus
 
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build
cmake ..
make
sudo make install 
```

### CSparse

```
sudo apt-get install libsuitesparse-dev
```

### g2o 
It is necessary to build g2o *after* installing suitesparse to let g2o detect suitesparse.
The version I use is commit ff647bd.

```
git clone https://github.com/RainerKuemmerle/g2o.git
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_LGPL_SHARED_LIBS:BOOL=OFF
make
sudo make install
```

We have to build g2o into static libraries 
and build DVO SLAM also as static libraries to avoid some weird undefined reference to g2o::csparse_extension. It could be possible to build as shared libraries (by default) but somehow it does not work on my machine.

## Building DVO SLAM
Go back to Catkin workspace's root folder and execute

```
catkin_make
```

to build all packages. You can also build each package separately by 

```
catkin_make --pkg dvo_core
catkin_make --pkg dvo_ros
catkin_make --pkg dvo_slam 
catkin make --pkg dvo_benchmark
```

