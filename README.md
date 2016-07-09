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
Recent g2o starts using C++11, which causes trouble when DVO links to the default PCL provided by Ubuntu. This PCL build does not have C++11 support. 

It is best to use an old version of g2o instead, which does not require C++11, so we don't need to rebuild PCL with C++11. The suitable g2o version is commit 67d5fa7.
(To speed up the g2o compilation, you can skip building g2o apps and examples, which are not necessary in our case.)

```
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 67d5fa7
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

## How to run

```
source devel/setup.sh  
```

This will make ROS able to find our newly built packages. 

Now we can navigate to src/dvo_benchmark/launch and execute

```
roslaunch dvo_benchmark benchmark.launch dataset:=<RGBD dataset folder>
```


## Rviz

After DVO SLAM is running, you can launch rviz to observe some results. 

Target frame is "world".

The cloud is only updated once in a while in the viewer.   
Try to display the interactive markers instead. If it works, you will see the camera icon moving. 

