# How to build DVO SLAM for Linux Mint Rosa 17.3 (based on Ubuntu Trusty 14.04 LTS)

The build has been tested on Linux Mint Rosa 17.3 (Ubuntu Trusty 14.04) + ROS Indigo. 
It also worked on Ubuntu Xenial 16.04 + ROS Kinetic. 
The following steps assume ROS Indigo. 

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
cmake .. -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_LGPL_SHARED_LIBS:BOOL=OFF -DG2O_BUILD_APPS:BOOL=OFF -DG2O_BUILD_EXAMPLES:BOOL=OFF
make
sudo make install
```

If your system has Eigen >= 3.3, don't use it with g2o. The old version of g2o is only compatible to Eigen 3.2.8. 
Download Eigen 3.2.8, put it into the EXTERNAL folder in g2o, and use this CMake command instead:

```
cmake .. -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_LGPL_SHARED_LIBS:BOOL=OFF -DG2O_BUILD_APPS:BOOL=OFF -DG2O_BUILD_EXAMPLES:BOOL=OFF -DG2O_EIGEN3_INCLUDE=/home/sutd/Workspace/g2o/EXTERNAL/eigen3.2.8/

```
This will force g2o to use our Eigen library instead of the system Eigen.

We have to build g2o into static libraries 
and build DVO SLAM also as static libraries to avoid some weird undefined reference to g2o::csparse_extension. It could be possible to build as shared libraries (by default) but somehow it does not work on my machine.

## Building DVO SLAM
Go back to Catkin workspace's root folder and execute

```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

to build all packages. You can also build each package separately by 

```
catkin_make --pkg dvo_core -DCMAKE_BUILD_TYPE=Release
catkin_make --pkg dvo_ros -DCMAKE_BUILD_TYPE=Release
catkin_make --pkg dvo_slam -DCMAKE_BUILD_TYPE=Release
catkin make --pkg dvo_benchmark -DCMAKE_BUILD_TYPE=Release
```

## How to run

```
source devel/setup.sh  
```

This will make ROS able to find our newly built packages. 

Now we can navigate to example folder and execute

```
roslaunch launch/benchmark.launch dataset:=<RGBD dataset folder>
```

You can check the output trajectory in example/output/trajectory.txt. 

Dataset follows structures in RGBD SLAM by TUM. The only difference is 
the depth scale is reset 1000 (in benchmark_slam.cpp) instead of 5000.
I found this to be more convenient as images from Kinect or Asus Xtion all follows this convention. 
 
The focal length is hard coded in benchmark_slam.cpp. Change it to your focal length. 

## Rviz

After DVO SLAM is running, you can launch rviz to observe some results. 

In global options in the left panel, set target frame is "world".
Then click Add button, switch to By Topic tab, and choose PointCloud2. You can also add Interactive Markers. 

If you see nothing in rviz but DVO is running, it could be because the project is built in Debug mode, which is slow. Make sure -DCMAKE_BUILD_TYPE=Release is used with catkin_make. 


