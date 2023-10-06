# 基于ROS与VIOBOT的自主导航无人车

## 一、项目简介
该项目基于ROS与VIOBOT，实现无人车自主导航。其中VIOBOT为基于视觉惯性里程计的Visn-Fusion系统，无人车通过VIOBOT的输出的里程计信息与点云图，结合mave_base功能包实现自主导航。
## 二、项目环境
### 1. 硬件环境
硬件环境为：
- 电脑：联想Y9000X（戴尔minipc）
- 无人车：PIXKIT（ET-01系列）

### 2. 软件环境
软件环境为：
- 系统：Ubuntu 18.04
- ROS Melodic

## 三、系统框架
[![mave-base.png](https://i.postimg.cc/3JrJj7V6/mave-base.png)](https://postimg.cc/ThS6f8Rq)
[![image.png](https://i.postimg.cc/pVznc20L/image.png)](https://postimg.cc/14RtfZhk)

## 四、使用步骤
### 1.ros(melodic)安装
推荐使用小鱼一键安装
```
wget http://fishros.com/install -O fishros && . fishros
```
### 2.安装mave_base功能包
#### 二进制安装
```
sudo apt-get install ros-kinetic-navigation
```
#### 源码安装
在工作空间的src目录下执行以下命令：
```
git clone https://github.com/ros-planning/navigation.git
```

```
roslaunch car_top car.launch
ros pr_loop_action_demo posegraph_action_client
ros viobot_demo Viobot_ctrl
roslaunch vio_demo vio_odom.launch
roslaunch pointcloud_to_laserscan tofToScan.launch
```