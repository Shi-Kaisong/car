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
sudo apt-get install ros-melodic-navigation
```
#### 源码安装
在工作空间的src目录下执行以下命令：
```
git clone https://github.com/ros-planning/navigation.git
```
然后编译
```
catkin_make
```
### 3.clone该仓库代码
在工作空间的src目录下执行以下命令：
```
git clone https://github.com/shikaison/car.git
```
然后编译
### 4. 进行viobot主从机配置
详细过程见文档[用户手册]:https://github.com/shikaison/car/blob/main/docx/viobot/%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C20230705.pdf
### 5.进行底盘通信USB权限设置
详细过程见文档[ET-01产品使用手册]:https://github.com/shikaison/car/blob/main/docx/%E5%A4%A7%E6%97%A0%E4%BA%BA%E8%BD%A6/ET-01%20%E4%BA%A7%E5%93%81%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C.pdf
### 6.其余相关文档在[docx]目录下
### 6.根据实际情况对mave_base配置文件进行修改
例如地图数据，小车数据等
### 7.运行
+ 启动底盘小车的通讯连接
```
roslaunch car_top car.launch
```
+ 启动viobot的算法（可设置为viobot开机自启动算法，则可省去这一步）
```
ros pr_loop_action_demo posegraph_action_client
roslaunch viobot_demo viobot_demo.launch
```
+ 对viobot输出信息进行处理
```
roslaunch vio_demo vio_odom.launch
```
+ 进行pointcloud转laserscan并启动mave_base功能包
```
roslaunch pointcloud_to_laserscan tofToScan.launch
```