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
[![mave-base.png](https://i.postimg.cc/4477pkPL/mave-base.png)](https://postimg.cc/0rvyq4LD)
[![image.jpg](https://i.postimg.cc/HkKFcMkP/image.jpg)](https://postimg.cc/pmfs3p8D)

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
+ 主机：viobot
+ 从机：电脑
+ 具体配置方法（仅限ros1。ros2配置方法见文档[用户手册]:https://github.com/shikaison/car/blob/main/docx/viobot/%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C20230705.pdf
（1）修改从机/etc/hosts
```
sudo gedit /etc/hosts
#在文件最后添加
192.168.1.100  PR-VIO  #192.168.1.100修改为设备对应ip
```
（2）配置从机
```
sudo gedit /home/username/.bashrc   #userbame是自己电脑的用户名
#在文件最后添加
export ROS_MASTER_URI=http://192.168.1.100:11311   #192.168.1.100修改为设备对应ip
```
（3）测试
从机修改并保存好以上两个文件之后，新开一个终端
```
rostopic list
rostopic echo /imu #查看imu信息，终端不断打印imu数据的时候，主从机已经配置完成
```
(4)控制和操作
该配置用于双向交互，一般可以不配置，看用户个人需求
ssh进入viobot设备
```
ssh PRR@192.168.1.100 #密码为PRR，192.168.1.100为设备ip
sudo vim /etc/hosts
#在文件最后添加
192.168.1.101 VIO-slave  #192.168.1.101为电脑ip
```
（5）测试控制
按照上述方法配置好之后，先将demo的msg拉到你从机的工作空间里面去编译。
```
cd catkin_ws #catkin_ws对应你自己的工作空间
catkin_make  -DCATKIN_WHITELIST_PACKAGES="sensor_pub;loop_action;system_ctrl"
source ./devel/setup.bash
```
在从机开启RQT验证是否配置完成：
```
rqt
```
[![vio1.png](https://i.postimg.cc/7YKsHrYp/vio1.png)](https://postimg.cc/bs2HgMnT)
[![vio2.png](https://i.postimg.cc/26dGgPK9/vio2.png)](https://postimg.cc/N9MRHPYR)
[![vio3.png](https://i.postimg.cc/W1DnsDyk/vio3.png)](https://postimg.cc/zyrWjf8z)
### 5.进行底盘通信USB权限设置
（1）创建一个新的udev规则。名称取为：99-myusb.rules
```
sudo gedit /etc/udev/rules.d/99-myusb.rules
```
然后在打开的文件中输入
```
ACTION=="add",SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8",
ATTRS{idProduct}=="0053", GROUP="users", MODE="0777"
```
保存后将usb线重插拔即可。
### 6.其余相关文档在[docx]目录下
### 7.根据实际情况对mave_base配置文件等进行修改
例如地图数据(需要提前建立一个地图先验，如使用viobot提前建图)，小车数据等
### 8.运行
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