# ROS多机通信图像传输改善:fire::fire::fire:
### 1.简言
- 本作者是通过手机热点来构建局域网，通过实验发现当从机端使用rqt_image_view工具包去订阅主机端原始相机节点时会出现严重的延迟掉帧现象，甚至是图像直接卡到不动。
- 通过查阅相关资料发现这是由于WiFi传输时带宽问题所导致的，本项目就是针对这个问题所写。
- 建议使用千兆以上的网络来构建局域网或者是提升相关WiFi模块的传输速率，也就是改善相关硬件设备。
### 2.操作步骤
```
mkdir -p ~/speed/src && cd speed/src
git clone https://github.com/LeonardoDiCaprio1/img_improve.git
cd ..
catkin_make
```
### 3.运行指令
```
cd speed 
source devel/setup.bash
rosrun image image.py
rosrun image monitoring_img.py
```
### 4.相关说明
- 请在主机端运行rosrun image image.py
- image.py是压缩原始相机节点，而monitoring_img.py是监听原始相机节点的传输速率，详情请阅览相关代码。
