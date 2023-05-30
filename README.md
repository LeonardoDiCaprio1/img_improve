# Ros多机通信图像传输改善
### 1.简言
- 通过试验发现当从机端订阅主机端时会出现延迟掉帧现象，甚至是通过图像卡成PPT。这是由于WiFi传输时带宽问题所导致的。
### 2.操作步骤
```
mkdir -p ~/improve/src && cd improve/src
git clone https://github.com/LeonardoDiCaprio1/img_improve.git
cd ..
catkin_make
```
### 3.运行指令
```
cd improve 
source devel/setup.bash
rosrun image image.py
rosrun image monitoring_img.py
```
### 4.相关说明
image.py是压缩原始相机节点，而monitoring_img.py是监听原始相机节点或压缩相机节点的传输速率，详情请阅览相关代码
