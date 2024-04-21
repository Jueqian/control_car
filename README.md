
## 首先需要给USB权限

sudo chmod 666 /dev/ttyUSB0

电脑密码是 1 

### 通过launch文件进行配置

最重要看看是不是USB0

### 最后运行程序

roslaunch control_car run.launch


### 通过下面指令发布速度，进行测试

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 10, y: 0, z: 0}, angular: {x: 0, y: 0, z: 1}}'

### 启动手柄控制程序（在launch文件中修改即可）

然后通过按键修改速度因子，调整即可。


