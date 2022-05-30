# external_position_sys
利用nooploop的linktrack和tofsense产品分别给无人机提供水平位置和垂直位置信息的rospy节点

# 使用方法
```shell
cd ~/codes/github.com
git clone https://github.com/nooploop-dev/serial.git # 安装串口通信库
cd serial
make # Build
make test # Build and run the tests
sudo make install # Install
cd ~/catkin_ws/src

git clone https://github.com/groers/external_position_sys.git

git clone --recursive https://github.com/nooploop-dev/nlink_parser.git  # 安装linktrack官方解析程序

cd ..
catkin_make

roscore
catkin_make run_tests #执行单元测试


在当前用户的配置文件 `~/.bashrc`下加入命令`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`
```
