# offboard-vicon
借助vicon动作捕捉系统实现无人机机载电脑控制起飞

# 平台
| software | version |
| --------- | ------- |
| ubuntu    | 18.04   |
| ros       | melodic |

# 编译运行
create workspace
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin_init_workspace
```
vicon相关设置可参考[vicon设置](https://gitee.com/ASSIL/vrpn_client_ros)

Download this project and put it into `src`.
```
git clone https://github.com/3113373723/offboard-vicon.git
cd ~/catkin_ws
catkin_make
```
run sample
```
//本项目主要包含两个例程，分别写在两个launch文件中，推荐使用no machine远程连接运行
// 1.机载电脑控制无人机起飞定点悬停15s后降落
source ./devel/setup.bash
roslaunch offboard takeoff.launch

//2.机载电脑控制无人机起飞，飞一个正方形，然后降落
roslaunch mavros px4.launch
//再开一个终端
source ./devel/setup.bash
roslaunch offboard new.launch 
```
