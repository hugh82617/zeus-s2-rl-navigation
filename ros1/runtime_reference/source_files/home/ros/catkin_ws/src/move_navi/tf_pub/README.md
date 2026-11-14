#### 介绍
 TF发布功能包

#### 安装教程
```
cd catkin_ws/src/move_navi
git clone https://gitee.com/mathcy/tf_pub.git
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release

```

#### 使用说明
```
启动2D导航功能：roslaunch tf_pub tf_pub_2d.launch
启动3D导航功能：roslaunch tf_pub tf_pub_3d.launch
启动自动模式导航功能：roslaunch tf_pub move_base.launch

```
