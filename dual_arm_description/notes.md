# 问题记录：从ROS1迁移至ROS2

## 日期：2026-05-25

## 问题1：rviz2中不显示机械臂，joint_state_publisher_gui界面空白

### 原因
URDF中所有关节（包括机械臂14个关节）均为`type="fixed"`，fixed关节不可运动，joint_state_publisher_gui不会为其生成滑块。

### 解决方法
将`laxis1_joint`~`laxis7_joint`和`raxis1_joint`~`raxis7_joint`的type从`fixed`改为`revolute`，并将axis从`0 0 0`改为`0 0 1`。

---

## 问题2：rviz2报错 Frame [map] does not exist

### 原因
`urdf.rviz`配置文件未被安装到ROS2包的share目录。CMakeLists.txt只install了`urdf/`、`meshes/`、`launch/`、`config/`目录，遗漏了根目录下的`urdf.rviz`。rviz2加载不到配置文件，使用默认Fixed Frame `map`，但URDF中不存在该frame。

### 解决方法
在`CMakeLists.txt`中添加：
```cmake
install(FILES urdf.rviz
  DESTINATION share/${PROJECT_NAME}
)
```

---

