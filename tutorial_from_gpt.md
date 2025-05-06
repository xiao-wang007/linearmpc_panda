Sure! Here's a **minimal, complete example** of how to run your **custom controller for the Franka Panda arm in Gazebo (ROS Noetic)** using the `franka_gazebo` setup — **not using panda\_simulator**.

---

## ✅ 1. Controller Package Structure (Minimal)

```bash
my_panda_controller_package/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── my_qp_controller.yaml
├── launch/
│   └── load_my_qp_controller.launch
├── src/
│   └── my_qp_controller.cpp
├── include/
│   └── my_panda_controller_package/
│       └── my_qp_controller.h
└── my_panda_controller_plugin.xml
```

---

## ✅ 2. `my_qp_controller.yaml`

```yaml
my_qp_controller:
  type: my_panda_controller_package/QPController
  joint_names:
    - panda_joint1
    - panda_joint2
    - panda_joint3
    - panda_joint4
    - panda_joint5
    - panda_joint6
    - panda_joint7
```

---

## ✅ 3. `my_panda_controller_plugin.xml`

```xml
<library path="libmy_panda_controller_package">
  <class name="my_panda_controller_package/QPController"
         type="panda_controllers::QPController"
         base_class_type="controller_interface::ControllerBase"/>
</library>
```

---

## ✅ 4. Load Controller Launch File (`load_my_qp_controller.launch`)

```xml
<launch>
  <!-- Load parameters -->
  <rosparam file="$(find my_panda_controller_package)/config/my_qp_controller.yaml" command="load"/>

  <!-- Load and start controller -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
        output="screen" args="my_qp_controller"/>
</launch>
```

---

## ✅ 5. Launch in Gazebo

```bash
roslaunch franka_gazebo panda.launch controller:=my_qp_controller
```

Make sure your `panda.launch` ultimately includes a line like:

```xml
<include file="$(find my_panda_controller_package)/launch/load_my_qp_controller.launch"/>
```

You can modify `franka_gazebo/launch/robot.launch` or make a wrapper launch file that loads your controller after spawning the robot.

---

## ✅ 6. CMakeLists.txt (key parts)

```cmake
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  franka_hw
  franka_gripper
)

add_library(${PROJECT_NAME}
  src/my_qp_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES my_panda_controller_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

---

## ✅ 7. package.xml (key parts)

```xml
<build_depend>pluginlib</build_depend>
<exec_depend>pluginlib</exec_depend>
<export>
  <controller_interface plugin="${prefix}/my_panda_controller_plugin.xml"/>
</export>
```

---

## ✅ 8. Build

```bash
catkin build
source devel/setup.bash
```

---

## ✅ 9. Done!

Now launch everything:

```bash
roslaunch franka_gazebo panda.launch controller:=my_qp_controller
```

This setup ensures your custom controller loads properly in Gazebo.

---

Would you like me to generate this as a ready-to-copy package skeleton for you?
