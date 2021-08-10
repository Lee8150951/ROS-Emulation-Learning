# ROS建模与仿真

## 一、统一机器人描述格式URDF

### 1.1概念

URDF全称为Unified Robot Description Format，也就是统一机器人描述格式，它是ROS中一个非常重要的机器人模型描述格式，ROS同时也提供URDF文件的C++解析器，可以解析UDRF文件中使用XML格式描述的机器人模型

### 1.2`<link>`标签

用于描述机器人某个刚体部分的外观和物理属性，如：尺寸、颜色、形状、惯性矩阵、碰撞参数等等

结构如下所示：

```xml
<link name="<link name>">
    <inertial>...</inertial>
    <visual>...</visual>
    <collision>...</collision>
</link>
```

- `<visual>`用于描述link部分的外观参数
- `<inertial>`用于描述link部分的惯性参数
- `<collision>`用于描述link部分的碰撞属性

### 1.3`<joint>`标签

根据机器人关节的运动形式，可以将关节分为六种类型：

> - continuous: 旋转关节，一个不受限制的, 绕着一根轴的转动副
>
> - revolute: 旋转关节，一个转动角度受到限制的, 绕着一根轴的转动副
>
> - prismatic: 滑动关节，一个沿着一根轴的滑动副, 并且有限位
>
> - fixed: 固定关节，不允许运动的特殊关节
>
> - floating: 浮动关节，允许进行平移、旋转运动
>
> - planar: 平面关节，允许在平面正交方向上平移或旋转

机器人关节的主要作用就是连接两个刚体，这两个刚体被称为parent link和child link

结构如下所示：

```xml
<joint>
    <parent link="parent_link"/>
    <child link="child_lind"/>
    <calibration .../>
    <dynamics damping .../>
    <limit effort .../>
    ...
</joint>
```

- `<calibration>`关节参考位置，用于校准关节的绝对位置
- `<dynamics>`用于描述关节的物理属性，常用于动力学仿真
- `<limit>`用于描述运动的一些极限值，包括关节运动的上下限位置、速度限制等等
- `<mimic>`用于描述该关节与已有关节的关系
- `<safety_controller>`用于秒栓全控制器的参数

### 1.4`<robot>`标签

该标签是完整机器人模型的最顶层标签，<link>标签和<joint>标签都必须包含在<robot>标签内

### 1.5`<gazebo>`标签

该标签用于描述机器人模型在Gazebo中仿真所需要的参数，包括机器人材料的属性、Gazebo插件等等

该标签不是必须部分，只有在进行Gazebo仿真时会用到

## 二、创建机器人URDF模型

### 2.1创建功能包

在已建立的工作空间下创建功能包

```shell
$ cakin_create_pkg mrobot_description urdf xacro
```

其中xacro包作用是更大XML表达式的宏来构建更短、更易读的XML文件

xacro是一种XML**宏语言**。

> 补充：宏语言是一种强有力的**工具语言**，可以用来**描述软件**和**解决软件移植**等问题，用于书写宏指命和宏定义的表示法和规则。宏语言是一类编程语言，其全部或多数计算是由扩展宏完成的。宏语言并未在通用编程中广泛使用，但在文本处理程序中应用普遍。

在存储URDF的功能包中包含了urdf、meshes、launch、config四个文件夹：

- urdf：用于存放机器人模型的URDF或者xacro文件
- meshes：用于放置URDF中引用的模型渲染文件
- launch：用于保存相关启动文件
- config：用于保存rviz的配置文件

### 2.2创建URDF模型

模拟创建一个机器人的底盘模型，该模型有7个link和6个joint：

其中7个link分别是：1个机器人底板、2个电机、2个驱动轮、2个万向轮

6个joint负责将驱动轮、万向轮、电机安装到底板上，并设置相应的连接方式

```xml
<?xml version="1.0" ?>
<robot name="mrobot_chassis">

    <link name="base_link">
        <visual>
            <origin xyz=" 0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder length="0.005" radius="0.13"/>
            </geometry>
            <material name="yellow">
                <color rgba="1 0.4 0 1"/>
            </material>
        </visual>
    </link>

    <joint name="base_left_motor_joint" type="fixed">
        <origin xyz="-0.055 0.075 0" rpy="0 0 0" />        
        <parent link="base_link"/>
        <child link="left_motor" />
    </joint>

    <link name="left_motor">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.02" length = "0.08"/>
            </geometry>
            <material name="gray">
                <color rgba="0.75 0.75 0.75 1"/>
            </material>
        </visual>
    </link>

    <joint name="left_wheel_joint" type="continuous">
        <origin xyz="0 0.0485 0" rpy="0 0 0"/>
        <parent link="left_motor"/>
        <child link="left_wheel_link"/>
        <axis xyz="0 1 0"/>
    </joint>

    <link name="left_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.033" length = "0.017"/>
            </geometry>
            <material name="white">
                <color rgba="1 1 1 0.9"/>
            </material>
        </visual>
    </link>

    <joint name="base_right_motor_joint" type="fixed">
        <origin xyz="-0.055 -0.075 0" rpy="0 0 0" />        
        <parent link="base_link"/>
        <child link="right_motor" />
    </joint>

    <link name="right_motor">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.02" length = "0.08" />
            </geometry>
            <material name="gray">
                <color rgba="0.75 0.75 0.75 1"/>
            </material>
        </visual>
    </link>

    <joint name="right_wheel_joint" type="continuous">
        <origin xyz="0 -0.0485 0" rpy="0 0 0"/>
        <parent link="right_motor"/>
        <child link="right_wheel_link"/>
        <axis xyz="0 1 0"/>
    </joint>

    <link name="right_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.033" length = "0.017"/>
            </geometry>
            <material name="white">
                <color rgba="1 1 1 0.9"/>
            </material>
        </visual>
    </link>

    <joint name="front_caster_joint" type="fixed">
        <origin xyz="0.1135 0 -0.0165" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="front_caster_link"/>
    </joint>

    <link name="front_caster_link">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0"/>
            <geometry>
                <sphere radius="0.0165" />
            </geometry>
            <material name="black">
                <color rgba="0 0 0 0.95"/>
            </material>
        </visual>
    </link>

</robot>
```

URDF提供了命令行工具`liburdfdom-tools`，可以帮助检查、梳理模型文件

```shell
$ sudo apt-get install liburdfdom-tools
```

- 使用`check_urdf`命令对mrobot_chassis.urdf文件进行检查

```shell
$ check_urdf mrobot_chassis.urdf
```

结果如下所示：

![2021-08-07 16-07-48 的屏幕截图.png](https://i.loli.net/2021/08/07/LSJrCxPIZ7kMniq.png)

- 使用urdf_to_graphiz命令查看模型整体结构

```shell
$ urdf_to_graphiz mrobot_chassis.urdf
```

使用该命令后会在文件夹中生成一个pdf文件，展示了该模型的结构：

<img src="https://i.loli.net/2021/08/07/YZFAk5vnMr17qOe.png" alt="2021-08-07 16-11-16 的屏幕截图.png" style="zoom: 80%;" />

### 2.3代码解析

#### 2.3.1整体

```xml
<?xml version="1.0" ?>
<robot name="mrobot_chassis">
```

使用`<robot>`标签定义了一个机器人模型，并定义了该机器人名为mrobot_chassis

#### 2.3.2底盘

```xml
<link name="base_link">    <visual>        <origin xyz=" 0 0 0" rpy="0 0 0" />        <geometry>            <cylinder length="0.005" radius="0.13"/>        </geometry>        <material name="yellow">            <color rgba="1 0.4 0 1"/>        </material>    </visual></link>
```

通过`<visual>`标签定义了该底盘的外观属性，将底盘想象成一个扁平的圆柱结构

`<cylinder>`标签定义了这个圆柱的半径和高

`<origin>`标签声明了这个圆柱底盘在空间内的三位坐标和旋转姿态，底盘的中心位于界面的中心点

`<material>`标签设置了颜色`<color>`

#### 2.3.3关节（底盘与电机）

```xml
<joint name="base_left_motor_joint" type="fixed">    <origin xyz="-0.055 0.075 0" rpy="0 0 0" />            <parent link="base_link"/>    <child link="left_motor" /></joint>
```

该关节用于连接机器人底盘和左边驱动电机，这个关节的类型`type="fixed"`是固定关节

`<origin>`标签定义了该关节的起点，将该节点设置在需要安装电机的底盘位置

#### 2.3.4电机

```xml
<link name="left_motor">
    <visual>
        <origin xyz="0 0 0" rpy="1.5707 0 0" />
        <geometry>
            <cylinder radius="0.02" length = "0.08"/>
        </geometry>
        <material name="gray">
            <color rgba="0.75 0.75 0.75 1"/>
        </material>
    </visual>
</link>
```

该段代码描述了左侧电机的模型，该电机模型被抽象为圆柱体并通过`<geometry>`标签进行配置

**注意：由于之前定义了的joint设置连接到了底盘上，所以电机的坐标位置是想对于joint计算的**

由于圆柱体默认是垂直于地面创建的，所以需要把圆柱体围绕x轴旋转90度（换算成弧度约为1.5707）

#### 2.3.5关节（电机与轮子）

```xml
<joint name="left_wheel_joint" type="continuous">
    <origin xyz="0 0.0485 0" rpy="0 0 0"/>
    <parent link="left_motor"/>
    <child link="left_wheel_link"/>
    <axis xyz="0 1 0"/>
</joint>
```

该关节属于continuous类型，这种类型的关节可以围绕着一个轴进行旋转，轮子就很适合这种类型

`<origin>`标签定义了该关节的起点，将起点放置在电机的一段

### 2.4Rviz展示模型

编辑launch

```xml
<launch>
	<param name="robot_description" textfile="$(find mrobot_description)/urdf/mrobot_chassis.urdf" />

	<!-- 设置GUI参数，显示关节控制插件 -->
	<param name="use_gui" value="true"/>
	
	<!-- 运行joint_state_publisher节点，发布机器人的关节状态  -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
	
	<!-- 运行robot_state_publisher节点，发布tf  -->
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
	
	<!-- 运行rviz可视化界面 -->
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find mrobot_description)/config/mrobot_urdf.rviz" required="true" />
</launch>
```

启动launch文件

```shell
$ roslaunch mrobot_description display_mrobot_chassis_urdf.launch
```

**注意：一定要使用命令`catkin_make`编译并`source`后启动**

展示模型如下所示：

<img src="https://i.loli.net/2021/08/07/xBQJsy8GwieCfWa.png" alt="2021-08-07 17-12-40 的屏幕截图.png" style="zoom: 80%;" />

## 三、添加物理和碰撞属性

```xml
<link name="base_link">
     <inertial>
        <mass value="2"/>
        <origin xyz="0.0 0.0 0.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.5"/>
    </inertial>
    <visual>
        <origin xyz=" 0 0 0" rpy="0 0 0" />
        <geometry>
            <cylinder length="0.005" radius="0.13"/>
        </geometry>
        <material name="yellow">
            <color rgba="1 0.4 0 1"/>
        </material>
    </visual>
    <collision>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
            <cylinder length="0.005" radius="0.13"/>
        </geometry>
    </collision>
</link>
```

在base_link中添加`<inertial>`和`<collision>`标签，用于描述物体的物理属性和碰撞属性

- 惯性参数主要包含了物体的质量和惯性矩阵

如果是规则物体可以通过尺寸和质量等公式计算得到惯性矩阵

- `<collision>`标签内容与`<visual>`内容几乎一致，因为本组模型都是较为简单的规则模型

如果是使用真是机器人模型可以使用**抽象**的方式，例如可以将机械臂的一根连杆抽象为圆柱体或长方体

## 四、使用xacro优化URDF

- URDF缺点：在URDF文件中有很多内容除了参数，几乎都是重复的内容，可以使用代码复用技术。但是，URDF并不支持代码复用，导致一个真实的机器人模型将变得非常复杂

针对URDF模型的冗长，ROS有一种精简化、可复用、模块化的描述形式，即xacro

xacro有以下优点：

- 精简模型代码
- 提供可编程接口

xacro是URDF的升级版，模型文件的后缀名由`.urdf`更改为`.xacro`，而且在模型`<robot>`标签中需要添加对xacro的声明

```xml
<?xml version="1.0"?>
<robot name="robot_name" xmlns:xacro="http://www.ros.org/wiki/xacro">
```

### 4.1使用常量

示例如下：

```xml
<!-- 常量的定义 -->
<xacro:property name="M_PI" value="3.14159"/>

<!-- 使用常量 -->
<origin xyz="0 0 0" rpy="${M_PI} 0 0">
```

如果改动机器人模型直接对这些参数进行修改即可

### 4.2使用公式

在`${....}`中不仅可以调用常量还可以进行一些常用的数学运算

```xml
<origin xyz="0 0 0" rpy="${M_PI + 1 / 2} 0 0">
```

### 4.3使用宏定义

xacro文件可以使用宏定义来声明**重复使用**的代码模块，而且可以包含默认参数，类似编程中的**函数概念**

比如：在MRobot模型中底板上有两层支撑板，支撑板之间由八根支撑柱连接而成，而支撑柱除了位置不同，模型是完全一样的，此时就可以使用宏定义的方式对支撑柱进行定义

宏定义示例模板：

```xml
<!-- 模板的定义 -->
<xacro:xacro name="macro_definition_name" params="parameter01 parameter02 parameter03">
    <joint name="joit_${parameter01}" type="fixed">
        <origin xyz="${parameter01} ${parameter02} ${parameter03}" rpy="0 0 0"/>
        <parent link="${parameter01}"/>
        <child link="child_${parameter01}"/>
    </joint>
</xacro:xacro>

<!-- 模板的使用 -->
<macro_definition_name paranmeter01="1" paranmeter02="2" paranmeter03="3"/>
```

## 五、xacro文件的引用

```xml
<?xml version="1.0"?>
<robot name="mrobot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="$(find mrobot_description)/urdf/mrobot_body.urdf.xacro" />
    <!-- MRobot机器人平台 -->
    <mrobot_body/>
</robot>
```

代码解读：

-  `<xacro:include filename="$(find mrobot_description)/urdf/mrobot_body.urdf.xacro" />`

该行代码用于描述该xacro文件所包含的其他的xacro文件，类似于C语言中include文件

声明包含关系之后，该文件就可以使用被包含文件中的模块

- `<mrobot_body/>`

直接调用该文件的文件名作为标签名即可

在这里，机器人的模型文件全部是在mrobot_body.urdf.xacro文件中使用一个宏来定义的

**问题：为何需要mrobot.urdf.xacro文件来包含调用？**

这里将整个机器人本体视作一个模块，如果需要与其他模块进行集成，使用这种方式就不需要修改机器人的模型文件，只需要在上层实现一个拼装模块的顶层文件即可，灵活性更强

## 六、优化模型

### 6.1转化文件

将xacro文件转换为URDF文件：

```shell
$ rosrun xacro xacro.py mrobot.urdf.xacro > mrobot.urdf
```

调用该命令后当前目录下会生成一个URDF文件，然后使用launch文件即可将URDF模型显示在rviz中

### 6.2直接调用xacro文件解析器

可以忽略手动转化模型的过程，直接在启动文件中调用xacro解析器，自动将xacro转为URDF文件

该过程可以在launch文件中使用如下语法进行配置

```xml
<arg name="model" default="$(find xacro)/xacro --inorder '$(find mrobot_description)/urdf/mrobot.urdf.xacro'" />

<param name="robot_description" command="$(arg model)" />
```

在终端中运行修改后的launch文件即可启动

结果如下所示：

<img src="https://i.loli.net/2021/08/09/Xnc4ZDCgsQpl17f.png" alt="2021-08-09 12-07-03 的屏幕截图.png" style="zoom: 80%;" />

## 七、添加传感器

### 7.1添加摄像头

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="camera">
    <xacro:macro name="usb_camera" params="prefix:=camera">
        <link name="${prefix}_link">
            <inertial>
                <mass value="0.1" />
                <origin xyz="0 0 0" />
                <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                         iyy="0.01" iyz="0.0"
                         izz="0.01" />
            </inertial>
            <visual>
                <origin xyz=" 0 0 0 " rpy="0 0 0" />
                <geometry>
                    <box size="0.01 0.04 0.04" />
                </geometry>
                <material name="black"/>
            </visual>
            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
                <geometry>
                    <box size="0.01 0.04 0.04" />
                </geometry>
            </collision>
        </link>
    </xacro:macro>
</robot>
```

成果：

<img src="https://i.loli.net/2021/08/10/ZR2s7nqtL19uQYb.png" alt="2021-08-10 11-07-42 的屏幕截图.png" style="zoom:80%;" />

### 7.2添加Kinect

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="kinect_camera">
    <xacro:macro name="kinect_camera" params="prefix:=camera">
        <link name="${prefix}_link">
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <visual>
                <origin xyz="0 0 0" rpy="0 0 ${M_PI/2}"/>
                <geometry>
                    <mesh filename="package://mrobot_description/meshes/kinect.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <box size="0.07 0.3 0.09"/>
                </geometry>
            </collision>
        </link>
        <joint name="${prefix}_optical_joint" type="fixed">
            <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
            <parent link="${prefix}_link"/>
            <child link="${prefix}_frame_optical"/>
        </joint>
        <link name="${prefix}_frame_optical"/>
    </xacro:macro>
</robot>
```

成果：

<img src="https://i.loli.net/2021/08/10/PGfFUmuZ9bolWJA.png" alt="2021-08-10 11-06-05 的屏幕截图.png" style="zoom:80%;" />

### 7.3添加激光雷达

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="laser">
	<xacro:macro name="rplidar" params="prefix:=laser">
		<link name="${prefix}_link">
			<inertial>
				<mass value="0.1" />
				<origin xyz="0 0 0" />
				<inertia ixx="0.01" ixy="0.0" ixz="0.0"
						 iyy="0.01" iyz="0.0"
						 izz="0.01" />
			</inertial>
			<visual>
				<origin xyz=" 0 0 0 " rpy="0 0 0" />
				<geometry>
					<cylinder length="0.05" radius="0.05"/>
				</geometry>
				<material name="black"/>
			</visual>
			<collision>
				<origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
				<geometry>
					<cylinder length="0.06" radius="0.05"/>
				</geometry>
			</collision>
		</link>
	</xacro:macro>
</robot>
```

成果：

<img src="https://i.loli.net/2021/08/10/Oxw7K2PcnpLr3ag.png" alt="2021-08-10 11-09-24 的屏幕截图.png" style="zoom:80%;" />

## 八、基于ArbotiX和rviz的仿真器

ArbotiX是一款控制电机、舵机的控制板，并提供了相应的ROS功能包，该功能包不仅可以驱动真实的ArbotiX控制板，还提供了一个差速控制器，通过接受速度控制指令更新机器人的joint状态

### 8.1安装ArbotiX

问题很大一部分集中于此，《ROS机器人开发实践》一书上所描述的方法基本上全是错误的

#### 8.1.1问题①

首先第一种方法，书中写道：`Indigo`版本ROS软件源中集成了ArbotiX功能包的二进制安装文件，使用命令如下即可安装：

```shell
$ sudo apt-get install ros-indiago-arbotix-*
```

但是目前为止，已在维护的ROS版本的软件源中均已集成ArbotiX功能包的二进制安装文件，只需要将ros-indiago更改为所对应版本即可

```shell
$ sudo apt-get install ros-***-arbotix
$ sudo apt-get install ros-***-arbotix-*
```

但是在使用该方式时遇到了另一些问题，在第三节中附解决方法

#### 8.1.2问题②

第二种方法，《ROS》一书中表示使用Git方法对所需ArbotiX功能包进行下载，保存至工作空间中编译即可

```shell
$ git clone https://github.com/vanadiumlabs/arbotix_ros.git
```

该方法也存在一定的误区，应使用如下命令：

```shell
$ git clone -b indigo-devel https://github.com/vanadiumlabs/arbotix_ros.git
```

原因：在git上下载的一定要选择indigo分支的源码，如果命令中没加上`-b indigo-devel`，下载的则是默认的最新的`noetic`版本，里面用的是python3，会和python2路径有冲突（可能会报`ImportError:dynamic module does not define module export function(PyInit_tf2)`等错误）

**注意：安装一定要在/src文件下进行，否则无法识别节点并启动**

### 8.2配置ArbotiX控制器

#### 8.2.1创建launch文件

在原有机器人模型的基础上添加启动arbotix_driver节点即可：

```xml
<node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
    <rosparam file="$(find mrobot_description)/config/fake_mrobot_arbotix.yaml" command="load" />
    <param name="sim" value="true"/>
</node>
```

arbotix_driver可以针对真实控制板进行控制，也可以在仿真环境中使用，但是需要配置`sim`为`true`

#### 8.2.2创建配置文件

```yaml
controllers: {   base_controller: {type: diff_controller, base_frame_id: base_footprint, base_width: 0.26, ticks_meter: 4100, Kp: 12, Kd: 12, Ki: 0, Ko: 50, accel_limit: 1.0 }}
```

### 8.3运行仿真环境

- 仿真环境搭建完成后使用以下命令运行：

```shell
$roslaunch mrobot_description arbotix_mrobot_with_kinect.launch
```

- 导入arbotix_driver节点功能包并运行：

```shell
$ roslaunch mrobot_teleop mrobot_teleop.launch
```

结果如下所示：

<img src="https://i.loli.net/2021/08/10/SflwH7pOoBUXtGu.png" alt="2021-08-10 19-33-10 的屏幕截图.png" style="zoom:80%;" />

## 九、ros_control

### 9.1概念

ROS提供了相当多的功能包，如：SLAM、导航、MoveIt等等，但是这些功能包的应用缺少了控制的中间件

ros_control就是应用和实际机器人或者机器人仿真器之间的中间件

ros_control就是ROS为用户提供的应用与机器人之间的中间件，包含一系列控制器接口、传动装置接口、硬件接口、控制器工具箱等等，可以帮助机器人应用快速落地，提高开发效率

<img src="https://i.loli.net/2021/08/11/A8IxVbE5MT9jNgU.png" alt="2021-08-11 09-57-29 的屏幕截图.png" style="zoom: 67%;" />

### 9.1ros_control框架

ros_control框架针对不同类型的机器人（移动机器人、机械臂等等）提供了多种类型的**控制器**（controller），但是这些控制器的接口各不相同

ros_control框架还提供了一个**硬件抽象层**，负责机器人硬件资源的管理，而controller从抽象层请求资源即可，无需直接接触硬件

<img src="https://i.loli.net/2021/08/11/uZjTApgV6QPWEvw.png" alt="2021-08-11 09-57-38 的屏幕截图.png" style="zoom:67%;" />

- 控制器管理器（Controller Manager）每一个机器人系统可能会有多个控制器，控制器管理器的作用就是提供一种通用的接口来管理不同的控制器，其输入就是ROS上层应用功能包的输出
- 控制器（Controller）控制器可以完成每个joint的控制，读取硬件资源接口中的状态，然后再发布控制命令并提供PID控制器
- 硬件资源（Hardware Resource）为上下两层提供硬件资源的接口
- 机器人硬件抽象（RobotHW）机器人硬件抽象和硬件资源直接交互，通过read和write方法完成对硬件的操作，这一层也包含了关节约束、例句转换、状态转换等功能
- 真实机器人（Real Robot）真实机器人上需要有自己的嵌入式控制器，将接收到的命令反映到执行器上

下面反映的就是一个完整的ros_control数据流图

<img src="https://i.loli.net/2021/08/11/iKQ5gnJBvatxuoO.png" alt="2021-08-11 09-58-08 的屏幕截图.png" style="zoom:80%;" />

### 9.2控制器

目前ROS中的ros_controllers功能包提供了以下控制器：

![2021-08-11 10-30-11 的屏幕截图.png](https://i.loli.net/2021/08/11/JUsNHFuDkQxSwfO.png)

![2021-08-11 10-30-35 的屏幕截图.png](https://i.loli.net/2021/08/11/vxt9DLQEaoe2BnT.png)

也可以根据自己的需求，创建需要的controller，然后通过控制器管理器（controller manager）来管理自己创建的controller

可参考`https://github.com/ros-controls/ros_control/wiki/controller_interface`

### 9.3硬件接口

硬件接口是控制器与机器人硬件抽象（RobotHW）沟通的接口，基本与控制器种类相互对应

![2021-08-11 10-36-00 的屏幕截图.png](https://i.loli.net/2021/08/11/tJz4cMWeBakoQX1.png)

![2021-08-11 10-36-10 的屏幕截图.png](https://i.loli.net/2021/08/11/hPaETIKjYJy8M3d.png)

同样可以自己创建需要的接口，可参考`https://github.com/ros-controls/ros_control/wiki/hardware_interface`

### 9.4传动系统

传动系统（Transmission）可以将机器人的关节指令转化为执行器的控制信号

**注意：机器人的每一个需要运动的关节都需要配置相应的传动系统**

其配置可以在机器人的URDF文件中按照以下方式进行：

```xml
<transmission name="simple_trans">
     <type>transmission_interface/SimpleTransmission</type>
     <joint name="foo_joint">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
     </joint>
     <actuator name="foo_motor">
          <mechanicalReduction>50</mechanicalReduction>
          <hardwareInterface>EffortJointInterface</hardwareInterface>
     </actuator>
</transmission>
```

### 9.5关节约束

关节约束（Joint Limits）是硬件抽象层中的一部分，维护一个关节约束的数据结构

这些约束数据可以从机器人的URDF文件中加载出来，也可以在ROS的参数服务器中加载（需要先用YAML文件导入ROS参数服务器）

其包含的约束包括：关节速度、位置、加速度、加加速度、力矩、位置软限位、速度边界、位置边界等等

可以使用如下方法在URDF中进行设置：

```xml
<joint name="$foo_joint" type="revolute">
  <!-- other joint description elements -->

  <!-- Joint limits -->
  <limit lower="0.0" upper="1.0" effort="10.0" velocity="5.0" />
 
  <!-- Soft limits -->
  <safety_controller k_position="100" k_velocity="10" soft_lower_limit="0.1" soft_upper_limit="0.9" /> 
</joint>
```

还有一些参数需要通过YAML配置文件事先加载到参数服务器中，示例如下所示：

```yaml
joint_limits:
  foo_joint:
    has_position_limits: true
    min_position: 0.0
    max_position: 1.0
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 5.0
    has_jerk_limits: true
    max_jerk: 100.0
    has_effort_limits: true
    max_effort: 5.0
  bar_joint:
    has_position_limits: false # Continuous joint
    has_velocity_limits: true
    max_velocity: 4.0
```

### 9.6控制器管理器

控制器管理器提供了一种多控制的机制，可以实现控制器的加载、开始运行、停止运行、卸载等多种操作

此外，控制器管理器还提供了多种工具来辅助操作

#### 9.6.1命令行工具

标准使用格式：

```shell
$ rosrun controller_manager controller_manager <command> <controller_name>
```

支持的`<command>`命令如下所示：

- load：加载控制器
- unload：卸载控制器
- start：启动控制器
- stop：停止控制器
- spawn：加载并启动控制器
- kill：停止并卸载控制器

如果希望查看某一个控制器的状态，使用命令：

```shell
$ rosrun controller_manager controller_manager <command>
```

支持的`<command>`命令如下所示：

- list：根据执行顺序列出所有控制器
- list-types：显示所有控制器的类型
- reload-libraries：以插件形式重新加载所有控制器的库，不需要重新启动，方便对控制器的开发和测试
- reload-libraries--restore：以插件形式重新加载所有控制器的库，并恢复到初始状态

使用spawner命令一次控制多个控制器：

```shell
$ rosrun controller_manager spawner [--stopped] name1 name2 name3
```

加上`--stop`命令表示仅加载不执行

如果想要停止一系列的控制器，但不需要卸载，使用如下命令：

```shell
$ rosrun controller_manager unspawner name1 name2 name3
```

#### 9.6.2launch工具

在launch文件中，同样可以通过运行controller_manager包的命令，来加载和启动一系列controller

```xml
<launch>
   <node pkg="controller_manager" type="spawner" args="controller_name1 controller_name2" /> 
</launch>
```

上边的launch文件会加载并启动controllers，如果只需要加载：

```xml
<launch>
  <node pkg="controller_manager" type="spawner" args="--stopped controller_name1 controller_name2" />
</launch>
```

#### 9.6.3可视化工具

controller_manager还提供了可视化工具`rqt_controller_manager`

安装：`rosrun rqt_controller_manager rqt_controller_manager`，直接使用下边的命令打开：

```shell
$ rosrun rqt_controller_manager rqt_controller_manager
```

## 十、Gazebo仿真

### 10.1引入Gazebo属性

首先应该确保每个link的`<inertia>`元素已经进行了合理的设置

然后为每个必要的`<link>`、`<joint>`、`<robot>`设置`<gazebo>`标签

#### 10.1.1为link添加`<gazebo>`标签

每一个link都应该添加`<gazebo>`标签，其包含的属性只有material，该属性与`<visual>`中的material作用相同

由于Gazebo无法通过visual来配置外观属性，所以需要单独设置，否则会采用默认的灰白色

```xml
<gazebo reference="wheel_${lr}_link">
    <material>Gazebo/Black</material>
</gazebo>
```

#### 10.1.2添加传动

**目的：为驱动机器人提供动力源**

需要在模型中加入`<transmission>`元素，将传动与joint绑定

```xml
<!-- Transmission is important to link the joints and the controller -->
<transmission name="wheel_${lr}_joint_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="base_to_wheel_${lr}_joint" />
    <actuator name="wheel_${lr}_joint_motor">
        <hardwareInterface>VelocityJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

代码解析：

- `<joint name="base_to_wheel_${lr}_joint" />`用于绑定对应的驱动器joint
-  `<type>transmission_interface/SimpleTransmission</type>`用于设定使用的传动装置类型
- `<hardwareInterface>VelocityJointInterface</hardwareInterface>`配置使用的硬件接口类型，这里使用的是速度接口类型

#### 10.1.3添加Gazebo控制器插件

Gazebo插件赋予了URDF模型更加强大的功能，可以帮助模型绑定ROS消息，从而完成传感器的方针输出以及对电机的控制

Gazebo插件可以根据插件的作用范围应用到URDF模型的`<robot>`、`<link>`、`<joint>`上，需要使用`<gazebo>`标签进行包裹

#### （1）为robot元素添加插件

```xml
<gazebo>
    <plugin name="unique_name" filename="plugin_name.so">
        ... plugin parameters ...
    </plugin>
</gazebo>
```

如果`<gazebo>`元素中没有设置`reference="x"`属性，则默认应用于`<robot>`标签

#### （2）为link、joint添加插件

```xml
<gazebo reference="your_link_name">
    <plugin name="unique_name" filename="plugin_name.so">
        ... plugin parameters ...
    </plugin>
</gazebo>
```

Gazebo插件支持的种类在ROS默认安装路径下的/opt/ros/melodic/lib文件夹中可以找到

**所有插件都是以`libgazeboXXX.so`方式进行命名的**

#### （3）范例

Gazebo提供的差速控制插件的应用

```xml
<!-- controller -->
<gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
        <rosDebugLevel>Debug</rosDebugLevel>
        <publishWheelTF>true</publishWheelTF>
        <!-- 机器人命名空间，插件所有数据的发布、订阅都在该命名空间中 -->
        <robotNamespace>/</robotNamespace>
        <publishTf>1</publishTf>
        <publishWheelJointState>true</publishWheelJointState>
        <alwaysOn>true</alwaysOn>
        <updateRate>100.0</updateRate>
        <legacyMode>true</legacyMode>
        <!-- 左右轮转动的关节joint，插件需要控制这两个joint转动 -->
        <leftJoint>base_to_wheel_left_joint</leftJoint>
        <rightJoint>base_to_wheel_right_joint</rightJoint>
        <!-- 模型相关尺寸，在计算差速参数时会用到 -->
        <wheelSeparation>${base_link_radius*2}</wheelSeparation>
        <wheelDiameter>${2*wheel_radius}</wheelDiameter>     
        <wheelTorque>30</wheelTorque>
        <!-- 车轮转动的加速度 -->
        <wheelAcceleration>1.8</wheelAcceleration>
        <!-- 控制器订阅的速度控制指令，在ROS中通常都命名为cmd_vel -->
        <commandTopic>cmd_vel</commandTopic>
        <!-- 里程计数据的参考坐标系，在ROS中通常为odom -->
        <odometryFrame>odom</odometryFrame> 
        <odometryTopic>odom</odometryTopic> 
        <robotBaseFrame>base_footprint</robotBaseFrame>
    </plugin>
</gazebo> 
```

### 10.2模型显示

首先需要更改launch文件，运行Gazebo并加载机器人模型，并启动必要节点

```xml
<launch>

    <!-- 设置launch文件的参数 -->
    <arg name="world_name" value="$(find mrobot_gazebo)/worlds/playground.world"/>
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

    <!-- 运行gazebo仿真环境 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(arg world_name)" />
        <arg name="debug" value="$(arg debug)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="headless" value="$(arg headless)"/>
    </include>

    <!-- 加载机器人模型描述参数 -->
    <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find mrobot_gazebo)/urdf/mrobot.urdf.xacro'" /> 

    <!-- 运行joint_state_publisher节点，发布机器人的关节状态  -->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ></node> 

    <!-- 运行robot_state_publisher节点，发布tf  -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"  output="screen" >
        <param name="publish_frequency" type="double" value="50.0" />
    </node>

    <!-- 在gazebo中加载机器人模型-->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
          args="-urdf -model mrobot -param robot_description"/> 

</launch>
```

该launch文件中主要做了两项工作：

- 启动机器人状态并发布节点，同事家在带有Gazebo属性的机器人URDF模型
- 启动Gazebo，并将机器人模型加载到Gazebo仿真环境中

启动命令：`roslaunch mrobot_gazebo view_mrobot_gazebo.launch`

结果如下所示：

<img src="https://i.loli.net/2021/08/11/BKcrRSDkiIeLa1b.png" alt="2021-08-11 12-24-58 的屏幕截图.png" style="zoom:80%;" />

### 10.3运动控制

由于模型中已经加入了`libgazebo_ros_diff_drive.so`插件，可以使用差速控制器控制机器人运动

启动命令：`$ roslaunch mrobot_teleop mrobot_teleop.launch`，即可运转

<img src="https://i.loli.net/2021/08/11/Iftv8dMglVhjHZT.png" alt="2021-08-11 12-42-55 的屏幕截图.png" style="zoom:80%;" />

### 10.4摄像头仿真

为机器人模型添加一个摄像头插件，让机器人能看到Gazebo中的虚拟环境

同样的需要在摄像头camera的URDF模型文件中添加`<gazebo>`相关配置

两个<gazebo>标签分别配置模型的样式和配置摄像头的各种属性

配置完成后运行仿真环境：`roslaunch mrobot_gazebo view_mrobot_with_camera_gazebo.launch`

使用rqt工具查看当前机器人眼前的世界

```shell
$ rqt_image_view
```

启动结果如下所示：

<img src="https://i.loli.net/2021/08/11/iJTzo3w97NnEcXa.png" alt="2021-08-11 13-57-00 的屏幕截图.png" style="zoom:80%;" />

### 10.5Kinect仿真

同样的，使用<gazebo>标签对URDF模型中的参数进行相关配置

完成后运行仿真环境：`roslaunch mrobot_gazebo view_mrobot_with_kinect_gazebo.launch`

运行仿真环境后可以使用RViz查看Kinect的点云数据：

```shell
$ rosrun rviz rviz
```

> 如果要在RViz中查看点云信息需要在软件中将Fixed Frame设置为`camera_frame_optical`并添加PointCloud2类型插件

结果如下所示：

![2021-08-11 14-05-02 的屏幕截图.png](https://i.loli.net/2021/08/11/BLkQYDlx9zcX45r.png)

### 10.6激光雷达仿真

同摄像头仿真和Kinect仿真，激光雷达仿真在配置gazebo相关设定后启动

```shell
$ roslaunch mrobot_gazebo view_mrobot_with_laser_gazebo.launch
```

另外使用RViz可以查看激光数据

```shell
$ rosrun rviz rviz
```

> 如果要在RViz中查看点云信息需要在软件中将Fixed Frame设置为`base_footprint`并添加LaserScan类型插件

结果如下所示：

<img src="https://i.loli.net/2021/08/11/KyRODcp62GjY4lk.png" alt="2021-08-11 14-18-59 的屏幕截图.png" style="zoom:80%;" />

## 十一、问题解决

### 11.1RViz无法显示模型

在使用`roslaunch mrobot_description display_mrobot_chassis_urdf.launch`命令后启动launch文件

成功启动三个节点并打开了Rviz软件，但是软件中出现了如下问题并无法展示URDF模型

![2021-08-07 17-01-19 的屏幕截图.png](https://i.loli.net/2021/08/07/uIsZObmjqSLXlEw.png)

- 解决办法

第一步：打开终端输入指令

```shell
$ rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 50
```

第二步：在rviz软件界面左上角Fixed Frame的map选项该改成base_link

![2021-08-07 17-09-15 的屏幕截图.png](https://i.loli.net/2021/08/07/VsmiSbgc73vEW12.png)

第三步：在rviz软件左下角点击Add按键，出现弹窗并选择RobotModel，点击OK即可正确展示

<img src="https://i.loli.net/2021/08/07/OSZWeCfAmDGUNhy.png" alt="2021-08-07 17-11-01 的屏幕截图.png" style="zoom: 80%;" />

### 11.2`.xacro`文件转为`.urdf`报错

在*1.4.1转化模型*中控制台出现错误如下：

![2021-08-09 12-08-58 的屏幕截图.png](https://i.loli.net/2021/08/09/qoF8DadY3ExObQM.png)

原因是在mrobot.urdf.xacro文件中出现的`<xacro:include filename="$(find mrobot_description)/urdf/mrobot_body.urdf.xacro" />`

由于路径的问题，解析器无法找到对应的mrobot_description文件夹

将其更正为相对路径即可解决：`<xacro:include filename="../urdf/mrobot_body.urdf.xacro" />`

### 11.3Arbotix安装404错误

在安装ArbotiX时，如果采用了第一种方式进行安装，很有可能会出现资源包无法找到并报404的错误

首先需要使用命令`sudo apt-get update`，控制台报错`EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>`

这个问题主要原因是ROS GPG**密钥过期**

- 解决办法：

控制台输入命令：

```shell
# 更新密钥$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654# 更新软件源$ sudo apt-get update
```

安装并更新密钥后即可解决ROS资源包并报404错误的问题

- 暴力解法：

直接在资源库中找到相应资源包并安装，网址：`http://packages.ros.org/ros/ubuntu/pool/main/r/`

### 11.4`Resource not found: gazebo_ros`

在首次运行命令式出现如下错误，问题根本原因在于没有安装Gazebo仿真环境

<img src="https://i.loli.net/2021/08/11/Gx1efmcF8XuL6Yr.png" alt="2021-08-11 12-14-22 的屏幕截图.png" style="zoom:67%;" />

- 解决办法：

```shell
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

### 11.5`Gazebo [Err] [REST.cc:205] Error in REST request`

运行终端命令启动Gazebo报错，Gazebo页面卡住黑屏，在终端出现`Gazebo [Err] [REST.cc:205] Error in REST request`

<img src="https://i.loli.net/2021/08/11/5SFZteRQzg673jl.png" alt="2021-08-11 12-21-00 的屏幕截图.png" style="zoom:67%;" />

- 解决办法：

```shell
 $  sudo gedit ~/.ignition/fuel/config.yaml
```

打开文件并将`url : https://api.ignitionfuel.org`修改为`url: https://api.ignitionrobotics.org`  

