# Getting Started
本页介绍如何下载和安装软件，运行MIT控制器，以及如何创建自己的控制器



# 安装依赖项

Packages（命令行安装包指令）:
```
sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev
```

Others（其他安装包指令）:
- LCM 1.3.1 (it says Java 6, but you can use newer) (https://lcm-proj.github.io/)
- Qt 5.10.0 or newer (requires the gamepad library) (https://www.qt.io/download-qt-installer)
- Eigen (http://eigen.tuxfamily.org/)

NOTE: on Ubuntu 18.10 or 19.04, you may instead install Qt with
```
sudo apt install libqt5 libqt5gamepad5
```




# 下载并编译代码

```
git clone https://github.com/mit-biomimetics/Cheetah-Software.git
cd Cheetah-Software
cd scripts # for now, you must actually go into this folder
./make_types.sh # you may see an error like `rm: cannot remove...` but this is okay
cd ..
mkdir build
cd build
cmake .. # there are still some warnings here
make -j
```




# Test测试
“common”文件夹中的代码有一些测试。 在build文件夹中，可以使用“common/test common”运行这些命令。有两种测试通常报错：

- OSQP -   解算器本身是不确定的，所以只要再次运行测试，它就会通过
- CASADI - 解算器在运行时加载库，有时在查找时遇到问题。如果失败也没关系，因为我们还没有使用这个解算器。





# 遥控手柄
我们使用罗技F310控制器。后面有个开关，应该在“X”位置。如果改变开关位置，控制器需要重新连接。另外，前面靠近模式按钮的LED应该关闭。
(https://www.amazon.com/Logitech-940-000110-Gamepad-F310/dp/B003VAHYQY)





# 仿真示例
模拟器的默认设置可以用`config/simulator-defaults.yaml`和`config/default-terrain.yaml`. 默认设置应该适合大多数用途，“default terrain”文件注释掉了如何添加网格、长方体和楼梯的示例。可以从地形文件中设置地板摩擦力。
    要启动模拟器，请先插入游戏杆，然后从“build”文件夹中运行“sim/sim”。选择“迷你猎豹”和“模拟器”，然后单击“开始”。在输出的某个地方，你应该看到

```
[GameController] Found 1 joystick
```


左面板允许您更改模拟器设置。最有用的设置是`simulation_speed`。当模拟器打开时,默认值已加载`imulator-defaults.yaml`。加载的设置文件必须与代码中定义的参数集完全相同。如果添加或删除参数，则必须重新编译`sim`。您可以随时保存和加载参数，但请注意，“使用弹簧阻尼器”设置将不会生效，除非您重新启动模拟器


中心面板允许您更改非特定于控制器的机器人设置。当迷你猎豹被选中,默认值从`mini-cheetah-defatuls.yaml`文件加载，你点击“开始”。如果您停止并启动模拟器，文件将被重新加载。加载的设置文件必须与代码中定义的参数集完全相同。如果添加或删除参数，则必须重新编译`sim`。目前，这些参数中的大多数不起作用，许多参数将被删除。最有用的设置是`cheater_mode`，它向机器人代码发送机器人的当前位置/方向/速度，以及“控制器”，它改变控制代码的运行频率。（“controller_dt”设置仍需测试）


右面板允许您更改特定于控制器的参数，称为`User Parameters`。如果你的控制代码不使用用户参数，它将被忽略。如果代码确实使用用户参数，则必须加载与代码所需参数匹配的用户配置参数，否则控制器将无法启动。


要启动robot控制代码，请运行指令`user/MIT_Controller/mit_ctrl m s`。`m`参数用于mini cheetah，`s`则表示它应该连接到模拟器。它使用共享内存与模拟器通信。模拟器应该开始运行，机器人应该移动到一个准备好的位置。在模拟器窗口的中间一列，将控制模式设置为10。一旦机器人停止移动，设置控制模式1。然后，将控制模式设置为4，机器人将开始小跑。

你可以用操纵杆带动机器人四处走动。你会看到两个机器人-灰色的一个是模拟中机器人的实际位置，红色的是我们的状态估计器对机器人位置的估计。打开“作弊器模式”将使估计位置等于实际位置。要调整模拟视图，可以在屏幕上单击并拖动并滚动。按住“t”可使模拟尽可能快地运行。按空格键启用“自由摄影机”模式。可以使用w、a、s、d、r、f键移动相机，然后单击并拖动以调整方向




# LCM通讯
我们使用LCM(https://lcm-proj.github.io/)将控制接口连接到实际的mini cheetah硬件上，并作为运行模拟器时的调试工具。
`make_types.sh`脚本运行LCM工具，为LCM数据类型生成C++头文件。当模拟器运行时，
`scripts/launch_lcm_spy.sh`运行LCM spy实用程序，该实用程序显示来自模拟器和控制器的详细信息。您可以单击数据流来绘制它们，这对调试很有帮助。还有一个叫做“lcm logger”的工具可以将lcm数据保存到文件中。



# 编写机器人控制器方法
想要添加您自己的机器人控制器，您应该在`Cheetah-Software/user`下添加一个文件夹，并将该文件夹添加到`user`文件夹中的`CMakeLists.txt文件`中。
`JPos_Controller`是一个非常简单的控制器的例子。`JPosUserParameters.h`文件有一个声明两个用户参数的示例，可以从模拟器界面调整这些参数，但使用用户参数是可选的。 `JPos_Controller.hpp`和 `JPos_Controller.cpp`文件是实际的控制器，它应该扩展到 `RobotController`
注意，在`JPos_Controller.hpp`文件中的 `getUserControlParameters`方法返回指向用户参数的指针。如果不使用用户参数，则`getUserControlParameters`应返回`nullptr`
最后，您的“main”函数必须与中的示例main函数类似


（1）控制器的 `runController` 方法将以1 kHz的频率自动调用。在这里，您可以访问以下内容：
- `_quadruped` :包含有关机器人的恒定参数（连杆长度、传动比、惯性…）。 `getHipLocation`函数返回“hip”在身体坐标系中的位置。x轴向前，y轴向左，z轴向上。腿是这样排列的

```
FRONT
1 0  RIGHT
3 2
BACK
```
- `_model` :   机器人的动力学模型。这可以用来计算正向运动学，雅可比等。。。
- `_legController`: 机器人腿的接口。这些数据在700赫兹左右与硬件同步。有多种方法可以控制腿部，所有控制器的结果都将相加。
- `commands[leg_id].tauFeedForward` :支腿扭矩，顺序是ab/ad，臀部，膝盖。
- `commands[leg_id].forceFeedForward` : 在臀部框架中，在脚（N）处施加力。（与车身骨架方向相同，原点为臀部）
- `commands[leg_id].qDes` :关节PD控制器的所需关节位置（弧度）。顺序是ab/ad，臀部，膝盖。`（0,0,0）`是指笔直向下的腿。
- `commands[leg_id].qdDes` : 关节PD控制器的期望关节速度（rad/sec）。
- `commands[leg_id].pDes, vDes` :笛卡尔PD控制器所需的脚位置/速度（米，臀部框架）
- `commands[leg_id].kpCartesian, kdCartesian, kpJoint, kdJoint` : PD控制器的增益（3x3矩阵）。仅使用对角线条目。
- `datas[leg_id].q` :腿部关节角度，即编码器（弧度）。顺序是ab/ad，臀部，膝盖。`（0,0,0）`是指笔直向下的腿。
- `datas[leg_id].qd` : 腿关节速度（弧度/秒）。与“q”的顺序相同
- `datas[leg_id].p`  : 脚笛卡尔位置，臀部框架。（与车身骨架方向相同，原点为臀部）
- `datas[leg_id].v`  :英尺笛卡尔速度，在臀部框架。
- `datas[leg_id].tau` : 由所有控制器组合估计电机转矩

（2）关节的PD控制器实际上在电机控制器上以40 kHz的频率运行
- `_stateEstimate, _stateEstimatorContainer` 所提供的状态估计器的结果和接口。如果提供机器人的接触状态（哪些脚接触地面），它将确定机器人在世界上的位置/速度。
- `_driverCommand` : 从游戏手柄的输入的指令接口
- `_controlParameters` : 机器人控制参数面板（GUI）中的参数输入接口
- `_visualizationData` : 将调试可视化添加到模拟器窗口的接口
- `_robotType` :迷你猎豹或猎豹3机器人

如果您想了解更多的工作原理，请查看`robot`文件夹
`RobotRunner`类实际上运行控制代码，并将其与`HardwareBridge` or `SimulationBridge`连接。
`rt`文件夹中的代码实际上与硬件交互



# 未完成事项
- 验证是否可以在仿真/机器人硬件中更改控制器更新率
- 使用状态估计器（如果你不想要的话就禁用它）
- 使用动力学
- 腿的最终更新率
- 其他人会使用无线遥控器吗？如果是，请将其添加到机器人控制器。
- 安全检查
- 可视化数据-并不是所有的东西都实现了，它们不适用于机器人


# 在真实和仿真模式下机器人上运行
在机器人上运行与运行模拟器非常相似。你仍然有游戏板、用户参数和机器人参数。在模拟窗口中，您将只看到机器人的状态估计，而作弊模式将不起作用。当前调试可视化不起作用。




