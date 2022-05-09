# Running Mini Cheetah的运行过程

（1）开启电源：从顶部看，迷你猎豹的两个开关在前面。中央开关为电机电源，右开关为计算机电源。把开关拨到机器人的后面，它就会打开。打开电脑电源。启动计算机大约需要2分钟。

（2）远程链接：用“SSH指令连接到计算机中用户 `ssh user@10.0.0.34`. 密码是通常的实验室密码。您必须将计算机的IP地址设置为类似“10.0.0.xxx”的值才能使用。

（3）运行LCM通讯：接下来，为LCM设置计算机。在scripts文件夹中，运行`./config_network_lcm.sh -I `指令后面是你要连接的网络适配器的名称。如果需要，可以编辑脚本以添加特定网络适配器的快捷方式

（4）编译可在mini cheetah上运行的代码：
- `mkdir mc-build && cd mc-build`
- `cmake -DMINI_CHEETAH_BUILD=TRUE ..`
- `make -j`

（5）把代码复制到机器人上：运行指令`../scripts/send_to_mini_cheetah.sh user/yourController/your_controller`系统将提示您输入迷你猎豹的密码。


（6）运行LCM: 如果要打开LCM spy，可以通过运行指令`../script/launch_lcm_spy.sh`. 如果收到有关Java的错误，请尝试运行`../scripts/launch_lcm_spy_jdk_fix.sh`，它修改了启动参数以支持较新版本的JVM。

(7)如果你连接到机器人顶部的以太网线:在mini cheetah上，您将在主目录中找到一个文件夹，名为 `robot-software`和日期。此文件夹中有一个配置lcm脚本的副本。运行`./configure_network_lcm.sh mc-top`

(8)运行robot代码:请进入build文件夹并运行脚本`(sudo) nohup mc_run.sh`. mini cheetah程序将一直等到接口打开并在minicheatah机器人模式下启动。


机器人控制器当前使用：
（1）- 状态估计（仅限定向），使用类`_stateEstimate`访问 
（2）- 腿部控制（转矩前馈，力前馈，PD控制笛卡尔坐标，PD控制关节），通过类`LegCommand`访问。请注意，leg命令在每次迭代中都为零。
（3）- 腿部数据（关节位置/速度、髋关节框架中的笛卡尔位置/速度、髋关节框架中的雅可比矩阵）
（4）- 游戏手柄数据
（5）- 主可视化（这只是一个cheetah模型，其余的还在实现中）
（6）- 控制参数（在模拟器gui中设置）
（7）- 配置文件（使用“THIS_COM”，或指向生成文件夹的相对路径）


它不适用于：
- 全状态估计器（位置、速度）
- 全可视化数据
- 在机器人上运行时的作弊器状态

当前LCM流
- 原始spi数据
- 原始矢量数据
- 游戏手柄指令
- 主要的可视化

当前缺少LCM流
- 通用腿部数据
- 状态估计数据



