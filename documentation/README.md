## Cheetah-Software
此存储库包含Robot和仿真软件项目。有关入门指南，请参阅documentation文件夹。


## 主要文件说明：
“common”文件夹包含带有动力学和实用程序的公用库

“resources”文件夹将包含数据文件，如用于可视化的机器人的CAD

"robot"文件夹将包含robot程序

"sim"文件夹将包含模拟程序。它是唯一依赖QT的程序。

" third-party"文件夹将包含我们修改过的*small*第三方库。这应该只是猎豹3的libsoem，Pat曾经修改过它。

## Build 编译步骤
To build all code:
```
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j4
```

如果您正在计算机上构建要复制到mini cheetah的代码，则必须将cmake命令替换为
```
cmake -DMINI_CHEETAH_BUILD=TRUE
```
否则就没办法正常运行了。如果你在迷你猎豹计算机（upboard）建立迷你猎豹代码，你不需要这样做。

这个编译过程会编译common library, robot code, and simulator三个文件夹
如果你仅仅更改了robot code，你可以仅仅再一次运行指令`make -j4`
如果你更改了LCM types，你要运行指令 `cmake ..; make -j4`，这会自动的运行 `make_types.sh`.


To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.

Part of this build process will automatically download the gtest software testing framework and sets it up. 
After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  
Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```
## Run simulator
To run the simulator:
1. Open the control board
```
./sim/sim
```
2. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/JPos_Controller/jpos_ctrl 3 s
```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot

## Run Mini cheetah
1. Create build folder `mkdir mc-build`
2. Build as mini cheetah executable `cd mc-build; cmake -DMINI_CHEETAH_BUILD=TRUE ..; make -j`
3. Connect to mini cheetah over ethernet, verify you can ssh in
4. Copy program to mini cheetah with `../scripts/send_to_mini_cheetah.sh`
5. ssh into the mini cheetah `ssh user@10.0.0.34`
6. Enter the robot program folder `cd robot-software-....`
7. Run robot code `./run_mc.sh` 



## Dependencies:
- Qt 5.10 - https://www.qt.io/download-qt-installer
- LCM - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- Eigen - http://eigen.tuxfamily.org
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

To use Ipopt, use CMake Ipopt option. Ex) cmake -DIPOPT_OPTION=ON ..
