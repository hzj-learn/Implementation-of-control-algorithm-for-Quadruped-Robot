/*! @file GameController.cpp
 *  @brief 读取Logitech F310游戏控制器的代码
 *创建要发送到robot控制器的DriverCommand对象
 *用于开发模拟器和机器人控制模式
 *注意：因为QT很奇怪，updateDriverCommand必须从
 *QT事件。在另一个线程中运行它会导致它无法工作。作为一个
 *结果，这只在QTObject的update方法中调用时有效
 */

#include "GameController.h"

#include <QtCore/QObject>
#include <QtGamepad/QGamepad>

/*!
 *默认情况下，游戏控制器选择“第一个”操纵杆，打印
 *警告：如果Linux上有多个操纵杆，则为/dev/input/js0 if
 *找不到操纵杆，它将打印错误消息，并返回零。
 *以后可以使用findNewController更改/添加操纵杆
 */
GameController::GameController(QObject *parent) : QObject(parent) {
  findNewController();
}

/*!
 *重新运行操纵杆查找代码以选择“第一个”操纵杆。这可能是
 *如果模拟器在没有操纵杆的情况下启动，则用于设置操纵杆
 *接通电源
 */
void GameController::findNewController() {
  delete _qGamepad;
  _qGamepad = nullptr; //万一这样不行！

  printf("[Gamepad] Searching for gamepads, please ignore \"Device discovery cannot open device\" errors\n");
  auto gamepadList = QGamepadManager::instance()->connectedGamepads();
  printf("[Gamepad] Done searching for gamepads.\n");
  if (gamepadList.empty()) {
    printf(
        "[ERROR: GameController] No controller was connected! All joystick "
        "commands will be zero!\n");
  } else {
    if (gamepadList.size() > 1) {
      printf(
          "[ERROR: GameController] There are %d joysticks connected.  Using "
          "the first one.\n",
          gamepadList.size());
    } else {
      printf("[GameController] Found 1 joystick\n");
    }

    _qGamepad = new QGamepad(*gamepadList.begin());
  }
}

/*!
 *用当前操纵杆状态覆盖驱动器命令。如果没有
 *操纵杆，发送0
 *TODO:如果操纵杆被拔下会发生什么？
 */
void GameController::updateGamepadCommand(GamepadCommand &gamepadCommand) {
  if (_qGamepad) {
    gamepadCommand.leftBumper = _qGamepad->buttonL1();
    gamepadCommand.rightBumper = _qGamepad->buttonR1();
    gamepadCommand.leftTriggerButton = _qGamepad->buttonL2() != 0.;
    gamepadCommand.rightTriggerButton = _qGamepad->buttonR2() != 0.;
    gamepadCommand.back = _qGamepad->buttonSelect();
    gamepadCommand.start = _qGamepad->buttonStart();
    gamepadCommand.a = _qGamepad->buttonA();
    gamepadCommand.b = _qGamepad->buttonB();
    gamepadCommand.x = _qGamepad->buttonX();
    gamepadCommand.y = _qGamepad->buttonY();
    gamepadCommand.leftStickButton = _qGamepad->buttonL3();
    gamepadCommand.rightStickButton = _qGamepad->buttonR3();
    gamepadCommand.leftTriggerAnalog = (float)_qGamepad->buttonL2();
    gamepadCommand.rightTriggerAnalog = (float)_qGamepad->buttonR2();
    gamepadCommand.leftStickAnalog =
        Vec2<float>(_qGamepad->axisLeftX(), -_qGamepad->axisLeftY());
    gamepadCommand.rightStickAnalog =
        Vec2<float>(_qGamepad->axisRightX(), -_qGamepad->axisRightY());
  } else {
    gamepadCommand.zero();  // 没有操纵杆，全部归零
  }

  // printf("%s\n", gamepadCommand.toString().c_str());
}

GameController::~GameController() { delete _qGamepad; }