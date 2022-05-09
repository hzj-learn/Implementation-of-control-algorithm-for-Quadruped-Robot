/*! @file GameController.h
 *  @brief 读取Logitech F310游戏控制器的代码
*创建要发送到robot控制器的DriverCommand对象
*用于开发模拟器和机器人控制模式
 */

#ifndef PROJECT_GAMECONTROLLER_H
#define PROJECT_GAMECONTROLLER_H

#include "SimUtilities/GamepadCommand.h"

#include <QtCore/QObject>

class QGamepad;  //出于未知原因，此处包括<QtGamepad/QGamepad>

//使编译变得非常缓慢
class GameController : public QObject {
  Q_OBJECT
 public:
  explicit GameController(QObject *parent = 0);
  void updateGamepadCommand(GamepadCommand &gamepadCommand);
  void findNewController();
  ~GameController();

 private:
  QGamepad *_qGamepad = nullptr;
};

#endif  // PROJECT_GAMECONTROLLER_H
