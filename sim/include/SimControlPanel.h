/*!
 * @file SimControlPanel.h
 * @brief QT gui for the simulator
 */

#ifndef SIMCONTROLPANEL_H
#define SIMCONTROLPANEL_H

#include <QMainWindow>
#include <thread>
#include "ControlParameters/SimulatorParameters.h"
#include "Graphics3D.h"
#include "RobotInterface.h"
#include "Simulation.h"

#define DEFAULT_TERRAIN_FILE "/default-terrain.yaml"
#define DEFAULT_USER_FILE "/default-user-parameters-file.yaml"

#include <lcm-cpp.hpp>
#include <src/MiniCheetahDebug.h>
#include <leg_control_data_lcmt.hpp>
#include "rs_pointcloud_t.hpp"
#include "heightmap_t.hpp"
#include "traversability_map_t.hpp"
#include "obstacle_visual_t.hpp"
#include "velocity_visual_t.hpp"

namespace Ui {
class SimControlPanel;
}

enum class SimulationWindowState {
  STOPPED,
  RUNNING,
  ERROR
};

class SimControlPanel : public QMainWindow {
  Q_OBJECT

 public:
  explicit SimControlPanel(QWidget* parent = nullptr);
  ~SimControlPanel();



public slots:
  void update_ui();
  void errorCallback(std::string errorMessage);

 private slots:

  void on_startButton_clicked();                          //（1）单击开始模拟/机器人跑步按键

  void on_stopButton_clicked();                           //（2） 单击停止当前运行模拟器或机器人连接按键

  void on_joystickButton_clicked();                       //（3）单击“操纵杆”按钮

  void on_driverButton_clicked();                         //（4）单击在驱动器上按钮

  void on_simulatorTable_cellChanged(int row, int column);//（5）单击仿真表对应的行列项

  void on_saveSimulatorButton_clicked();                  //（6）单击保存仿真参数按钮

  void on_loadSimulatorButton_clicked();                  //（7）单击加载仿真器参数按钮

  void on_robotTable_cellChanged(int row, int column);    //（8）单击机器人表对应的行列项

  void on_saveRobotButton_clicked();                      //（9）单击保存机器人参数按钮

  void on_loadRobotButton_clicked();                      //（10）单击加载机器人参数按钮

  void on_goHomeButton_clicked();                         //（11）单击返回主菜单按钮

  void on_kickButton_clicked();                           //（12）单击kick按钮

  void on_userControlTable_cellChanged(int row, int column);//（13）单击用户控制表对应的行列项

  void on_saveUserButton_clicked();                       //（14）单击保存用户参数按钮

  void on_loadUserButton_clicked();                       //（15）单击加载用户参数按钮

  void on_setTerrainButton_clicked();                     //（16）单击设置遥控器按钮

  void on_hide_floor_checkbox_toggled(bool x)             //（17）单击隐藏层——复选框——切换按钮
  {
    if(_graphicsWindow) 
    {
      _graphicsWindow->setHideFloor(x);
    }
  }

  void on_hide_robot_checkbox_toggled(bool x)             //（18）单击隐藏机器人层——复选框——切换按钮
  {
    if(_graphicsWindow) {
      _graphicsWindow->setHideRobot(x);
    }
  }




  void updateTerrainLabel();//

  void loadSimulationParameters(SimulatorControlParameters& params);//（）
  
  void loadRobotParameters(RobotControlParameters& params);//（）
 
  void loadUserParameters(ControlParameters& params);//（）

 private:

  std::string getDefaultUserParameterFileName();
  void updateUiEnable();
  bool isStopped() 
  {
    return _state == SimulationWindowState::STOPPED;
  }

  bool isError() 
  {
    return _state == SimulationWindowState::ERROR;
  }

  bool isRunning() 
  {
    return _state == SimulationWindowState::RUNNING;
  }


  std::thread _simThread;
  //bool _started = false;
  SimulationWindowState _state = SimulationWindowState::STOPPED;
  Ui::SimControlPanel* ui;
  Simulation* _simulation = nullptr;
  PeriodicTaskManager* _interfaceTaskManager = nullptr;
  RobotInterface* _robotInterface = nullptr;
  Graphics3D* _graphicsWindow = nullptr;
  SimulatorControlParameters _parameters;
  ControlParameters _userParameters;
  bool _simulationMode = false;
  bool _firstStart = true;
  bool _ignoreTableCallbacks = false;
  bool _loadedUserSettings = false;
  std::string _terrainFileName;

  // 视觉数据绘制
  void handleHeightmapLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
      const heightmap_t* msg);
  void heightmapLCMThread() { while (true) { _heightmapLCM.handle(); } }

  void handlePointsLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
      const rs_pointcloud_t* msg);
  void pointsLCMThread() { while (true) { _pointsLCM.handle(); } }

  void handleIndexmapLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
      const traversability_map_t* msg);
  void indexmapLCMThread() { while (true) { _indexmapLCM.handle(); } }

  void handleObstacleLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
      const obstacle_visual_t* msg);
  void handleVelocityCMDLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, 
      const velocity_visual_t* msg);
  void ctrlVisionLCMThread(){ while(true){ _ctrlVisionLCM.handle();  } }

  void handleSpiDebug(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const leg_control_data_lcmt* msg);

  lcm::LCM _heightmapLCM;
  lcm::LCM _pointsLCM;
  lcm::LCM _indexmapLCM;
  lcm::LCM _ctrlVisionLCM;
  lcm::LCM _miniCheetahDebugLCM;

  std::thread _pointsLCMThread;
  std::thread _heightmapLCMThread;
  std::thread _indexmapLCMThread;
  std::thread _ctrlVisionLCMThread;
  std::thread _miniCheetahDebugLCMThread;

  MiniCheetahDebug _mcDebugWindow;

};

#endif  // SIMCONTROLPANEL_H
