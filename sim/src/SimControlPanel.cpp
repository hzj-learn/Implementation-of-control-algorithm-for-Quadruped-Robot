#include "SimControlPanel.h"
#include <ControlParameters/ControlParameters.h>
#include <QFileDialog>
#include <QMessageBox>
#include <ParamHandler.hpp>
#include <leg_control_command_lcmt.hpp>
#include "ui_SimControlPanel.h"
#include "JoystickTest.h"


/*!
 * 显示带有给定文本的错误消息框
 */
static void createErrorMessage(const std::string& text) {
  QMessageBox mb;
  mb.setText(QString(text.c_str()));
  mb.exec();
}

/*!
 * 在qtable中显示控制参数。
 */
static void updateQtableWithParameters(ControlParameters& params,
                                       QTableWidget& table) 
{
  table.setRowCount((s32)params.collection._map.size());
  table.setColumnCount(2);
  s32 i = 0;
  for (auto& kv : params.collection._map) 
 {
    (void)kv;
    for (s32 col = 0; col < 2; col++) 
    {
      QTableWidgetItem* cell = table.item(i, col);
      if (!cell) 
      {
        cell = new QTableWidgetItem;
        table.setItem(i, col, cell);
      }
    }
    table.item(i, 0)->setText(QString(kv.first.c_str()));
    table.item(i, 1)->setText(QString(kv.second->toString().c_str()));
    i++;
  }
}

std::string SimControlPanel::getDefaultUserParameterFileName() 
{
  std::string path = getConfigDirectoryPath() + DEFAULT_USER_FILE;
  ParamHandler paramHandler(path);

  if(!paramHandler.fileOpenedSuccessfully()) 
  {
    throw std::runtime_error("Could not open yaml file for default user parameter file: " + path);
  }

  std::string collectionName;
  if(!paramHandler.getString("__collection-name__", collectionName)) 
  {
    throw std::runtime_error("Could not find __collection-name__ parameter in default user parameter file");
  }

  if(collectionName != "user-parameter-file") 
  {
    throw std::runtime_error("default user parameter file has the wrong collection name, should be user-parameter-file");
  }

  std::string fileName;

  if(!paramHandler.getString("file_name", fileName)) 
  {
    throw std::runtime_error("default user parameter file does not have a parameter named file_name");
  }

  return fileName;
}
/*!
 * 初始化sim卡窗口
 */
SimControlPanel::SimControlPanel(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::SimControlPanel),
      _userParameters("user-parameters"),
      _terrainFileName(getConfigDirectoryPath() + DEFAULT_TERRAIN_FILE),
      _heightmapLCM(getLcmUrl(255)),
      _pointsLCM(getLcmUrl(255)),
      _indexmapLCM(getLcmUrl(255)),
      _ctrlVisionLCM(getLcmUrl(255)),
      _miniCheetahDebugLCM(getLcmUrl(255))
{
  ui->setupUi(this); // QT设置
  updateUiEnable();  // 根据需要启用/禁用按钮。
  updateTerrainLabel(); // 加载地形文件的显示名称
  _loadedUserSettings = true;//尝试加载默认用户设置。

  try 
  {
    _userParameters.defineAndInitializeFromYamlFile(getConfigDirectoryPath() + getDefaultUserParameterFileName());
  } catch (std::runtime_error& ex) 
  {
    _loadedUserSettings = false;
  }

  if(!_loadedUserSettings) 
  {
    printf("[SimControlPanel] Failed to load default user settings!\n");
    throw std::runtime_error("Failed to load default user settings!");
  }
  else 
  {
    // 如果加载成功，则在qtable中显示用户设置
    loadUserParameters(_userParameters);
  }

  // 负载模拟器参数
  printf("[SimControlPanel] Init simulator parameters...\n");
  _parameters.initializeFromYamlFile(getConfigDirectoryPath() +
                                     SIMULATOR_DEFAULT_PARAMETERS);
  if (!_parameters.isFullyInitialized()) {
    printf(
        "[ERROR] Simulator parameters are not fully initialized.  You forgot: "
        "\n%s\n",
        _parameters.generateUnitializedList().c_str());
    throw std::runtime_error("simulator not initialized");
  } else {
    printf("\tsim parameters are all good\n");
  }
  loadSimulationParameters(_parameters);

  _pointsLCM.subscribe("cf_pointcloud", &SimControlPanel::handlePointsLCM, this);
  _pointsLCMThread = std::thread(&SimControlPanel::pointsLCMThread, this); 

  _heightmapLCM.subscribe("local_heightmap", &SimControlPanel::handleHeightmapLCM, this);
  _heightmapLCMThread = std::thread(&SimControlPanel::heightmapLCMThread, this);

  _indexmapLCM.subscribe("traversability", &SimControlPanel::handleIndexmapLCM, this);
  _indexmapLCMThread = std::thread(&SimControlPanel::indexmapLCMThread, this);

  _ctrlVisionLCM.subscribe("velocity_cmd", &SimControlPanel::handleVelocityCMDLCM, this);
  _ctrlVisionLCM.subscribe("obstacle_visual", &SimControlPanel::handleObstacleLCM, this);
  _ctrlVisionLCMThread = std::thread(&SimControlPanel::ctrlVisionLCMThread, this);

  //订阅mc调试 
  _miniCheetahDebugLCM.subscribe("leg_control_data", &SimControlPanel::handleSpiDebug, this);
  _miniCheetahDebugLCMThread = std::thread([&](){
   for(;;)
     _miniCheetahDebugLCM.handle();
  });

}

void SimControlPanel::handleVelocityCMDLCM(const lcm::ReceiveBuffer* rbuf, 
    const std::string & chan,
    const velocity_visual_t* msg){
  (void)rbuf;
  (void)chan;
 
  if(_graphicsWindow){
    for(size_t i(0); i<3; ++i){
      _graphicsWindow->_vel_cmd_dir[i] = msg->vel_cmd[i];
      _graphicsWindow->_vel_cmd_pos[i] = msg->base_position[i];
    }
    _graphicsWindow->_vel_cmd_update = true;
  }
}

void SimControlPanel::handleObstacleLCM(const lcm::ReceiveBuffer* rbuf, 
    const std::string & chan,
    const obstacle_visual_t* msg){
  (void)rbuf;
  (void)chan;
 
  if(_graphicsWindow){
    _graphicsWindow->_obs_list.clear();
    Vec3<double> obs_loc;
    for(int i(0); i<msg->num_obs; ++i){
      obs_loc[0] = msg->location[i][0];
      obs_loc[1] = msg->location[i][1];
      obs_loc[2] = msg->location[i][2];

      _graphicsWindow->_obs_list.push_back(obs_loc);
    }
    _graphicsWindow->_obs_sigma = msg->sigma;
    _graphicsWindow->_obs_height = msg->height;

    _graphicsWindow->_obstacle_update = true;
  }

}
void SimControlPanel::handlePointsLCM(const lcm::ReceiveBuffer *rbuf,
    const std::string &chan,
                                      const rs_pointcloud_t*msg) {
  (void)rbuf;
  (void)chan;

  if(_graphicsWindow){
    for(size_t i(0); i<_graphicsWindow->_num_points; ++i){
      for(size_t j(0); j<3; ++j){
        _graphicsWindow->_points[i][j] = msg->pointlist[i][j];
      }
    }
    _graphicsWindow->_pointcloud_data_update = true;
  }
}

void SimControlPanel::handleIndexmapLCM(const lcm::ReceiveBuffer *rbuf,
                                      const std::string &chan,
                                      const traversability_map_t *msg) {
  (void)rbuf;
  (void)chan;

  if(_graphicsWindow){
    for(size_t i(0); i<_graphicsWindow->x_size; ++i){
      for(size_t j(0); j<_graphicsWindow->y_size; ++j){
        _graphicsWindow->_idx_map(i,j) = msg->map[i][j];
      }
    }
    _graphicsWindow->_indexmap_data_update = true;
  }
}


void SimControlPanel::handleHeightmapLCM(const lcm::ReceiveBuffer *rbuf,
                                      const std::string &chan,
                                      const heightmap_t *msg) {
  (void)rbuf;
  (void)chan;

  if(_graphicsWindow){
    for(size_t i(0); i<_graphicsWindow->x_size; ++i){
      for(size_t j(0); j<_graphicsWindow->y_size; ++j){
        _graphicsWindow->_map(i,j) = msg->map[i][j];
      }
    }
    _graphicsWindow->_pos[0] = msg->robot_loc[0];
    _graphicsWindow->_pos[1] = msg->robot_loc[1];
    _graphicsWindow->_pos[2] = msg->robot_loc[2];

    _graphicsWindow->_heightmap_data_update = true;
  }
}

void SimControlPanel::handleSpiDebug(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                     const leg_control_data_lcmt *msg) {
  (void)rbuf;
  (void)chan;
  MiniCheetahDebugData ddata;

  u32 idx = 0;
  for(u32 leg = 0; leg < 4; leg++) {
    for(u32 joint = 0; joint < 3; joint++) {
      ddata.p[leg][joint] = msg->q[idx];
      ddata.v[leg][joint] = msg->qd[idx];
      idx++;
    }
  }

  if(_mcDebugWindow.setDebugData(ddata)) {
    MiniCheetahDebugCommand cmd;
    _mcDebugWindow.getDebugCommand(cmd);

    leg_control_command_lcmt lcm_cmd;
    memset(&lcm_cmd, 0, sizeof(leg_control_command_lcmt));
    idx = 0;
    for(u32 leg = 0; leg < 4; leg++) {
      for(u32 joint = 0; joint < 3; joint++) {
        if(cmd.enable[leg][joint]) {
          lcm_cmd.q_des[idx] = cmd.qd[leg][joint];
          lcm_cmd.kp_joint[idx] = cmd.kp[leg][joint];
          lcm_cmd.kd_joint[idx] = cmd.kd[leg][joint];
        }
        idx++;
      }
    }

    _miniCheetahDebugLCM.publish("spi_debug_cmd", &lcm_cmd);
  }

}


SimControlPanel::~SimControlPanel() {
  delete _simulation;
  delete _interfaceTaskManager;
  delete _robotInterface;
  delete _graphicsWindow;
  delete ui;
}

/*!
 * 需要UI更新的外部通知
 */
void SimControlPanel::update_ui() {
  updateUiEnable();
}

/*!
 * 根据运行的内容根据需要启用/禁用按钮
 */
void SimControlPanel::updateUiEnable() {
  ui->startButton->setEnabled(!(isRunning() || isError()));
  ui->stopButton->setEnabled(isRunning() || isError());
  ui->joystickButton->setEnabled(isRunning() || isError());
  ui->robotTable->setEnabled(isRunning() || isError());
  ui->goHomeButton->setEnabled(isRunning() || isError());

  switch(_state) {
    case SimulationWindowState::STOPPED:
      ui->simulatorStateLabel->setText("Simulator State: Stopped");
      ui->stopButton->setText("Stop");
      break;
    case SimulationWindowState::RUNNING:
      ui->simulatorStateLabel->setText("Simulator State: Running");
      ui->stopButton->setText("Stop");
      break;
    case SimulationWindowState::ERROR:
      ui->simulatorStateLabel->setText("Simulator State: Error");
      ui->stopButton->setText("Reset Error");
      break;
  }

  if((isRunning() || isError()) && _simulation) {
    if(_simulation->isRobotConnected()) {
      ui->simulatorConnectedLabel->setText("Sim Connection: Yes");
    } else {
      ui->simulatorConnectedLabel->setText("Sim Connection: No");
    }
  } else {
    ui->simulatorConnectedLabel->setText("Sim Connection: N/A");
  }

}

/*!
 * 更新加载的地形文件标签的名称
 */
void SimControlPanel::updateTerrainLabel() {
  ui->terrainFileLabel->setText(QString(_terrainFileName.c_str()));
}

/*!
 * 模拟误差回调函数
 */
void SimControlPanel::errorCallback(std::string errorMessage) 
{
  _state = SimulationWindowState::ERROR;                   //1）转到错误状态
  updateUiEnable();                                        //2）更新用户界面
  createErrorMessage("Simulation Error\n" + errorMessage); //3）显示错误对话框
}

/*!
 * 开始模拟/机器人跑步按键
 */
void SimControlPanel::on_startButton_clicked() 
{
 /*（1）从ui控制界面获取机器人类型*/
  RobotType robotType;

  if (ui->cheetah3Button->isChecked())         //1）按下cheetah3按钮,选择cheetah3
  {
    robotType = RobotType::CHEETAH_3;
  } 
  else if (ui->miniCheetahButton->isChecked()) //2）按下cheetah mini按钮,选择cheetah mini
  {
    robotType = RobotType::MINI_CHEETAH;
  } 
  else                                         //3）既没按下cheetah3按钮，也没按下cheetah mini按钮 ,报错
  {
    createErrorMessage("Error: you must select a robot");
    return;
  }


 /*（2）获取是真实运行或者仿真的模式*/
  if (!ui->simulatorButton->isChecked() && !ui->robotButton->isChecked())   //1）既没有按下仿真界面，也没有按下真实界面，报错
  {
    createErrorMessage(
        "Error: you must select either robot or simulation mode");
    return;
  }
  _simulationMode = ui->simulatorButton->isChecked();                       //2）根据按下的按钮，设置仿真类型或者真实运行模式



/*（3）显示三围图形界面*/
  printf("[SimControlPanel] Initialize Graphics...\n");
  _graphicsWindow = new Graphics3D();       //1）初始化三维可视化窗口
  _graphicsWindow->show();                  //2）显示三维视图
  _graphicsWindow->resize(1280, 720);       //3）调整大小以保留大小对象


/*（4）根据模式，运行仿真或者真实机器人*/
//若是运行仿真模拟器
  if (_simulationMode) 
  {
    try 
    {
      //1）使用类Simulation初始化一个变量_simulation
      printf("[SimControlPanel] Initialize simulator...\n");
      _simulation = new Simulation(robotType, _graphicsWindow, _parameters, _userParameters,//这将允许模拟线程在状态发生变化时提示我们
        [this]()
      {
        QMetaObject::invokeMethod(this,"update_ui");
      });
      //2）加载仿真参数
      loadSimulationParameters(_simulation->getSimParams());    
      //3）加载机器人参数
      loadRobotParameters(_simulation->getRobotParams());       
      //4）加载地形参数
      printf("[SimControlParameter] Load terrain...\n");
      _simulation->loadTerrainFile(_terrainFileName);
    } 
    catch (std::exception& e) 
    {
      createErrorMessage("FATAL: Exception thrown during simulator setup\n" + std::string(e.what()));
      throw e;
    }

    //5）启动sim仿真线程【非常重要】
    _simThread = std::thread(
      //模拟功能
      [this]() 
      {
        //1、错误回调函数
        std::function<void(std::string)> error_function = [this](std::string str) 
        {
          //Qt将负责在UI事件循环中执行调用
          QMetaObject::invokeMethod(this, [=]() 
          {
            this->errorCallback(str);
          });
        };

        try 
        {
          //2、将IMU,SPI作弊数据传递到模拟器【非常重要！！！】
           //在当前线程中运行模拟器，直到设置了 running变量错了。如果需要，以每秒60帧的速度更新图形。在运行模拟期望速度
          _simulation->runAtSpeed(error_function);
        } 

        catch (std::exception &e) 
        {
          //3、捕获仿真是的异常
          error_function("Exception thrown in simulation thread: " + std::string(e.what()));
        }

      });


    //5）启用动画显示
    _graphicsWindow->setAnimating(true);
  } 

//若是运行真实机器人  
  else 
  {
    printf("[SimControlPanel] Init Robot Interface...\n");
    
    //1）创建一个周期任务管理器
    _interfaceTaskManager = new PeriodicTaskManager;  

    //2）创建一个机器人接口
    _robotInterface =
        new RobotInterface(robotType, _graphicsWindow, _interfaceTaskManager, _userParameters);
    
    //3）加载机器人接口参数
    loadRobotParameters(_robotInterface->getParams());

    //4）启动机器人接口
    _robotInterface->startInterface();
    
    //5）启用动画显示
    _graphicsWindow->setAnimating(true);
  }

/*（5）启动仿真界面*/
  _state = SimulationWindowState::RUNNING;

/*（6）更新用户界面*/
  updateUiEnable();
}

/*!
 * 停止当前运行的模拟或机器人连接
 */
void SimControlPanel::on_stopButton_clicked() {
  if (_simulation) {
    _simulation->stop();
    _simThread.join();
  } else {
    _robotInterface->stopInterface();
  }

  if (_graphicsWindow) {
    _graphicsWindow->setAnimating(false);
    _graphicsWindow->hide();
  }

  delete _interfaceTaskManager;
  delete _robotInterface;
  delete _simulation;
  delete _graphicsWindow;

  _simulation = nullptr;
  _graphicsWindow = nullptr;
  _robotInterface = nullptr;
  _interfaceTaskManager = nullptr;

  _state = SimulationWindowState::STOPPED;
  updateUiEnable();
}



/*!
 * 填充模拟器qtable参数
 */
void SimControlPanel::loadSimulationParameters(
    SimulatorControlParameters& params) {
  _ignoreTableCallbacks = true;
  updateQtableWithParameters(params, *ui->simulatorTable);
  _ignoreTableCallbacks = false;
}

/*!
 * 填充robot qtable参数
 */
void SimControlPanel::loadRobotParameters(RobotControlParameters& params) {
  _ignoreTableCallbacks = true;
  updateQtableWithParameters(params, *ui->robotTable);
  _ignoreTableCallbacks = false;
}

/*!
 * 填充robot qtable参数
 */
void SimControlPanel::loadUserParameters(ControlParameters& params) {
  _ignoreTableCallbacks = true;
  updateQtableWithParameters(params, *ui->userControlTable);
  _ignoreTableCallbacks = false;
}

/*!
 *如果连接了新的操纵杆，尝试重置操纵杆
 */
void SimControlPanel::on_joystickButton_clicked() {
  if(isRunning()) {
    _graphicsWindow->resetGameController();
    JoystickTestWindow* window = new JoystickTestWindow(_graphicsWindow->getGameController());
    window->exec();
    delete window;
  }
}

void SimControlPanel::on_driverButton_clicked() {
  _mcDebugWindow.show();
}

/*!
 * 响应模拟器表中的更改。
 */
void SimControlPanel::on_simulatorTable_cellChanged(int row, int column) {
  if (_ignoreTableCallbacks) return;

  // 我们只允许值更改，这些值在第1列中
  if (column != 1) {
    return;
  }

  // 获取参数的名称。。。。
  auto cell = ui->simulatorTable->item(row, 0);
  std::string cellName = cell->text().toStdString();

  if (cellName == "") {
    return;
  }

  //获取参数 
  auto& parameter = _parameters.collection.lookup(cellName);
  ControlParameterValueKind kind = parameter._kind;
  ControlParameterValue oldValue = parameter.get(kind);

  bool success = true;

  // 尝试设置，基于字符串。
  try {
    parameter.setFromString(
        ui->simulatorTable->item(row, 1)->text().toStdString());
  } catch (std::exception& e) {
    success = false;
  }

  //如果失败（错误的用户输入字符串），请还原为旧值 
  if (!success) {
    printf("[ERROR] invalid data, restoring old data!\n");
    //设置参数值 
    parameter.set(oldValue, kind);

    assert(!_ignoreTableCallbacks);

    //手动修复表
    _ignoreTableCallbacks = true;
    ui->simulatorTable->item(row, 1)->setText(
        QString(_parameters.collection.lookup(cellName).toString().c_str()));
    _ignoreTableCallbacks = false;
  } else {
    //此更新“重写”表中的值。如果它是一个整数，它会杀死任何十进制数。如果它是一个浮点数，如果需要的话，它会加上科学符号。
    _ignoreTableCallbacks = true;
    ui->simulatorTable->item(row, 1)->setText(
        QString(parameter.toString().c_str()));
    _ignoreTableCallbacks = false;
  }
}

/*!
 * 将模拟配置保存到文件
 */
void SimControlPanel::on_saveSimulatorButton_clicked() {
  QString fileName = QFileDialog::getSaveFileName(
      nullptr, ("Save Simulator Table Values"), "../config", "All Files (*)");
  if (fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }

  _parameters.lockMutex();
  _parameters.writeToYamlFile(fileName.toStdString());
  _parameters.unlockMutex();
}

/*!
 * 从文件加载模拟配置
 */
void SimControlPanel::on_loadSimulatorButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(
      nullptr, ("Load Simulator Table Values"), "../config", "All Files (*)");
  if (fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  };
  _parameters.collection.clearAllSet();
  _parameters.initializeFromYamlFile(fileName.toStdString());
  if (!_parameters.collection.checkIfAllSet()) {
    printf(
        "new settings file %s doesn't contain the following simulator "
        "parameters:\n%s\n",
        fileName.toStdString().c_str(),
        _parameters.generateUnitializedList().c_str());
    throw std::runtime_error("bad new settings file");
  }
  loadSimulationParameters(_parameters);
  _parameters.unlockMutex();
}

void SimControlPanel::on_robotTable_cellChanged(int row, int column) {
  if (_ignoreTableCallbacks) return;
  if (column != 1) {
    return;
  }

  auto cell = ui->robotTable->item(row, 0);
  std::string cellName = cell->text().toStdString();

  if (cellName == "") {
    return;
  }

  auto& parameter = (_simulationMode ? _simulation->getRobotParams()
                                     : _robotInterface->getParams())
                        .collection.lookup(cellName);
  ControlParameterValueKind kind = parameter._kind;
  ControlParameterValue oldValue = parameter.get(kind);

  bool success = true;

  try {
    parameter.setFromString(ui->robotTable->item(row, 1)->text().toStdString());
  } catch (std::exception& e) {
    success = false;
  }

  if (!success) {
    printf("[ERROR] invalid data, restoring old data!\n");
    parameter.set(oldValue, kind);

    assert(!_ignoreTableCallbacks);

    _ignoreTableCallbacks = true;
    ui->robotTable->item(row, 1)->setText(
        QString(parameter
                    .toString()
                    .c_str()));
    _ignoreTableCallbacks = false;
  } else {
    if (_simulationMode) {
      if (_simulation->isRobotConnected()) {
        _simulation->sendControlParameter(
            cellName, parameter.get(parameter._kind), parameter._kind, false);
      }

      _ignoreTableCallbacks = true;
      ui->robotTable->item(row, 1)->setText(
          QString(parameter.toString().c_str()));
      _ignoreTableCallbacks = false;
    } else {
      _robotInterface->sendControlParameter(
          cellName, parameter.get(parameter._kind), parameter._kind, false);
    }
  }
}

void SimControlPanel::on_saveRobotButton_clicked() {
  QString fileName = QFileDialog::getSaveFileName(
      nullptr, ("Save Robot Table Values"), "../config", "All Files (*)");
  if (fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }
  _simulation->getRobotParams().writeToYamlFile(fileName.toStdString());
}

void SimControlPanel::on_loadRobotButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(
      nullptr, ("Load Quadruped Table Values"), "../config", "All Files (*)");
  if (fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }

  if (_simulationMode) {
    _simulation->getRobotParams().lockMutex();
    _simulation->getRobotParams().collection.clearAllSet();
    _simulation->getRobotParams().initializeFromYamlFile(
        fileName.toStdString());
    if (!_simulation->getRobotParams().collection.checkIfAllSet()) {
      printf(
          "new settings file %s doesn't contain the following robot "
          "parameters:\n%s\n",
          fileName.toStdString().c_str(),
          _simulation->getRobotParams().generateUnitializedList().c_str());
      throw std::runtime_error("bad new settings file");
    }
    loadRobotParameters(_simulation->getRobotParams());

    if (_simulation->isRobotConnected()) {
      for (auto& kv : _simulation->getRobotParams().collection._map) {
        _simulation->sendControlParameter(
            kv.first, kv.second->get(kv.second->_kind), kv.second->_kind, false);
      }
    }
    _simulation->getRobotParams().unlockMutex();
  } else {
    _robotInterface->getParams().lockMutex();
    _robotInterface->getParams().collection.clearAllSet();
    _robotInterface->getParams().initializeFromYamlFile(fileName.toStdString());
    if (!_robotInterface->getParams().collection.checkIfAllSet()) {
      printf(
          "new settings file %s doesn't contain the following robot "
          "parameters:\n%s\n",
          fileName.toStdString().c_str(),
          _robotInterface->getParams().generateUnitializedList().c_str());
      throw std::runtime_error("bad new settings file");
    }
    loadRobotParameters(_robotInterface->getParams());

    for (auto& kv : _robotInterface->getParams().collection._map) {
      _robotInterface->sendControlParameter(
          kv.first, kv.second->get(kv.second->_kind), kv.second->_kind, false);
    }

    _robotInterface->getParams().unlockMutex();
  }
}

void SimControlPanel::on_setTerrainButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(
      nullptr, ("Load Terrain Definition"), "../config", "All Files (*)");
  if (fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }

  _terrainFileName = fileName.toStdString();
  updateTerrainLabel();
}

void SimControlPanel::on_userControlTable_cellChanged(int row, int column) {
  if (_ignoreTableCallbacks) return;
  if (column != 1) {
    return;
  }

  auto cell = ui->userControlTable->item(row, 0);
  std::string cellName = cell->text().toStdString();

  if (cellName == "") {
    return;
  }

  auto& parameter = _userParameters.collection.lookup(cellName);

  ControlParameterValueKind kind = parameter._kind;
  ControlParameterValue oldValue = parameter.get(kind);

  bool success = true;

  try {
    parameter.setFromString(ui->userControlTable->item(row, 1)->text().toStdString());
  } catch (std::exception& e) {
    success = false;
  }

  if (!success) {
    printf("[ERROR] invalid data, restoring old data!\n");
    parameter.set(oldValue, kind);

    assert(!_ignoreTableCallbacks);

    _ignoreTableCallbacks = true;
    ui->userControlTable->item(row, 1)->setText(
        QString(_userParameters
                    .collection.lookup(cellName)
                    .toString()
                    .c_str()));
    _ignoreTableCallbacks = false;
  } else {
    if(isRunning() || isError()) {
      if (_simulationMode) {
        if (_simulation && _simulation->isRobotConnected()) {
          _simulation->sendControlParameter(
              cellName, parameter.get(parameter._kind), parameter._kind, true);
        }

        _ignoreTableCallbacks = true;
        ui->userControlTable->item(row, 1)->setText(
            QString(parameter.toString().c_str()));
        _ignoreTableCallbacks = false;
      } else {
      _robotInterface->sendControlParameter(
          cellName, parameter.get(parameter._kind), parameter._kind, true);
      }
    }
  }
}

void SimControlPanel::on_loadUserButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(
      nullptr, ("Load User Table Values"), "../config", "All Files (*)");
  if (fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }

  _userParameters.lockMutex();
  _userParameters.collection.deleteAll();
  _userParameters.defineAndInitializeFromYamlFile(
      fileName.toStdString());
  loadUserParameters(_userParameters);
  _userParameters.unlockMutex();
  _loadedUserSettings = true;

  if(isRunning() || isError()) {
    if (_simulationMode) {
      if (_simulation && _simulation->isRobotConnected()) {
        for (auto& kv : _userParameters.collection._map) {
          _simulation->sendControlParameter(
              kv.first, kv.second->get(kv.second->_kind), kv.second->_kind, true);
        }
      }
    } else {
    for (auto& kv : _userParameters.collection._map) {
      _robotInterface->sendControlParameter(
          kv.first, kv.second->get(kv.second->_kind), kv.second->_kind, true);
    }

    }
  }
}

void SimControlPanel::on_saveUserButton_clicked() {
  QString fileName = QFileDialog::getSaveFileName(
      nullptr, ("Save User Table Values"), "../config", "All Files (*)");
  if (fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }
  _userParameters.writeToYamlFile(fileName.toStdString());
}

void SimControlPanel::on_goHomeButton_clicked() {
  printf("go home\n");
  FBModelState<double> homeState;
  homeState.bodyOrientation << 1, 0, 0, 0;
  homeState.bodyPosition = Vec3<double>(0, 0, 0.4);
  homeState.bodyVelocity = SVec<double>::Zero();
  homeState.q = DVec<double>(12);
  homeState.q << -0.05, -0.8, 1.7, 0.05, -0.8, 1.7, -0.05, -0.8, 1.7, 0.05, -0.8, 1.7;
  homeState.qd = homeState.q;

  _simulation->setRobotState(homeState);
}

void SimControlPanel::on_kickButton_clicked() {
  //浮基速度： 
  SVec<double> kickVelocity;
  kickVelocity << ui->kickAngularX->text().toDouble(),
      ui->kickAngularY->text().toDouble(), ui->kickAngularZ->text().toDouble(),
      ui->kickLinearX->text().toDouble(), ui->kickLinearY->text().toDouble(),
      ui->kickLinearZ->text().toDouble();

  FBModelState<double> state = _simulation->getRobotState();
  state.bodyVelocity += kickVelocity;
  _simulation->setRobotState(state);
}
