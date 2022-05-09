#include "Controllers/GaitScheduler.h"

/*=========================== Gait Data（步态数据） ===============================*/
/**
 *
 */
template <typename T>
void GaitData<T>::zero() 
{
  // 停止任何步态转换
  _nextGait = _currentGait;

  //一般步态描述
  periodTimeNominal = 0.0;      // 总周期
  initialPhase = 0.0;           // 初始相位为0
  switchingPhaseNominal = 0.0;  // 转换触点的法相位

  //步态使能
  gaitEnabled = Eigen::Vector4i::Zero();  // 启用增益控制腿

  //每只脚启用标志
  periodTime = Vec4<T>::Zero();           // 整体步态周期时间
  timeStance = Vec4<T>::Zero();           // 总站姿时间
  timeSwing = Vec4<T>::Zero();            // 总摆动时间
  timeStanceRemaining = Vec4<T>::Zero();  // 站姿剩余时间
  timeSwingRemaining = Vec4<T>::Zero();   // 剩余摆动时间

  //基于相位的描述符
  switchingPhase = Vec4<T>::Zero();  // 切换至摆动的相位
  phaseVariable = Vec4<T>::Zero();   // 每只脚的整体步态阶段
  phaseOffset = Vec4<T>::Zero();     // 发步态相位设置为0
  phaseScale = Vec4<T>::Zero();      // 相对于变量的相位标度
  phaseStance = Vec4<T>::Zero();     // 站姿亚相
  phaseSwing = Vec4<T>::Zero();      // 摆动副相

  //计划联系状态
  contactStateScheduled = Eigen::Vector4i::Zero();  //脚的接触状态
  contactStatePrev =
      Eigen::Vector4i::Zero();                      //脚以前的接触状态
  touchdownScheduled = Eigen::Vector4i::Zero();     //预定着陆标志
  liftoffScheduled = Eigen::Vector4i::Zero();       //预定升空标志

  //在事件中脚在世界范围内的位置
  posFootTouchdownWorld =
      Mat34<T>::Zero();  //预定起飞时的脚部位置
  posFootLiftoffWorld =
      Mat34<T>::Zero();  // 按计划着陆时的脚部位置
}

template struct GaitData<double>;
template struct GaitData<float>;


/*========================= Gait Scheduler（步态规划器）============================*/

/**
 * 功能：构造函数，设置基本站立步态
 */
template <typename T>
GaitScheduler<T>::GaitScheduler() 
{
  initialize();
}



/**
 * 功能：初始化步态数据函数
 * 默认创建站立的步态
 */
template <typename T>
void GaitScheduler<T>::initialize() 
{
  std::cout << "[GAIT] Initialize Gait Scheduler" << std::endl;

  //（1）开始步态类型设置为站立状态，因为我们用这个最多
  gaitData._currentGait = GaitType::STAND;

  //（2）将所有步态数据归零
  gaitData.zero();

  //（3）从标称初始值创建步态
  createGait();
  period_time_natural = gaitData.periodTimeNominal;
  switching_phase_natural = gaitData.switchingPhaseNominal;
}

/**
 * 功能：周期运行的相位控制器【选择步态】+【支撑相或腾空相步态调度相位增量迭代】函数
 */
template <typename T>
void GaitScheduler<T>::step() 
{
  ///////*（A）判断是否请求新步态*/////////////////////////////////////////////////////////////////
  if (gaitData._currentGait != gaitData._nextGait) //如果当前步态类型和下一个步态类型不一致
  {
    std::cout << "[GAIT] Transitioning gait from " << gaitData.gaitName
              << " to ";
    createGait();                                         //（1）重新建立一个新步态
    std::cout << gaitData.gaitName << "\n" << std::endl;
    gaitData._currentGait = gaitData._nextGait;           //（2）进行步态迭代，即把下个步态类型传给当前步态类型
  }

  ///////*（B）判断四条腿上是否开启了步态调度器*///////////////////////////////////////////////////////////
  for (int foot = 0; foot < 4; foot++)    
  {
    /*如果有开启步态调度器*/
    if (gaitData.gaitEnabled(foot) == 1) 
    {
      //（1）相位随着时间自增
      if (gaitData._currentGait == GaitType::STAND) // 在待机模式下不要增加相位，STAND就是站立不动视为待机
      {
        dphase = 0.0;  
      }
      else                                          //在非待机（工作）模式, 基于单调时间的相位增量
      {
        //这条是基于周期相位变化
        //公式：该腿的相位=该腿的相位刻度*（迭代周期/总周期时间）
        dphase = gaitData.phaseScale(foot) * (dt / gaitData.periodTimeNominal);
      }

      //（2）找到每只脚的当前相位
      //公式：每只脚的当前相位=上一次相位+本次的相位增量
      gaitData.phaseVariable(foot) =
          fmod((gaitData.phaseVariable(foot) + dphase), 1);

      //（3）计算支撑/摆动腿状态相关参数

      /*若处于支撑相阶段，计算支撑向步态相关参数*/
      if (gaitData.phaseVariable(foot) <= gaitData.switchingPhase(foot))   //腿现在的相位<摆动腿的相位，即处于支撑相阶段
      {
        //（1）设置足部接触状态：支撑相=1，摆动相=0
        gaitData.contactStateScheduled(foot) = 1;

        //（2）判断支撑相位状态
        //     方法：支撑相状态=每只脚的当前相位【0~1】/相切换到swing的相位【1】，若为1就可以切换到支撑相，反之
        gaitData.phaseStance(foot) =
            gaitData.phaseVariable(foot) / gaitData.switchingPhase(foot);

        //（3）计算摆动相位，摆动阶段尚未开始，因为脚处于站姿
        gaitData.phaseSwing(foot) = 0.0;

        //（4）计算站姿剩余时间  
        //     公式：站姿剩余的时间=步态总时间*（摆动相位-现在的相位）
        gaitData.timeStanceRemaining(foot) =
            gaitData.periodTime(foot) *
            (gaitData.switchingPhase(foot) - gaitData.phaseVariable(foot));

        //（5）计算摆动剩余时间，支撑相没有摆动时间
        gaitData.timeSwingRemaining(foot) = 0.0;

        //(6)判断是不是第一次接触，处理从站立到其他步态刚开始逻辑用的
        if (gaitData.contactStatePrev(foot) == 0)  //第一次接触表示预定着陆
        {
          //将触地标志设为1
          gaitData.touchdownScheduled(foot) = 1;
        } 
        else                                       //第二次接触
        {
          // 将触地标志设置为0
          gaitData.touchdownScheduled(foot) = 0;
        }

      } 
      
      /*若处于摆动相阶段，计算支撑向步态相关参数*/
      else                                                                 //腿现在的相位<摆动腿的相位，即处于摆动相阶段
      {
        //（1）设置足部接触状态：支撑相=1，摆动相=0，摆动相足部没有接触
        gaitData.contactStateScheduled(foot) = 0;

        //（2）判断支撑相位，脚处于摆动状态，站立状态位0，摆动状态位1
        gaitData.phaseStance(foot) = 1.0;

        //（3）计算摆动相位         
        //    公式：摆动相相位=（当前的相位-摆动相默认相位）/（1-摆动相默认相位）
        gaitData.phaseSwing(foot) =
            (gaitData.phaseVariable(foot) - gaitData.switchingPhase(foot)) /
            (1.0 - gaitData.switchingPhase(foot));

        //（4）计算站姿剩余时间：脚在摆动，没有站立时间，故站立剩余时间为0
        gaitData.timeStanceRemaining(foot) = 0.0;

        //（5）计算摆动剩余时间    
        //     公式：挥杆剩余时间=步态总时间*（1-当前相位值）
        gaitData.timeSwingRemaining(foot) =
            gaitData.periodTime(foot) * (1 - gaitData.phaseVariable(foot));

        //（6）判断是不是第一次摆动，设置摆动标志位，处理从站立到其他步态刚开始逻辑用的
        if (gaitData.contactStatePrev(foot) == 1) //第一次接触表示预定着陆
        {
          //将升空标志设置为1
          gaitData.liftoffScheduled(foot) = 1;
          //记住接地时脚的位置
          // posFootLiftoffWorld = ;
        } 
        else                                      //第二次接触
        {
          // 将升空标志设置为0
          gaitData.liftoffScheduled(foot) = 0;
        }
      }
    } 
    
    /*如果腿未启用步态调度器*/
    else                                 
    {
      gaitData.phaseVariable(foot) = 0.0;       //（1）设置相位为0
      gaitData.contactStateScheduled(foot) = 0; //（2）设置接触状态为0
    }

    /////////*(C)迭代，用该次接触状态赋值上一个时间步*///////////////////////////////////////////////
    gaitData.contactStatePrev(foot) = gaitData.contactStateScheduled(foot);
  }
}



/**
 * 功能：修改步态函数
 * 代码没有调用过
 */
template <typename T>
void GaitScheduler<T>::modifyGait()
{
  switch ((int)userParameters->gait_override)  // 选择步态和参数
  {
  case 0:    // （1）使用来自控制代码的默认设置
    if (gaitData._currentGait != gaitData._nextGait)
    {
      createGait();
    }
    break;

  case 1:    //（2）使用用户设定步态
    if (gaitData._currentGait != GaitType(userParameters->gait_type))
    {
      gaitData._nextGait = GaitType(userParameters->gait_type);
      createGait();
    }
    break;

  case 2:    // （3）使用这里设置的特定步态及其参数来修改步态
    if (gaitData._currentGait != GaitType(userParameters->gait_type))
    {
      gaitData._nextGait = GaitType(userParameters->gait_type);
      createGait();
    }

    //调整步态参数 当前步态周期或摆动切换点和设定步态的不同
    if (fabs(gaitData.periodTimeNominal - (T)userParameters->gait_period_time) > 0.0001 ||
        fabs(gaitData.switchingPhaseNominal - (T)userParameters->gait_switching_phase) > 0.0001)
    {
      // 如果参数当前可覆盖 修改步态参数
      if (gaitData.overrideable == 1)
      {
        gaitData.periodTimeNominal = userParameters->gait_period_time;
        gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
        calcAuxiliaryGaitData();
      }
    }
    break;

  case 3:// （4）使用fsm来的设定
    if (gaitData._currentGait != gaitData._nextGait)
    {
      createGait();
    }
    break;

  case 4:   //使用fsm来的设定
    if (gaitData._currentGait != gaitData._nextGait)
    {
      createGait();
      period_time_natural = gaitData.periodTimeNominal;
      switching_phase_natural = gaitData.switchingPhaseNominal;
    }
    else
    {
      gaitData.periodTimeNominal = period_time_natural;
      gaitData.switchingPhaseNominal = switching_phase_natural;
      calcAuxiliaryGaitData();
    }
    break;
  }
}

/**
 *功能：根据每个步态的类型和参数创建步态结构函数
 *要创建标准步态，只需定义以下内容：
 *   gaitData.periodTimeNominal
 *   gaitData.switchingPhaseNominal
 *   gaitData.phaseOffset
 * 其余设置为：
 *   gaitData.gaitEnabled << 1, 1, 1, 1;
 *   gaitData.initialPhase = 0.0;
 *   gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
 *
 *这些增加了灵活性，用于非常不规则的步态和过渡。
 */
template <typename T>
void GaitScheduler<T>::createGait() 
{
  // （1）根据步态类型获取适当的初始化参数，初始化参数包括创建步态（1）名称、（2）步态使能、（3）步态周期、（4）初始相位、（5）转换相位、（6）相位偏移、（7）相位范围
  switch (gaitData._nextGait) 
  {
    //1）站立步态类型
    case GaitType::STAND:                         //gaitData是一个类，后面的参数都是类内成员
      gaitData.gaitName = "STAND";                //创建步态名称
      gaitData.gaitEnabled << 1, 1, 1, 1;         //步态使能
      gaitData.periodTimeNominal = 10.0;          //步态周期                                      【这个必填】
      gaitData.initialPhase = 0.0;                //腿的初始相位，就是该种步态默认的刚开始的相位
      gaitData.switchingPhaseNominal = 1.0;       //转换相位，腾空相或者支撑相                      【这个必填】
      gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5; //相位偏移，即人为摆动这条腿的，使该退相位偏移的量  【这个必填】
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;  //相位范围，即腿摆动相位的范围
      break;

    //2）周期站立步态类型
    case GaitType::STAND_CYCLE:
      gaitData.gaitName = "STAND_CYCLE";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 1.0;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 1.0;
      gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //3）静态行走步态类型
    case GaitType::STATIC_WALK:
      gaitData.gaitName = "STATIC_WALK";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 1.25;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.8;
      gaitData.phaseOffset << 0.25, 0.0, 0.75, 0.5;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //4）慢跑步态类型
    case GaitType::AMBLE:
      gaitData.gaitName = "AMBLE";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 1.0;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.8;
      gaitData.phaseOffset << 0.0, 0.5, 0.25, 0.75;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //5）快走步态类型
    case GaitType::TROT_WALK:
      gaitData.gaitName = "TROT_WALK";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.6;
      gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //6）小跑步态类型
    case GaitType::TROT:
      gaitData.gaitName = "TROT";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.5;
      gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //7）快跑步态类型
    case GaitType::TROT_RUN:
      gaitData.gaitName = "TROT_RUN";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.4;
      gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //8）溜步步态类型
    case GaitType::PACE:
      gaitData.gaitName = "PACE";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.5;
      gaitData.phaseOffset << 0.0, 0.5, 0.0, 0.5;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //9）跳跃步态类型
    case GaitType::BOUND:
      gaitData.gaitName = "BOUND";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.5;
      gaitData.phaseOffset << 0.0, 0.0, 0.5, 0.5;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //10）自身旋转步态类型
    case GaitType::ROTARY_GALLOP:
      gaitData.gaitName = "ROTARY_GALLOP";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.4;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.2;
      gaitData.phaseOffset << 0.0, 0.8571, 0.3571, 0.5;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //11）快跑步态类型
    case GaitType::TRAVERSE_GALLOP:
      // TODO: find the right sequence, should be easy
      gaitData.gaitName = "TRAVERSE_GALLOP";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.2;
      gaitData.phaseOffset << 0.0, 0.8571, 0.3571, 0.5;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //12）冲刺步态类型
    case GaitType::PRONK:
      gaitData.gaitName = "PRONK";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.5;
      gaitData.phaseOffset << 0.0, 0.0, 0.0, 0.0;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
      break;

    //13）三腿运动步态类型
    case GaitType::THREE_FOOT:
      gaitData.gaitName = "THREE_FOOT";
      gaitData.gaitEnabled << 0, 1, 1, 1;
      gaitData.periodTimeNominal = 0.5;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal = 0.5;
      gaitData.phaseOffset << 0.0, 0.666, 0.0, 0.333;
      gaitData.phaseScale << 0.0, 1.0, 1.0, 1.0;
      break;

    case GaitType::CUSTOM:
      gaitData.gaitName = "CUSTOM";
      // TODO: get custom gait parameters from operator GUI
      break;

    //14）过渡到站立步态类型
    case GaitType::TRANSITION_TO_STAND:
      gaitData.gaitName = "TRANSITION_TO_STAND";
      gaitData.gaitEnabled << 1, 1, 1, 1;
      T oldGaitPeriodTimeNominal = gaitData.periodTimeNominal;
      gaitData.periodTimeNominal = 3 * gaitData.periodTimeNominal;
      gaitData.initialPhase = 0.0;
      gaitData.switchingPhaseNominal =
          (gaitData.periodTimeNominal +
           oldGaitPeriodTimeNominal * (gaitData.switchingPhaseNominal - 1)) /
          gaitData.periodTimeNominal;
      gaitData.phaseOffset << (gaitData.periodTimeNominal +
                               oldGaitPeriodTimeNominal *
                                   (gaitData.phaseVariable(0) - 1)) /
                                  gaitData.periodTimeNominal,
          (gaitData.periodTimeNominal +
           oldGaitPeriodTimeNominal * (gaitData.phaseVariable(1) - 1)) /
              gaitData.periodTimeNominal,
          (gaitData.periodTimeNominal +
           oldGaitPeriodTimeNominal * (gaitData.phaseVariable(2) - 1)) /
              gaitData.periodTimeNominal,
          (gaitData.periodTimeNominal +
           oldGaitPeriodTimeNominal * (gaitData.phaseVariable(3) - 1)) /
              gaitData.periodTimeNominal;
      gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;

      break;
  }

  // （2）根据上面的7个步态类型参数为每只脚设置步态参数
  for (int foot = 0; foot < 4; foot++) 
  {
    if (gaitData.gaitEnabled(foot) == 1) //若步态使能了
    {
      //（1）计算每英尺的缩放周期时间，每英尺的缩放周期时间=步态周期 /腿的相位范围
      gaitData.periodTime(foot) =
          gaitData.periodTimeNominal / gaitData.phaseScale(foot);

      //（2）将脚从站姿切换到摆动的阶段
      gaitData.switchingPhase(foot) = gaitData.switchingPhaseNominal;

      //（3）计算偏移量初始化相位变量，偏移量初始化相位变量=腿的初始相位+相位偏移
      gaitData.phaseVariable(foot) =
          gaitData.initialPhase + gaitData.phaseOffset(foot);

      //（4）计算整个步态周期的总站姿时间，整个步态周期的总站姿时间=步态周期*站立在步态周期的比例
      gaitData.timeStance(foot) =
          gaitData.periodTime(foot) * gaitData.switchingPhase(foot);

      //（5）计算整个步态周期的总摆动时间，整个步态周期的总摆动时间=步态周期*（1-站立在步态周期的比例）
      gaitData.timeSwing(foot) =
          gaitData.periodTime(foot) * (1.0 - gaitData.switchingPhase(foot));

    } 
    else                                //若步态没有使能
    {
      //(1)设置每英尺的缩放周期时间为0
      gaitData.periodTime(foot) = 0.0;

      //(2)设置将脚从站姿切换到摆动的阶段为0
      gaitData.switchingPhase(foot) = 0.0;

      //(3)设置根据偏移量初始化相位变量为0
      gaitData.phaseVariable(foot) = 0.0;

      //(4)设置脚的站立时间为0
      gaitData.timeStance(foot) = 0.0;

      //(5)设置脚的摆动时间很大
      gaitData.timeSwing(foot) = 1.0 / gaitData.periodTime(foot);
    }
  }
}


/**
 * 功能：计算辅助步态数据函数
 * 没有调用到的
 */
template <typename T>
void GaitScheduler<T>::calcAuxiliaryGaitData()
{
  if (gaitData.overrideable == 1)
  {
    if (userParameters->gait_override == 2)
    {
      // gaitData.periodTimeNominal = userParameters->gait_period_time;
      // gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
    }
    else if (userParameters->gait_override == 4)
    {
      //gaitData.periodTimeNominal = userParameters->gait_period_time;
      //gaitData.switchingPhaseNominal = userParameters->gait_switching_phase;
    }
  }

  for (int foot = 0; foot < 4; foot++)  //为每只脚设置步态参数 
  {
    if (gaitData.gaitEnabled(foot) == 1)//支撑相时
    {
      // 每只脚的缩放周期时间
      gaitData.periodTime(foot) =
          gaitData.periodTimeNominal / gaitData.phaseScale(foot);

     //将脚从站姿转换成摇摆姿势的相位
      gaitData.switchingPhase(foot) = gaitData.switchingPhaseNominal;

      //根据偏移量初始化相位变量
      gaitData.phaseVariable(foot) =
          gaitData.initialPhase + gaitData.phaseOffset(foot);

      //整个步态周期的总站立时间
      gaitData.timeStance(foot) =
          gaitData.periodTime(foot) * gaitData.switchingPhase(foot);

      // 整个步态周期的摆动时间
      gaitData.timeSwing(foot) =
          gaitData.periodTime(foot) * (1.0 - gaitData.switchingPhase(foot));
    }
    else                                //腾空相时
    {
      // 每只脚的缩放周期时间
      gaitData.periodTime(foot) = 0.0;

      //将脚从站姿转换成摇摆姿势的相位
      gaitData.switchingPhase(foot) = 0.0;

      //根据偏移量初始化相位变量
      gaitData.phaseVariable(foot) = 0.0;

      //脚一直在摆动，故站立设置为0
      gaitData.timeStance(foot) = 0.0;

      //脚一直在摆动，故摆动设置为1
      gaitData.timeSwing(foot) = 1.0 / gaitData.periodTime(foot);
    }
  }
}



/**
 * 功能：打印有关步态和当前步态状态的信息函数
 */
template <typename T>
void GaitScheduler<T>::printGaitInfo() 
{
  //
  printIter++;

  // 增量打印迭代
  if (printIter == printNum) {
    std::cout << "[GAIT SCHEDULER] Printing Gait Info...\n";
    std::cout << "Gait Type: " << gaitData.gaitName << "\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Enabled: " << gaitData.gaitEnabled(0) << " | "
              << gaitData.gaitEnabled(1) << " | " << gaitData.gaitEnabled(2)
              << " | " << gaitData.gaitEnabled(3) << "\n";
    std::cout << "Period Time: " << gaitData.periodTime(0) << "s | "
              << gaitData.periodTime(1) << "s | " << gaitData.periodTime(2)
              << "s | " << gaitData.periodTime(3) << "s\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Contact State: " << gaitData.contactStateScheduled(0) << " | "
              << gaitData.contactStateScheduled(1) << " | "
              << gaitData.contactStateScheduled(2) << " | "
              << gaitData.contactStateScheduled(3) << "\n";
    std::cout << "Phase Variable: " << gaitData.phaseVariable(0) << " | "
              << gaitData.phaseVariable(1) << " | " << gaitData.phaseVariable(2)
              << " | " << gaitData.phaseVariable(3) << "\n";
    std::cout << "Stance Time Remaining: " << gaitData.timeStanceRemaining(0)
              << "s | " << gaitData.timeStanceRemaining(1) << "s | "
              << gaitData.timeStanceRemaining(2) << "s | "
              << gaitData.timeStanceRemaining(3) << "s\n";
    std::cout << "Swing Time Remaining: " << gaitData.timeSwingRemaining(0)
              << "s | " << gaitData.timeSwingRemaining(1) << "s | "
              << gaitData.timeSwingRemaining(2) << "s | "
              << gaitData.timeSwingRemaining(3) << "s\n";
    std::cout << std::endl;

    //重置迭代计数器
    printIter = 0;
  }
}

template class GaitScheduler<double>;
template class GaitScheduler<float>;