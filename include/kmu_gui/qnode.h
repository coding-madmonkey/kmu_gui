/**
 * @file /include/rok3_qt_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef kmu_gui_QNODE_HPP_
#define kmu_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include "kmu_gui/battery.h"
#include "kmu_gui/capture.h"
#include "kmu_gui/fan.h"
#include "kmu_gui/gas.h"
#include "kmu_gui/joystick.h"
#include "kmu_gui/motor.h"
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QImage>
#include <QLabel>



struct Gas{
// for Gas_CMD, Gas_State;
//# 00 : Nothing
//# 01 : GAS SENSOR OFF
//# 02 : GAS SENSOR ON
    uint8_t   Gas_NodeID;
    uint8_t   Gas_CMD;

    uint8_t   Gas_State;
    uint64_t   Gas_DATE; //YYYYMMDD
    uint64_t   Gas_TIME; //HHMMSS
    uint8_t   Gas_Data;
//# 00 : No data
//# 01 : C3H5
//# 02 : H2S
};

struct Capture{
    int    line; //for command? should talk.
    int    CMD; //for command? should talk.

    int    STATE;
//# 01 : START DONE
//# 02 : START CHECKSUM ERROR
//# 03 : START LINE ERROR
//# 04 : START DISCHARGE ERROR
//# 05 : START TIME ERROR
//# 06 : STOP DONE
//# 07 : STOP CHECKSUM ERROR
//# 08 : WAITING FINISH
//# 09 : FINISH
//# 10 : REQUEST SAVED DATA
//# 11 : RECEIVE SAVED DATA

    int    Cap_NSavedData;
    int    Cap_ISavedData;

    int    Cap_GetLine[100]; // number of tube champer (1~4)
    uint64_t  Cap_GetMsmtTime[100];  //shoud time wisely.
    int    Cap_GetFlowRate[100]; //flux, fluid volume
    int    Cap_GetCapTime[100];   //should time wisely. what is difference with msmt?
    double    Cap_GetIntegFlowRate[100];  //integrated flux float
};

struct Motor{
    int HEADER_SEC;
    int HEADER_NSEC;
    std::string NAME;

    std::string CM;
    std::string RP;

    bool LMC_S;
    bool RMC_S;
    bool FMC_S;

    int LM_SPD;
    float LW_SPD;

    int RM_SPD;
    float RW_SPD;

    int FM_SPD;
    float FW_SPD;

    int FM_AGL;
    float FW_AGL;
};

struct Fan{ //220922
    /* for both CMD, State
    // 00 : No Command
    # 01 : FAN OFF
    # 02 : FAN ON
    # 03 : FAN DUTY 0%
    # 04 : FAN DUTY 25%
    # 05 : FAN DUTY 50%
    # 06 : FAN DUTY 75%
    # 07 : FAN DUTY 100%
    */
    unsigned int CMD;
    unsigned int State;
};

struct Battery{ //220922

    float V;
    float A;

    unsigned int SOC;
    unsigned int TEMP;
};

struct Joystick{
    int HEADER_SEC;
    int HEADER_NSEC;
    std::string NAME;

    double LS_X;
    double LS_Y;
    int L_T;

    bool L1_BTN;
    bool L2_BTN;
    bool L3_BTN;

    double RS_X;
    double RS_Y;
    int R_T;

    bool R1_BTN;
    bool R2_BTN;
    bool R3_BTN;

    double D_BTN_X;
    double D_BTN_Y;

    bool BTN0, BTN1, BTN2, BTN3;

    bool S_BTN;
    bool O_BTN;
    bool PS_BTN;

    bool DMC_BTN;
    bool FMC_BTN;
};

struct Movie{
    int cam_width;
    int cam_height;

    int rviz_width;
    int rviz_height;
};

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kmu_gui {

  /*****************************************************************************
  ** Class
  *****************************************************************************/

  	class QNode : public QThread {
  		Q_OBJECT
  	public:
  		QNode(int argc, char** argv );
  		virtual ~QNode();
  		bool init();
  		bool init(const std::string &master_url, const std::string &host_url);
  		void run();
      void Camera_Callback(const sensor_msgs::ImageConstPtr &msg);
      void Rviz_Top_Callback(const sensor_msgs::ImageConstPtr &msg);
      void Rviz_Tpf_Callback(const sensor_msgs::ImageConstPtr &msg);

      void battery_Callback(const kmu_gui::battery& battery_msg);
      void capture_Callback(const kmu_gui::capture& capture_msg);
      void fan_Callback(const kmu_gui::fan& fan_msg);
      void gas_Callback(const kmu_gui::gas& gas_msg);

      void joy_Callback(const kmu_gui::joystick& joystick_msg);


      void motor_Callback(const kmu_gui::motor& motor_msg);

      //void CtoR_J_Callback(const sensor_msgs::Joy::ConstPtr& msg);
      void CtoR_J_Callback(const sensor_msgs::Joy msg);



      /*********************
      ** Logging
      **********************/
  		enum LogLevel {
  			Debug,
  			Info,
  			Warn,
  			Error,
  			Fatal
  		};
      QImage view_image;
  		QStringListModel* loggingModel() { return &logging_model; }
  		void log( const LogLevel &level, const std::string &msg);

  		Q_SIGNALS:
  		void loggingUpdated();

      void motorflagUpdated();
      void batteryflagUpdated();
      void fanflagUpdated();
      //void RtoC_joystickflagUpdated();
      //void CtoR_joystickflagUpdated();
      void CtoR_joystickflagUpdated();

      void rosShutdown();
      void CameraUpdated(const QImage &);
      void Rviz_Top_Updated(const QImage &);
      void Rviz_Tpf_Updated(const QImage &);


      void gasflagUpdated();
      void captureflagUpdated();

      void mainWindowForceUpdate(const double);
  	private:
  		int init_argc;
  		char** init_argv;
      ros::Publisher rosmode_publisher;

      ros::Publisher P_gas;
      ros::Publisher P_capture;

      ros::Subscriber S_Camera_movie;
      ros::Subscriber S_top_movie;
      ros::Subscriber S_tpf_movie;

      ros::Subscriber S_Battery_data;
      ros::Subscriber S_capture_data;
      ros::Subscriber S_fan_data;
      ros::Subscriber S_gas_data;

      ros::Subscriber S_Motor_data;

      ros::Subscriber S_RtoC_Joystick_data;
      ros::Subscriber S_CtoR_Joystick_data;

      ros::Time lastImgMsgTime;

        QStringListModel logging_model;
  	};



  }  // namespace kmu_gui

#endif /* kmu_gui_QNODE_HPP_ */
