/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <sstream>
#include "../include/kmu_gui/qnode.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

#define PI 3.141592
#define D2R PI/180
#define R2D 180/PI


extern Motor motor_data;
extern Battery battery_data;
extern Fan fan_data;

extern Movie movie_data;
extern Gas gas_data;
extern Capture capture_data;
extern int flag_gas_send;
extern int flag_capture_send;

Motor motor_data;
Battery battery_data;
Fan fan_data;
Movie movie_data;
Gas gas_data;
Capture capture_data;

kmu_gui::gas msg_gas_send;
kmu_gui::capture msg_capture_send;

int flag_gas_send;
int flag_capture_send;

sensor_msgs::Joy joy_message_g; //joy message global(shared)

namespace kmu_gui {
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}


bool QNode::init() {
    ros::init(init_argc,init_argv,"kmu_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    rosmode_publisher = n.advertise<std_msgs::UInt16>("rosmode", 1);
    start();
    return true;
}


bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;

    ros::init(remappings,"kmu_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // Add your ros communications here.
//    rosmode_publisher = n.advertise<std_msgs::UInt16>("rosmode", 1);
    P_gas     = n.advertise<kmu_gui::gas>("RtoG_Cmd", 1);
    P_capture = n.advertise<kmu_gui::capture>("RtoC_Cmd", 1);


    S_Camera_movie = n.subscribe("/movie/image_raw",      1, &QNode::Camera_Callback, this);
    S_top_movie    = n.subscribe("/movie/top_image_raw/", 1, &QNode::Rviz_Top_Callback, this);
    S_tpf_movie    = n.subscribe("/movie/tpf_image_raw/", 1, &QNode::Rviz_Tpf_Callback, this);


    /*
    S_Motor_data = n.subscribe("/RtoC_M_State", 100, &QNode::RtoC_M_Callback, this);
    S_Battery_data = n.subscribe("/RtoC_B_State", 100, &QNode::RtoC_B_Callback, this);
    S_RtoC_Joystick_data = n.subscribe("/RtoC_J_State", 100, &QNode::RtoC_J_Callback, this);
    S_CtoR_Joystick_data = n.subscribe("/joy", 100, &QNode::CtoR_J_Callback, this);
    */
    S_CtoR_Joystick_data = n.subscribe("/joy", 1, &QNode::CtoR_J_Callback, this);

    S_Battery_data = n.subscribe("/BtoR_State", 100, &QNode::battery_Callback, this);
    S_capture_data = n.subscribe("/CtoR_State", 100, &QNode::capture_Callback, this);
    S_fan_data = n.subscribe("/FtoR_State", 100, &QNode::fan_Callback, this);
    S_gas_data = n.subscribe("/GtoR_State", 100, &QNode::gas_Callback, this);

    S_Motor_data = n.subscribe("/MtoR_State", 100, &QNode::motor_Callback, this);

    start();

    lastImgMsgTime=ros::Time::now();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(10);
//    int count = 0;
    ros::NodeHandle n;


    bool gastoggle=0;
    bool captoggle=0;
    while ( ros::ok() ) {
        ros::Duration diff = ros::Time::now()-lastImgMsgTime;
        double rosTimeDiff_byImg= diff.toSec();

        if(flag_gas_send==1)
        {

           if(gastoggle==0)
           {
               gastoggle=1;
               msg_gas_send.CMD=1;
               P_gas.publish(msg_gas_send);
           }
           else
           {
               gastoggle=0;
               msg_gas_send.CMD=2;
               P_gas.publish(msg_gas_send);
           }


           flag_gas_send=0;
        }


        if(flag_capture_send==12)
        {
            std::cout<<"captoggle "<<captoggle<<std::endl;

            if(captoggle==0)
            {
                captoggle=1;
                msg_capture_send.CMD=2;
                P_capture.publish(msg_capture_send);

                flag_capture_send=0;


            }
            else
            {
                captoggle=0;

                msg_capture_send.CMD=1;
                P_capture.publish(msg_capture_send);

               flag_capture_send=0;

            }
        }
        else if(flag_capture_send==3)
        {
            msg_capture_send.CMD=3;
            P_capture.publish(msg_capture_send);

            flag_capture_send=0;

        }


        Q_EMIT mainWindowForceUpdate(rosTimeDiff_byImg);
        ros::spinOnce();
        loop_rate.sleep();
//        ++count;
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}

void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }

    }

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);

    //Q_EMIT loggingUpdated(); // used to readjust the scrollbar

}

void QNode::motor_Callback(const kmu_gui::motor& motor_msg){

    std::cout<<"mot_hit\n";
    /*
    motor_data.HEADER_SEC = motor_msg.HEADER.stamp.sec;
    motor_data.HEADER_NSEC = motor_msg.HEADER.stamp.nsec;
    motor_data.NAME = motor_msg.NAME;
    motor_data.LMC_S = motor_msg.LMC_S;
    motor_data.RMC_S = motor_msg.RMC_S;
    motor_data.FMC_S = motor_msg.FMC_S;

    // LM_GAIN
//    motor_data.LMC_S = motor_msg.LMC_S;
//    motor_data.RMC_S = motor_msg.RMC_S;
//    motor_data.FMC_S = motor_msg.FMC_S;

    motor_data.LM_SPD = motor_msg.LM_SPD;
    motor_data.LW_SPD = motor_msg.LW_SPD;
    motor_data.RM_SPD = motor_msg.RM_SPD;
    motor_data.RW_SPD = motor_msg.RW_SPD;
    motor_data.FM_SPD = motor_msg.FM_SPD;
    motor_data.FW_SPD = motor_msg.FW_SPD;
    motor_data.FM_AGL = motor_msg.FM_AGL;
    motor_data.FW_AGL = motor_msg.FW_AGL;
*/
    Q_EMIT motorflagUpdated();

}

void QNode::battery_Callback(const kmu_gui::battery& battery_msg){

    std::cout<<"bat_hit\n";

//    battery_data.HEADER_SEC = battery_msg.HEADER.stamp.sec;
//    battery_data.HEADER_NSEC = battery_msg.HEADER.stamp.nsec;
//    battery_data.NAME = battery_msg.NAME;
    battery_data.V = battery_msg.V;
    battery_data.A = battery_msg.A;
    battery_data.SOC = battery_msg.SOC;
    battery_data.TEMP = battery_msg.TEMP;

    Q_EMIT batteryflagUpdated();
}

void QNode::capture_Callback(const kmu_gui::capture& capture_msg)
{
    std::cout<<"cap_hit\n";
    //ROS_INFO("gc I heard: ");
    capture_data.line =capture_msg.LINE;
    capture_data.CMD =capture_msg.CMD;
    capture_data.STATE =capture_msg.STATE;

    capture_data.Cap_NSavedData	=capture_msg.N_SAVE_DATA;
    capture_data.Cap_ISavedData	=capture_msg.I_SAVE_DATA;

    for(int i=0; i<100; i++)
    {
        capture_data.Cap_GetLine[i]	  =capture_msg.GET_LINE[i];
        capture_data.Cap_GetFlowRate[i]	  =capture_msg.GET_FLOW_RATE[i];
        capture_data.Cap_GetCapTime[i]	  =capture_msg.GET_CAP_TIME[i];
        capture_data.Cap_GetIntegFlowRate[i] =double(capture_msg.GET_INTEG_FLOW_RATE[i]);

        capture_data.Cap_GetMsmtTime[i]  =capture_msg.GET_MSMT_TIME[i];

        //if (strlen(capture_msg.GET_MSMT_TIME[i].c_str())>0)
        //  sPacket.Cap_GetMsmtTime[i]  =std::stoll(capture_msg.GET_MSMT_TIME[i]);
        //else
        //  sPacket.Cap_GetMsmtTime[i]  =0;
        //std::cout<<capture_msg.GET_FLOW[i]<<" "<<sPacket.Cap_GetMsmtTime[i]<<std::endl;

    }

    Q_EMIT captureflagUpdated();
}



void QNode::fan_Callback(const kmu_gui::fan& fan_msg){

    std::cout<<"fan_hit\n";
    fan_data.CMD = fan_msg.CMD;
    fan_data.State = fan_msg.STATE;
    Q_EMIT fanflagUpdated();
}


void QNode::gas_Callback(const kmu_gui::gas& gas_msg)
{
    std::cout<<"gas_hit\n";
    gas_data.Gas_NodeID = gas_msg.NODE_ID;
    gas_data.Gas_CMD = gas_msg.CMD;
    gas_data.Gas_State = gas_msg.STATE;
    gas_data.Gas_DATE = gas_msg.DATE;
    gas_data.Gas_TIME = gas_msg.TIME;
    gas_data.Gas_Data = gas_msg.DATA;

    Q_EMIT gasflagUpdated();
}


void QNode::joy_Callback(const kmu_gui::joystick& joystick_msg){
    /*
    RtoC_joystick_data.HEADER_SEC = joystick_msg.HEADER.stamp.sec;
    RtoC_joystick_data.HEADER_NSEC = joystick_msg.HEADER.stamp.nsec;
    RtoC_joystick_data.NAME = joystick_msg.NAME;
    RtoC_joystick_data.LS_X = joystick_msg.LS_X;
    RtoC_joystick_data.LS_Y = joystick_msg.LS_Y;
    RtoC_joystick_data.L_T = joystick_msg.L_T;
    RtoC_joystick_data.L1_BTN = joystick_msg.L1_BTN;
    RtoC_joystick_data.L2_BTN = joystick_msg.L2_BTN;
    RtoC_joystick_data.L3_BTN = joystick_msg.L3_BTN;
    RtoC_joystick_data.RS_X = joystick_msg.RS_X;
    RtoC_joystick_data.RS_Y = joystick_msg.RS_Y;
    RtoC_joystick_data.R_T = joystick_msg.R_T;
    RtoC_joystick_data.R1_BTN = joystick_msg.R1_BTN;
    RtoC_joystick_data.R2_BTN = joystick_msg.R2_BTN;
    RtoC_joystick_data.R3_BTN = joystick_msg.R3_BTN;
    RtoC_joystick_data.D_BTN_X = joystick_msg.D_BTN_X;
    RtoC_joystick_data.D_BTN_Y = joystick_msg.D_BTN_Y;
    RtoC_joystick_data.BTN0 = joystick_msg.BTN[0];
    RtoC_joystick_data.BTN1 = joystick_msg.BTN[1];
    RtoC_joystick_data.BTN2 = joystick_msg.BTN[2];
    RtoC_joystick_data.BTN3 = joystick_msg.BTN[3];
    RtoC_joystick_data.S_BTN = joystick_msg.S_BTN;
    RtoC_joystick_data.O_BTN = joystick_msg.O_BTN;
    RtoC_joystick_data.PS_BTN = joystick_msg.PS_BTN;
    RtoC_joystick_data.DMC_BTN = joystick_msg.DMC_BTN;
    RtoC_joystick_data.FMC_BTN = joystick_msg.FMC_BTN;
*/
    //Q_EMIT RtoC_joystickflagUpdated();
}


void QNode::CtoR_J_Callback(const sensor_msgs::Joy msg){
  joy_message_g = msg;
  Q_EMIT CtoR_joystickflagUpdated();
}

/*
void QNode::CtoR_J_Callback(const sensor_msgs::Joy::ConstPtr& msg){
    CtoR_joystick_data.HEADER_SEC =  msg->header.stamp.sec;
    CtoR_joystick_data.HEADER_NSEC = msg->header.stamp.nsec;
    CtoR_joystick_data.NAME = msg->header.frame_id;
    CtoR_joystick_data.LS_X = msg->axes[0];
    CtoR_joystick_data.LS_Y = msg->axes[1];
    CtoR_joystick_data.L_T = msg->buttons[11];
    CtoR_joystick_data.L1_BTN = msg->buttons[4];
    CtoR_joystick_data.L2_BTN = msg->buttons[6];
    CtoR_joystick_data.L3_BTN = msg->buttons[11];
    CtoR_joystick_data.RS_X = msg->axes[3];
    CtoR_joystick_data.RS_Y = msg->axes[4];
    CtoR_joystick_data.R_T = msg->buttons[12];
    CtoR_joystick_data.R1_BTN = msg->buttons[5];
    CtoR_joystick_data.R2_BTN = msg->buttons[7];
    CtoR_joystick_data.R3_BTN = msg->buttons[12];
    CtoR_joystick_data.D_BTN_X = msg->axes[6];
    CtoR_joystick_data.D_BTN_Y = msg->axes[7];
    CtoR_joystick_data.BTN0 = msg->buttons[0]; // X
    CtoR_joystick_data.BTN1 = msg->buttons[1]; // O
    CtoR_joystick_data.BTN2 = msg->buttons[2]; // triangle
    CtoR_joystick_data.BTN3 = msg->buttons[3]; // square
    CtoR_joystick_data.S_BTN = msg->buttons[8];
    CtoR_joystick_data.O_BTN = msg->buttons[9];
    CtoR_joystick_data.PS_BTN = msg->buttons[10];
//    CtoR_joystick_data.DMC_BTN = msg->axes[0];
//    CtoR_joystick_data.FMC_BTN = msg->axes[0];

    Q_EMIT CtoR_joystickflagUpdated();
}*/

void QNode::Camera_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  std::cout<<"IMAGE CALLBACK"<<std::endl;
  QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);

  temp = temp.rgbSwapped();
  QImage image = temp.copy();

  if(!image.isNull())   {
    Q_EMIT CameraUpdated(image);

    lastImgMsgTime = msg->header.stamp;
    std::cout<<"["<<lastImgMsgTime<<"]"<<std::endl;

  }
//   Use image ...
}

void QNode::Rviz_Top_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
  QImage image = temp.copy();

  if(!image.isNull())   {
    Q_EMIT Rviz_Top_Updated(image);

  }
//   Use image ...
}

void QNode::Rviz_Tpf_Callback(const sensor_msgs::ImageConstPtr &msg)
{
  QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
  QImage image = temp.copy();

  if(!image.isNull())   {
    Q_EMIT Rviz_Tpf_Updated(image);
  }
//   Use image ...
}

}  // namespace kmu_gui
