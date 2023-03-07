/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/kmu_gui/main_window.h"
#include "../include/kmu_gui/qnode.h"
#include <ctime>
#include <iomanip>
/*****************************************************************************
** Namespaces
*****************************************************************************/

extern Motor motor_data;
extern Battery battery_data;
extern Fan fan_data;

extern Movie movie_data;

extern Gas gas_data;
extern Capture capture_data;

extern int flag_gas_send;
extern int flag_capture_send;
bool capture_toggle=0;
bool gas_toggle=1;
bool cap1_is_diff=0;
double cap1_old;
double cap1_latest;

unsigned short capDisplayNumber=0;

extern sensor_msgs::Joy joy_message_g; //joy message global(shared)

namespace kmu_gui {

        using namespace Qt;
        /*****************************************************************************
            ** Implementation [MainWindow]
            *****************************************************************************/

            MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
                : QMainWindow(parent, Qt::FramelessWindowHint) // 상단바 없애기. (frameless)
            , qnode(argc,argv) {
                ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
                QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

                ReadSettings();
                setWindowIcon(QIcon(":/images/icon.png"));

                ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

                ui.slam_reset_pushButton->setFocusPolicy(Qt::NoFocus);
                ui.slam_help_pushButton->setFocusPolicy(Qt::NoFocus);
                ui.slam_save_pushButton->setFocusPolicy(Qt::NoFocus);
                ui.slam_gas_pushButton->setFocusPolicy(Qt::NoFocus);

                ui.gas_pushButton->setFocusPolicy(Qt::NoFocus);
                ui.capture_pushButton->setFocusPolicy(Qt::NoFocus);
                ui.capture_button->setFocusPolicy(Qt::NoFocus);

                ui.gas_pushButton->setStyleSheet("qproperty-icon: url(:/ic_sensor_off.png);");
                ui.capture_pushButton->setStyleSheet("qproperty-icon: url(:/ic_capture_off.png);");
                ui.capture_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                 );

                ui.gas_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                     "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                 );

                ui.capture_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                                                      "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                 );

                //ui.tab_manager->setAlignment(Qt::AlignCenter);
                QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
                QObject::connect(&qnode, SIGNAL(mainWindowForceUpdate(const double)), this, SLOT(mainUIupdate(const double)));

                /*********************
                ** Logging
                **********************/
//                ui.view_logging->setModel(qnode.loggingModel()); //diff.sec, diff.nsec
                QObject::connect(&qnode, SIGNAL(CameraUpdated(const QImage &)),    this, SLOT(updateCameraView(const QImage &)));
                QObject::connect(&qnode, SIGNAL(Rviz_Top_Updated(const QImage &)), this, SLOT(updateRviz_topView(const QImage &)));
                QObject::connect(&qnode, SIGNAL(Rviz_Tpf_Updated(const QImage &)), this, SLOT(updateRviz_tpfView(const QImage &)));
                QObject::connect(&qnode, SIGNAL(motorflagUpdated()), this, SLOT(updateMotorDataView()));
                QObject::connect(&qnode, SIGNAL(batteryflagUpdated()), this, SLOT(updateBatteryDataView()));
                QObject::connect(&qnode, SIGNAL(fanflagUpdated()), this, SLOT(updateFanDataView()));
                //QObject::connect(&qnode, SIGNAL(RtoC_joystickflagUpdated()), this, SLOT(RtoC_updateJDataView()));
                //QObject::connect(&qnode, SIGNAL(CtoR_joystickflagUpdated()), this, SLOT(CtoR_updateJDataView()));
                QObject::connect(&qnode, SIGNAL(CtoR_joystickflagUpdated()), this, SLOT(CtoR_updateJDataView()));

                QObject::connect(&qnode, SIGNAL(gasflagUpdated()), this, SLOT(gas_updateDataView()));
                QObject::connect(&qnode, SIGNAL(captureflagUpdated()), this, SLOT(capture_updateDataView()));

                /*********************
                ** Auto Start
                **********************/
                if ( ui.comm_checkbox_remember_settings->isChecked() ) {
                    on_comm_connect_button_clicked(true);
                }

                /*********************
                ** Label
                **********************/

                m_lightimg[0].load(":/images/Red.png");
                m_lightimg[1].load(":/images/Green.png");

                m_switchimg[0].load(":/images/switch1.jpg");
                m_switchimg[1].load(":/images/switch2.jpg");



                cam_icon_img.load(":/ic_camera.png");
                slam_icon_img.load(":/ic_slam.png");
                ui.main_cam_icon_label->setPixmap(cam_icon_img);
                ui.main_top_icon_label->setPixmap(slam_icon_img);
                ui.main_tpf_icon_label->setPixmap(slam_icon_img);

                top_header_img.load(":/img_header.png");
                kiro_img.load(":/kiro_logo.png");
                kmu_img.load(":/kmu_logo.png");
                rclab_img.load(":/RCLAB_A1A5.png");
                ui.top_header_label->setPixmap(top_header_img);
                ui.kiro_label->setPixmap(kiro_img);
                //ui.kmu_label->setPixmap(rclab_img);
                ui.kmu_label_2->setPixmap(kmu_img);

                QString titleText= QString::fromUtf8("원격 이동형 측정 로봇");
                //QFont font("font: Pretendard Variable", 16, QFont::System);
                //ui.top_title_label->setFont(font);

                gasText_normal= QString::fromUtf8("정상상태");
                gas0Text= QString::fromUtf8("니트로글리세린");
                gas1Text= QString::fromUtf8("황화수소");
                gas2Text= QString::fromUtf8("일산화린탄소");
                gas3Text= QString::fromUtf8("암모니아");


                ui.top_title_label->setText(titleText);
                //Main Tab
                //ui.main_comm_delay_lcd->setDigitCount(5);
                //ui.main_comm_speed_lcd->setDigitCount(5);
                //ui.main_robotstatus_FW_AGL_lcd->setDigitCount(5);

                comm_off_img.load(":/ic_connection_off.png");
                comm_on_img.load(":/ic_connection_on.png");

                ui.main_connectionstatus_label->setPixmap(comm_off_img);
                ui.main_connectionstatus_label->setStyleSheet("QLabel {background-color : #2a2d43 }");
                ui.basic_con_frame->setStyleSheet("QFrame {border: 0px solid #FFFFFF;}");

                power_off0_img.load(":/ic_power_off_0.png");
                power_00_img.load(":/ic_power_on_0.png");
                power_20_img.load(":/ic_power_on_20.png");
                power_40_img.load(":/ic_power_on_40.png");
                power_60_img.load(":/ic_power_on_60.png");
                power_80_img.load(":/ic_power_on_80.png");
                power_100_img.load(":/ic_power_on_100.png");
                ui.main_powerstatus_label->setStyleSheet("QLabel { border: solid 2px #339bff; background-color : #2a2d43 }");
                ui.main_powerstatus_label->setPixmap(power_off0_img);
                ui.basic_power_frame->setStyleSheet("QFrame {border: 0px solid #FFFFFF;}");

                motor_on_img.load(":/ic_motor_on.png");
                motor_off_img.load(":/ic_motor_off.png");
                ui.main_motorstatus_label->setStyleSheet("QLabel { border: solid 2px #339bff; background-color : #2a2d43 }");
                ui.main_motorstatus_label->setPixmap(motor_off_img);
                ui.basic_motor_frame->setStyleSheet("QFrame {border: 0px solid #FFFFFF;}");

                fan_off_img.load(":/ic_fan_off.png");
                fan_stop_img.load(":/ic_fan_stop.png");
                fan_step_1_img.load(":/ic_fan_1step.png");
                fan_step_2_img.load(":/ic_fan_2step.png");
                fan_step_3_img.load(":/ic_fan_3step.png");
                fan_step_4_img.load(":/ic_fan_4step.png");
                ui.main_fanstatus_label->setStyleSheet("QLabel { border: solid 2px #339bff; background-color : #2a2d43 }");
                ui.main_fanstatus_label->setPixmap(fan_off_img);
                ui.basic_fan_frame->setStyleSheet("QFrame {border: 0px solid #FFFFFF; }");



                gas_led_red.load(":/gas-red.png");
                gas_led_yellow.load(":/gas-yellow.png");
                gas_led_blue.load(":/gas-blue.png");
                gas_led_black.load(":/gas-black.png");
                ui.gas_led_0->setPixmap(gas_led_black);
                ui.gas_led_1->setPixmap(gas_led_black);
                ui.gas_led_2->setPixmap(gas_led_black);
                ui.gas_led_3->setPixmap(gas_led_black);

                gas_on_img.load(":/ic_sensor_on.png");
                gas_off_img.load(":/ic_sensor_off.png");
                //ui.main_gasstatus_label->setStyleSheet("QLabel { border: solid 2px #339bff; background-color : #2a2d43 }");
                //ui.main_gasstatus_label->setPixmap(gas_off_img);
                ui.basic_gas_frame->setStyleSheet("QFrame {border: 0px solid #FFFFFF;}");

                ui.gas_lineEdit_0->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");
                ui.gas_lineEdit_1->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");
                ui.gas_lineEdit_2->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");
                ui.gas_lineEdit_3->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");


                capture_on_img.load(":/ic_capture_on.png");
                capture_off_img.load(":/ic_capture_off.png");
                //ui.main_capturestatus_label->setStyleSheet("QLabel { border: solid 2px #339bff; background-color : #2a2d43 }");
                //ui.main_capturestatus_label->setPixmap(capture_off_img);
                ui.basic_capture_frame->setStyleSheet("QFrame {border: 0px solid #FFFFFF;}");
                ui.capture_lineEdit_0->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");
                ui.capture_lineEdit_1->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");
                ui.capture_lineEdit_2->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");
                ui.capture_lineEdit_3->setStyleSheet("QLineEdit {border: 2px solid #575a6c; border-radius: 4px; background-color: #191b29; color: #575a6c }");


                ui.capture_button->setStyleSheet("QPushButton{ border: 1px solid rgb(161, 165, 190); }\n"
                                                 "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                 "QPushButton{ color:rgb(161, 165, 190);}\n"
                                                        );


                slam_on_img.load(":/ic_SLAM_on.png");
                slam_off_img.load(":/ic_SLAM_off.png");
                //slam_reset_img.load(":/ic_reset.png");
                //slam_help_img.load(":/ic_help.png");
                //slam_save_img.load(":/ic_save.png");
                //slam_gas_img.load(":/ic_gas.png");


                ui.main_slamonoff_label->setStyleSheet("QLabel { border: solid 2px #339bff; background-color : #2a2d43 }");
                ui.main_slamonoff_label->setPixmap(slam_off_img);
                //ui.main_slamreset_label->setPixmap(slam_reset_img);
                //ui.main_slamhelp_label->setPixmap(slam_help_img);
                //ui.main_slamsave_label->setPixmap(slam_save_img);
                //ui.main_slamgass_label->setPixmap(slam_gas_img);
                //background-color: #2a2d43;
/*
                ui.summary_LM_SPD_lcd->setDigitCount(10);
                ui.summary_LW_SPD_lcd->setDigitCount(10);
                ui.summary_FM_SPD_lcd->setDigitCount(10);
                ui.summary_FW_SPD_lcd->setDigitCount(10);
                ui.summary_RM_SPD_lcd->setDigitCount(10);
                ui.summary_RW_SPD_lcd->setDigitCount(10);
                ui.summary_FM_AGL_lcd->setDigitCount(10);
                ui.summary_FW_AGL_lcd->setDigitCount(10);
                ui.summary_￣voltage_lcd->setDigitCount(10);
                ui.summary_ampere_lcd->setDigitCount(10);
                ui.summary_temperature_lcd->setDigitCount(10);
*/

                ui.sensor_gps_latitude_lcd ->setDigitCount(10);
                ui.sensor_gps_longitude_lcd ->setDigitCount(10);
                ui.sensor_gps_altitude_lcd ->setDigitCount(10);
                ui.sensor_gps_time_lcd ->setDigitCount(10);
                ui.sensor_imu_angular_vel_x_lcd ->setDigitCount(10);
                ui.sensor_imu_angular_vel_y_lcd ->setDigitCount(10);
                ui.sensor_imu_angular_vel_z_lcd ->setDigitCount(10);
                ui.sensor_imu_linear_accel_x_lcd ->setDigitCount(10);
                ui.sensor_imu_linear_accel_y_lcd ->setDigitCount(10);
                ui.sensor_imu_linear_accel_z_lcd ->setDigitCount(10);

                ui.sensor_imu_quaternion_x_lcd ->setDigitCount(10);
                ui.sensor_imu_quaternion_y_lcd ->setDigitCount(10);
                ui.sensor_imu_quaternion_z_lcd ->setDigitCount(10);
                ui.sensor_imu_quaternion_w_lcd ->setDigitCount(10);

                ui.motor_header_sec_lcd ->setDigitCount(5);
                ui.motor_header_nsec_lcd ->setDigitCount(5);

                ui.motor_LM_SPD_lcd ->setDigitCount(5);
                ui.motor_LW_SPD_lcd ->setDigitCount(5);
                ui.motor_FM_SPD_lcd ->setDigitCount(5);
                ui.motor_FW_SPD_lcd ->setDigitCount(5);
                ui.motor_RM_SPD_lcd ->setDigitCount(5);
                ui.motor_RW_SPD_lcd ->setDigitCount(5);
                ui.motor_FM_AGL_lcd ->setDigitCount(5);
                ui.motor_FW_AGL_lcd ->setDigitCount(5);

                /*
                ui.battery_sec_lcd ->setDigitCount(5);
                ui.battery_nsec_lcd ->setDigitCount(5);
                ui.battery_voltage_lcd ->setDigitCount(5);
                ui.battery_ampere_lcd ->setDigitCount(5);
                ui.battery_temperature_lcd ->setDigitCount(5);
*/
                /*******************************
                ** Button test - explicit way
                ********************************/




                ui.header_sec_lcdNumber->setDigitCount(9);
                ui.header_nsec_lcdNumber->setDigitCount(9);
                ui.ls_x_lcdNumber->setDigitCount(6);
                ui.ls_y_lcdNumber->setDigitCount(6);
                ui.rs_x_lcdNumber->setDigitCount(6);
                ui.rs_y_lcdNumber->setDigitCount(6);

                init_joy_resource_map();
                initialize_joy_tab();


            }

            MainWindow::~MainWindow() {}

            /*****************************************************************************
            ** Implementation [Slots]
            *****************************************************************************/

            void MainWindow::showNoMasterMessage() {
                QMessageBox msgBox;
                msgBox.setText("Couldn't find the ros master.");
                msgBox.exec();
                close();
            }

            void MainWindow::showButtonTestMessage() {
                QMessageBox msgBox;
                msgBox.setText("Button test ...");
                msgBox.exec();
                //close();
            }
            /*
             * These triggers whenever the button is clicked, regardless of whether it
             * is already checked or not.
             */

            void MainWindow::on_comm_connect_button_clicked(bool check ) {
                    if ( ui.comm_checkbox_use_environment->isChecked() ) {
                        if ( !qnode.init() ) {
                            showNoMasterMessage();
                        } else {
                            ui.comm_connect_button->setEnabled(false);
                            //
                            ui.main_connectionstatus_label->setPixmap(comm_on_img);
                            //ui.main_connectionstatus_label->setPixmap(comm_off_img);
                            ui.basic_con_frame->setStyleSheet("QFrame {border: 1px solid #339bff;}");


                        }
                    } else {
                        if ( ! qnode.init(ui.comm_connect_line_edit_master->text().toStdString(),
                            ui.comm_connect_line_edit_host->text().toStdString()) ) {
                            showNoMasterMessage();
                    } else {
                        ui.comm_connect_button->setEnabled(false);
                        ui.comm_connect_line_edit_master->setReadOnly(true);
                        ui.comm_connect_line_edit_host->setReadOnly(true);
                        ui.comm_connect_line_edit_topic->setReadOnly(true);
                        //
                        ui.main_connectionstatus_label->setPixmap(comm_on_img);
                        ui.basic_con_frame->setStyleSheet("QFrame {border: 1px solid #339bff;}");

                    }
                }
            }

//            void MainWindow::on_button_test_clicked(bool check ) {
//                showButtonTestMessage();
//            }

            void MainWindow::on_comm_checkbox_use_environment_stateChanged(int state) {
                bool enabled;
                if ( state == 0 ) {
                    enabled = true;
                } else {
                    enabled = false;
                }
                ui.comm_connect_line_edit_master->setEnabled(enabled);
                ui.comm_connect_line_edit_host->setEnabled(enabled);
                //ui.line_edit_topic->setEnabled(enabled);
            }

            /*****************************************************************************
            ** Implemenation [Slots][manually connected]
            *****************************************************************************/

            /**
             * This function is signalled by the underlying model. When the model changes,
             * this will drop the cursor down to the last line in the QListview to ensure
             * the user can always see the latest log message.
             */

            void MainWindow::mainUIupdate(const double rosTimeDiff_byImg){
                //ui.main_comm_delay_lcd->display(rosTimeDiff_byImg);
                //sstd::cout<<rosTimeDiff_byImg<<std::endl;
                QString timetext = QDateTime::currentDateTime().toString("hh:mm:ss");
                QString datetext = QDateTime::currentDateTime().toString("yyyy-MM-dd");
                //setETLabelText();
                ui.top_date_label->setText(datetext);
                ui.top_time_label->setText(timetext);
            }


            void MainWindow::updateCameraView(const QImage &msg) {
                ui.main_camera->setPixmap(QPixmap::fromImage(msg));
//                QPixmap p = QPixmap::fromImage(msg);
            }

            void MainWindow::updateRviz_topView(const QImage &msg) {
                ui.main_movie_top->setPixmap(QPixmap::fromImage(msg));
                ui.main_slamonoff_label->setPixmap(slam_on_img);
                ui.main_slamonoff_label->setStyleSheet("QFrame {border: 2px solid #339bff;}");
//                QPixmap p = QPixmap::fromImage(msg);
            }

            void MainWindow::updateRviz_tpfView(const QImage &msg) {
                ui.main_movie_tpf->setPixmap(QPixmap::fromImage(msg));
                ui.main_slamonoff_label->setPixmap(slam_on_img);
                ui.main_slamonoff_label->setStyleSheet("QFrame {border: 2px solid #339bff;}");
//                QPixmap p = QPixmap::fromImage(msg);
            }

            void MainWindow::updateMotorDataView() {
              ui.motor_header_sec_lcd->display(motor_data.HEADER_SEC);
              ui.motor_header_nsec_lcd->display(motor_data.HEADER_NSEC);

              ui.motor_Name->setText(QString(motor_data.NAME.c_str()));
              ui.motor_CM->setText(QString(motor_data.CM.c_str()));
              ui.motor_RP->setText(QString(motor_data.RP.c_str()));

//              if(motor_data.DMC_S == true){
//                ui.motor_DMC_S_signal->setPixmap(m_lightimg[1]);
//                ui.main_robotstatus_controller_signal->setPixmap(m_lightimg[1]);
//              }
//              else{
//                ui.motor_DMC_S_signal->setPixmap(m_lightimg[0]);
//                ui.main_robotstatus_controller_signal->setPixmap(m_lightimg[0]);
//              }

              if(motor_data.FMC_S == true){
                ui.motor_FMC_S_signal->setPixmap(m_lightimg[1]);
              }
              else{
                ui.motor_FMC_S_signal->setPixmap(m_lightimg[0]);
              }

              ui.motor_LM_SPD_lcd->display(motor_data.LM_SPD);
              ui.motor_LW_SPD_lcd->display(motor_data.LW_SPD);
              ui.motor_RM_SPD_lcd->display(motor_data.RM_SPD);
              ui.motor_RW_SPD_lcd->display(motor_data.RW_SPD);
              ui.motor_FM_SPD_lcd->display(motor_data.FM_SPD);
              ui.motor_FW_SPD_lcd->display(motor_data.FW_SPD);
              ui.motor_FM_AGL_lcd->display(motor_data.FM_AGL);
             // ui.main_robotstatus_FW_AGL_lcd->display(motor_data.FW_AGL);

              ui.main_motorstatus_label->setPixmap(motor_on_img);
              ui.basic_motor_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");
            }



            void MainWindow::updateBatteryDataView() {

                //ui.main_powerstatus_label->setPixmap(power_off0_img);
                //ui.main_powerstatus_label->setPixmap(power_00_img);
                //ui.main_powerstatus_label->setPixmap(power_20_img);
                //ui.main_powerstatus_label->setPixmap(power_40_img);
                //ui.main_powerstatus_label->setPixmap(power_60_img);
                //ui.main_powerstatus_label->setPixmap(power_80_img);
                //ui.main_powerstatus_label->setPixmap(power_100_img);

                ui.basic_power_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");
                if(battery_data.V<16.0)
                {
                    ui.main_powerstatus_label->setPixmap(power_00_img);
                    ui.basic_fan_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");

                }
                else if(battery_data.V<17.0)
                {
                    ui.main_powerstatus_label->setPixmap(power_20_img);
                    //ui.basic_fan_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");
                    ui.basic_fan_frame->setStyleSheet("QFrame {border: 2px solid #ff3333;}");

                    ;
                }
                else if(battery_data.V<18.0)
                {
                    ui.main_powerstatus_label->setPixmap(power_40_img);
                    ui.basic_fan_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");
                }
                else if(battery_data.V<19.0)
                {
                    ui.main_powerstatus_label->setPixmap(power_60_img);
                    ui.basic_fan_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");
                }
                else if(battery_data.V<20.0)
                {
                    ui.main_powerstatus_label->setPixmap(power_80_img);
                    ui.basic_fan_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");
                }
                else if(battery_data.V>=20.0)
                {
                    ui.main_powerstatus_label->setPixmap(power_100_img);
                }
                else
                {
                    ui.main_powerstatus_label->setPixmap(power_off0_img);
                }


                /*
              ui.battery_sec_lcd->display(battery_data.HEADER_SEC);
              ui.battery_nsec_lcd->display(battery_data.HEADER_NSEC);
              ui.battery_name->setText(QString(battery_data.NAME.c_str()));

              ui.battery_batterybar->setValue(battery_data.SOC);
              ui.main_robotstatus_batterybar->setValue(battery_data.SOC);
              ui.battery_voltage_lcd->display(battery_data.V);
              ui.battery_ampere_lcd->display(battery_data.A);
              ui.battery_temperature_lcd->display((int)battery_data.TEMP);
              */
            }
            /////////////////////////////////////////////////////////////////////

            void MainWindow::updateFanDataView() {
                //just for show
                //fan_off_img.load(":/ic_fan_off.png");
                //fan_stop_img.load(":/ic_fan_stop.png");
                //fan_step_1_img.load(":/ic_fan_1step.png");
                //fan_step_2_img.load(":/ic_fan_2step.png");
                //fan_step_3_img.load(":/ic_fan_3step.png");
                //fan_step_4_img.load(":/ic_fan_4step.png");
                //ui.main_fanstatus_label->setStyleSheet("QLabel { border: solid 2px #339bff; background-color : #2a2d43 }");
                //ui.main_fanstatus_label->setPixmap(fan_off_img);

                ui.basic_fan_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");
                if(fan_data.State==1) //no command, fan off
                {
                    //should fix? no command
                    ui.main_fanstatus_label->setPixmap(fan_off_img);
                }
                else if(fan_data.State==6)
                {
                    ui.main_fanstatus_label->setPixmap(fan_step_3_img);
                }


            }

            void MainWindow::gas_updateDataView()
            {
                std::cout<<"mainwindow_h. gasUp"<<std::endl;
                //ui.main_gasstatus_label->setPixmap(gas_on_img);
/*

                if(gas_data.Gas_State==2)
                {
                ui.gas_pushButton->setStyleSheet("qproperty-icon: url(:/ic_sensor_on.png);");
                //ui.gas_pushButton->setPixmap(gas_on_img);

                ui.basic_gas_frame->setStyleSheet("QFrame {border: 2px solid #339bff;}");

                ui.gas_lineEdit_0->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
                ui.gas_lineEdit_1->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
                ui.gas_lineEdit_2->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
                ui.gas_lineEdit_3->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
                }
                else {
                    ui.gas_pushButton->setStyleSheet("qproperty-icon: url(:/ic_sensor_off.png);");
                    //ui.gas_pushButton->setPixmap(gas_on_img);

                    ui.basic_gas_frame->setStyleSheet("QFrame {border: 0px solid #FFFFFF;}");

                }
  */
//                gas0Text= QString::fromUtf8("니트로글리세린");
//                gas1Text= QString::fromUtf8("황화수소");
//                gas2Text= QString::fromUtf8("일산화탄소");
//                gas3Text= QString::fromUtf8("암모니아");


                if(gas_data.Gas_NodeID==1)
                {
                    ui.gas_lineEdit_0->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");

                    switch(gas_data.Gas_Data)
                    {
                    case 0:
                        ui.gas_led_0->setPixmap(gas_led_blue);
                        ui.gas_lineEdit_0->setText(gasText_normal);
                        break;
                    case 1:
                        ui.gas_led_0->setPixmap(gas_led_red);
                        ui.gas_lineEdit_0->setText(gas0Text);
                        break;
                    case 2:
                        ui.gas_led_0->setPixmap(gas_led_black);
                        ui.gas_lineEdit_0->setText(gas1Text);
                        break;
                    }
                }
                if(gas_data.Gas_NodeID==2)
                {
                    ui.gas_lineEdit_1->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
                    switch(gas_data.Gas_Data)
                    {
                    case 0:
                        ui.gas_led_1->setPixmap(gas_led_blue);
                        ui.gas_lineEdit_1->setText(gasText_normal);
                        break;
                    case 1:
                        ui.gas_led_1->setPixmap(gas_led_red);
                        ui.gas_lineEdit_1->setText(gas0Text);
                        break;
                    case 2:
                        ui.gas_led_1->setPixmap(gas_led_black);
                        ui.gas_lineEdit_1->setText(gas1Text);
                        break;
                    }
                }
                if(gas_data.Gas_NodeID==3)
                {
                    ui.gas_lineEdit_2->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
                    switch(gas_data.Gas_Data)
                    {
                    case 0:
                        ui.gas_led_2->setPixmap(gas_led_blue);
                        ui.gas_lineEdit_2->setText(gasText_normal);
                        break;
                    case 1:
                        ui.gas_led_2->setPixmap(gas_led_red);
                        ui.gas_lineEdit_2->setText(gas0Text);
                        break;
                    case 2:
                        ui.gas_led_2->setPixmap(gas_led_black);
                        ui.gas_lineEdit_2->setText(gas1Text);
                        break;
                    }
                }
                if(gas_data.Gas_NodeID==4)
                {
                    ui.gas_lineEdit_3->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");



                    switch(gas_data.Gas_Data)
                    {
                    case 0:
                        ui.gas_led_3->setPixmap(gas_led_blue);
                        ui.gas_lineEdit_3->setText(gasText_normal);
                        break;
                    case 1:
                        ui.gas_led_3->setPixmap(gas_led_red);
                        ui.gas_lineEdit_3->setText(gas0Text);
                        break;
                    case 2:
                        ui.gas_led_3->setPixmap(gas_led_black);
                        ui.gas_lineEdit_3->setText(gas1Text);
                        break;
                    }
                }
            }

            void MainWindow::capture_updateDataView()
            {


                bool newdataIn;
                newdataIn=0;
                if(cap1_old !=cap1_latest)//capture_data.Cap_GetIntegFlowRate[0])
                {
                    newdataIn=1;
                    cap1_latest=capture_data.Cap_GetIntegFlowRate[0];
                    std::cout<<"new!= old : "<<capture_data.Cap_GetIntegFlowRate[0]<<" "<<std::endl;

                }
                else {
                    newdataIn=0;
                }


                if(newdataIn==1 )//&& capture_data.Cap_GetIntegFlowRate[0]!=0)
                {

                    if(capDisplayNumber%4==0)
                    {
                        capDisplayNumber++;
                        ui.capture_lineEdit_0->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");

                        cap1timetext = QDateTime::currentDateTime().toString("hh:mm:ss");
                        ui.capture_lineEdit_0->setText(cap1timetext);
                        std::cout<<"Inloop"<<std::endl;

                        capture_toggle=0;
                        flag_capture_send=12;
                        ui.capture_pushButton->setStyleSheet("qproperty-icon: url(:/ic_capture_off.png);");
                        ui.capture_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                            "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                     );
                    }
                    else if(capDisplayNumber%4==1)
                    {
                        capDisplayNumber++;
                        ui.capture_lineEdit_1->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");

                        cap1timetext = QDateTime::currentDateTime().toString("hh:mm:ss");
                        ui.capture_lineEdit_1->setText(cap1timetext);
                        std::cout<<"Inloop"<<std::endl;

                        capture_toggle=0;
                        flag_capture_send=12;
                        ui.capture_pushButton->setStyleSheet("qproperty-icon: url(:/ic_capture_off.png);");
                        ui.capture_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                            "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                             );
                    }
                    else if(capDisplayNumber%4==2)
                    {
                        capDisplayNumber++;
                        ui.capture_lineEdit_2->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");

                        cap1timetext = QDateTime::currentDateTime().toString("hh:mm:ss");
                        ui.capture_lineEdit_2->setText(cap1timetext);
                        std::cout<<"Inloop"<<std::endl;

                        capture_toggle=0;
                        flag_capture_send=12;
                        ui.capture_pushButton->setStyleSheet("qproperty-icon: url(:/ic_capture_off.png);");
                        ui.capture_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                            "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                     );
                    }
                    else if(capDisplayNumber%4==3)
                    {
                        capDisplayNumber++;
                        ui.capture_lineEdit_3->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");

                        cap1timetext = QDateTime::currentDateTime().toString("hh:mm:ss");
                        ui.capture_lineEdit_3->setText(cap1timetext);
                        std::cout<<"Inloop"<<std::endl;
                        capture_toggle=0;

                        flag_capture_send=12;
                        ui.capture_pushButton->setStyleSheet("qproperty-icon: url(:/ic_capture_off.png);");
                        ui.capture_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                            "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                     );
                    }

                }



//                ui.capture_lineEdit_1->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
//                ui.capture_lineEdit_2->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");
//                ui.capture_lineEdit_3->setStyleSheet("QLineEdit { border: 2px solid #339bff; border-radius: 4px; background-color: #191b29; color: #339bff}");

                double cap_data_buffer[4]={0,0,0,0};

                /*for(int i=0; i<25; i++)
                {
                    cap_data_buffer[0]+=capture_data.Cap_GetLine[0+i];//int
                    cap_data_buffer[1]+=capture_data.Cap_GetLine[25+i];//int
                    cap_data_buffer[2]+=capture_data.Cap_GetLine[50+i];//int
                    cap_data_buffer[3]+=capture_data.Cap_GetLine[75+i];//int
                }*/

//                    capture_data.Cap_GetFlow[i];//int
//                    capture_data.Cap_GetCapTime[i];//int
//                    capture_data.Cap_GetIntegFlow[i];//int
                cap_data_buffer[0]=(capture_data.Cap_GetIntegFlowRate[0]);
//                cap_data_buffer[1]=(capture_data.Cap_GetIntegFlowRate[1]);
//                cap_data_buffer[2]=(capture_data.Cap_GetIntegFlowRate[2]);
//                cap_data_buffer[3]=(capture_data.Cap_GetIntegFlowRate[3]);
                //std::cout<<"cap0 "<<cap_data_buffer[0]<<capture_data.Cap_GetIntegFlowRate[0]<<std::endl;
                QString dispTime0=QString("%1")
                        .arg(cap_data_buffer[1], 6, 'f', 2);
                QString dispTime1=QString("%1")
                        .arg(cap_data_buffer[1], 6, 'f', 2);
                QString dispTime2=QString("%1")
                        .arg(cap_data_buffer[2], 6, 'f', 2);
                QString dispTime3=QString("%1")
                        .arg(cap_data_buffer[3], 6, 'f', 2);
//                ui.capture_lineEdit_1->setText(dispTime1);
//                ui.capture_lineEdit_2->setText(dispTime2);
//                ui.capture_lineEdit_3->setText(dispTime3);

                /*///////////////////////////////////////////////////////////*/


                cap1_old=capture_data.Cap_GetIntegFlowRate[0];
            }

            void MainWindow::CtoR_updateJDataView() {

              if(joy_message_g.buttons[0] > 0.5) // X
                set_joy_pixmap_image(ui.X, ON);
              else
                set_joy_pixmap_image(ui.X, OFF);

              if(joy_message_g.buttons[1]>0.5) // circle
                set_joy_pixmap_image(ui.circle, ON);
              else
                set_joy_pixmap_image(ui.circle, OFF);

              if(joy_message_g.buttons[2]>0.5) // triangle
                set_joy_pixmap_image(ui.triangle, ON);
              else
                set_joy_pixmap_image(ui.triangle, OFF);

              if(joy_message_g.buttons[3]>0.5) // square
                set_joy_pixmap_image(ui.square, ON);
              else
                set_joy_pixmap_image(ui.square, OFF);

              if(joy_message_g.buttons[8]>0.5) // share
                set_joy_pixmap_image(ui.bttn_share, ON);
              else
                set_joy_pixmap_image(ui.bttn_share, OFF);

              if(joy_message_g.buttons[9]>0.5) // options
                set_joy_pixmap_image(ui.bttn_options, ON);
              else
                set_joy_pixmap_image(ui.bttn_options, OFF);


              if(joy_message_g.buttons[4]>0.5) //L1
                set_joy_pixmap_image(ui.L1, ON);
              else
                set_joy_pixmap_image(ui.L1, OFF);
              if(joy_message_g.buttons[6]>0.5) //L2
                set_joy_pixmap_image(ui.L2, ON);
              else
                set_joy_pixmap_image(ui.L2, OFF);
              if(joy_message_g.buttons[5]>0.5) //R1
                set_joy_pixmap_image(ui.R1, ON);
              else
                set_joy_pixmap_image(ui.R1, OFF);
              if(joy_message_g.buttons[7]>0.5) //R2
                set_joy_pixmap_image(ui.R2, ON);
              else
                set_joy_pixmap_image(ui.R2, OFF);


             // joy_message_g.axes[0];  //left stick (left : 1 , right -1)
             // joy_message_g.axes[1];  //left stick (up : 1 , down : -1)
             // joy_message_g.axes[3];  //right stick (left : 1 , right : -1)
             // joy_message_g.axes[4];  //right stick (up : 1 , down : -1)

              if(joy_message_g.axes[6]>0.5) // button (l : 1 , r : -1)
                set_joy_pixmap_image(ui.bttn_l, ON);
              else
                set_joy_pixmap_image(ui.bttn_l, OFF);

              if(joy_message_g.axes[6]<-0.5)
                set_joy_pixmap_image(ui.bttn_r, ON);
              else
                set_joy_pixmap_image(ui.bttn_r, OFF);

              if(joy_message_g.axes[7]>0.5) // button (u : 1 , d : -1)
                set_joy_pixmap_image(ui.bttn_u, ON);
              else
                set_joy_pixmap_image(ui.bttn_u, OFF);

              if(joy_message_g.axes[7]<-0.5)
                set_joy_pixmap_image(ui.bttn_d, ON);
              else
                set_joy_pixmap_image(ui.bttn_d, OFF);

              set_joy_lcd_number(ui.header_sec_lcdNumber, ((int)(joy_message_g.header.stamp.sec))%1000000000); //can't display number over 999999999
              set_joy_lcd_number(ui.header_nsec_lcdNumber, (int)(joy_message_g.header.stamp.nsec));

              if(fabs(joy_message_g.axes[0])>0.01 or fabs(joy_message_g.axes[1])>0.01)
                set_joy_pixmap_image(ui.L, ON);
              else
                set_joy_pixmap_image(ui.L, OFF);

              if(fabs(joy_message_g.axes[3])>0.01 or fabs(joy_message_g.axes[4])>0.01)
                set_joy_pixmap_image(ui.R, ON);
              else
                set_joy_pixmap_image(ui.R, OFF);

              set_joy_lcd_number(ui.ls_x_lcdNumber, (double)(joy_message_g.axes[1]));
              set_joy_lcd_number(ui.ls_y_lcdNumber, (double)(joy_message_g.axes[0]));
              set_joy_lcd_number(ui.rs_x_lcdNumber, (double)(joy_message_g.axes[4]));
              set_joy_lcd_number(ui.rs_y_lcdNumber, (double)(joy_message_g.axes[3]));

            }


            void MainWindow::RtoC_updateJDataView() {
            }


            /*****************************************************************************
            ** Implementation [Menu]
            *****************************************************************************/

            void MainWindow::on_actionAbout_triggered() {
                QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
            }

            /*****************************************************************************
            ** Implementation [Configuration]
            *****************************************************************************/

            void MainWindow::ReadSettings() {
                QSettings settings("Qt-Ros Package", "kmu_gui");
                restoreGeometry(settings.value("geometry").toByteArray());
                restoreState(settings.value("windowState").toByteArray());
                QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
                QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
                //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
                ui.comm_connect_line_edit_master->setText(master_url);
                ui.comm_connect_line_edit_host->setText(host_url);
                //ui.line_edit_topic->setText(topic_name);
                bool remember = settings.value("remember_settings", false).toBool();
                ui.comm_checkbox_remember_settings->setChecked(remember);
                bool checked = settings.value("use_environment_variables", false).toBool();
                ui.comm_checkbox_use_environment->setChecked(checked);
                if ( checked ) {
                    ui.comm_connect_line_edit_master->setEnabled(false);
                    ui.comm_connect_line_edit_host->setEnabled(false);
                    //ui.line_edit_topic->setEnabled(false);
                }
            }

            void MainWindow::WriteSettings() {
                QSettings settings("Qt-Ros Package", "kmu_gui");
                settings.setValue("master_url",ui.comm_connect_line_edit_master->text());
                settings.setValue("host_url",ui.comm_connect_line_edit_host->text());
                //settings.setValue("topic_name",ui.line_edit_topic->text());
                settings.setValue("use_environment_variables",QVariant(ui.comm_checkbox_use_environment->isChecked()));
                settings.setValue("geometry", saveGeometry());
                settings.setValue("windowState", saveState());
                settings.setValue("remember_settings",QVariant(ui.comm_checkbox_remember_settings->isChecked()));
            }

            void MainWindow::closeEvent(QCloseEvent *event) {
                WriteSettings();
                QMainWindow::closeEvent(event);
            }


            void MainWindow::on_slam_reset_pushButton_pressed()
            {
                ui.slam_reset_pushButton->setStyleSheet("QPushButton{ border: 1px solid #339bff; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );
            }
            void MainWindow::on_slam_reset_pushButton_released()
            {
                ui.slam_reset_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );
            }
            void MainWindow::on_slam_help_pushButton_pressed()
            {
                ui.slam_help_pushButton->setStyleSheet("QPushButton{ border: 1px solid #339bff; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );
            }

            void MainWindow::on_slam_help_pushButton_released()
            {
                ui.slam_help_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );

            }

            void MainWindow::on_slam_save_pushButton_pressed()
            {
                ui.slam_save_pushButton->setStyleSheet("QPushButton{ border: 1px solid #339bff; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );
            }
            void MainWindow::on_slam_save_pushButton_released()
            {
                ui.slam_save_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );

            }

            void MainWindow::on_slam_gas_pushButton_pressed()
            {
                ui.slam_gas_pushButton->setStyleSheet("QPushButton{ border: 1px solid #339bff; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );
            }
            void MainWindow::on_slam_gas_pushButton_released()
            {
                ui.slam_gas_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                        "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                        //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                        //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                        );

            }



            void MainWindow::on_capture_button_pressed()
            {

                //flag_capture_send=3;
                std::cout<<"oncapbtnclicked";

            }



            void MainWindow::on_capture_button_clicked()
            {


            }


            void MainWindow::on_gas_pushButton_clicked()
            {
                flag_gas_send=1;

                ui.gas_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                     "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                     //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                     //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                     );


            }

            void MainWindow::on_capture_pushButton_clicked()
            {
                flag_capture_send=12;

                ui.capture_pushButton->setStyleSheet("QPushButton{ border: none; }\n"
                                                     "QPushButton{ background-color:rgb(42, 45, 67); }\n"
                                                     //"QPushButton:enabled { background-color: rgb(200,0,0); }\n"
                                                     //"QPushButton:selected { background-color: rgb(200,0,0); }\n"
                                                     );

                std::cout<<"oncappushclicked";

            }

            void MainWindow::init_joy_resource_map(){
              QDirIterator it(":/joy_tab", QDirIterator::Subdirectories);
              std::string name;
              int i=0;
              while (it.hasNext()) {
                  //qDebug() << it.next();
                QFile f(it.next());
                name = f.fileName().toStdString();
                std::string name_only = name.substr(10,name.length()-14);// to remove ":/joy_tab/" and ".png"
                joy_tap_names.insert({name_only,i});
                //std::cout<<name_only<<std::endl;
                i++;
              }

              for(auto itr = joy_tap_names.begin(); itr != joy_tap_names.end(); itr++){

                  //cout<< itr->first << " " << itr->second << endl;
                  std::string resource_path = ":/joy_tab/" + itr->first + ".png";
                  joy_img[itr->second].load(resource_path.c_str());
              }
            }

            void MainWindow::set_joy_pixmap_image(const QObject *joy_object, int on_off){ // 0 : off, 1 : on
               std::string label_name;
               label_name = joy_object->objectName().toStdString();
               label_name = label_name+((on_off==0)?std::string("_off"):std::string("_on"));
               ((QLabel*)joy_object)->setPixmap(joy_img[joy_tap_names[label_name]]);
            }
            void MainWindow::set_joy_lcd_number(const QObject *joy_object, int num){ // 0 : off, 1 : on
               ((QLCDNumber*)joy_object)->display(num);
            }
            void MainWindow::set_joy_lcd_number(const QObject *joy_object, double num){ // 0 : off, 1 : on
               ((QLCDNumber*)joy_object)->display(num);
            }

            void MainWindow::initialize_joy_tab(){
                set_joy_pixmap_image(ui.L,  OFF);
                set_joy_pixmap_image(ui.L1,  OFF);
                set_joy_pixmap_image(ui.L2,  OFF);
                set_joy_pixmap_image(ui.L3,  OFF);
                set_joy_pixmap_image(ui.L_T, OFF);
                set_joy_pixmap_image(ui.R,  OFF);
                set_joy_pixmap_image(ui.R1,  OFF);
                set_joy_pixmap_image(ui.R2,  OFF);
                set_joy_pixmap_image(ui.R3,  OFF);
                set_joy_pixmap_image(ui.R_T, OFF);
                set_joy_pixmap_image(ui.L2_R2, OFF);
                set_joy_pixmap_image(ui.L3_R3, OFF);
                set_joy_pixmap_image(ui.bttn_l, OFF);
                set_joy_pixmap_image(ui.bttn_r, OFF);
                set_joy_pixmap_image(ui.bttn_u, OFF);
                set_joy_pixmap_image(ui.bttn_d, OFF);
                set_joy_pixmap_image(ui.circle, OFF);
                set_joy_pixmap_image(ui.triangle, OFF);
                set_joy_pixmap_image(ui.square, OFF);
                set_joy_pixmap_image(ui.X, OFF);
                set_joy_pixmap_image(ui.bttn_share, OFF);
                set_joy_pixmap_image(ui.bttn_options, OFF);
            }


        }  // namespace kmu_gui







