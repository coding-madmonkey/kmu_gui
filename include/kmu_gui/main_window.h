/**
 * @file /include/rok3_qt_gui/main_window.hpp
 *
 * @brief Qt based gui for rok3_qt_gui.
 *
 * @date November 2010
 **/
#ifndef kmu_gui_MAIN_WINDOW_H
#define kmu_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QStringListModel>
#include <QThread>
#include "ui_main_window.h"
#include "qnode.h"

#include <map>
#include <string>

#define OFF 0
#define  ON 1

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kmu_gui {

	/*****************************************************************************
	** Interface [MainWindow]
	*****************************************************************************/
	/**
	 * @brief Qt central, all operations relating to the view part here.
	 */
		class MainWindow : public QMainWindow {
			Q_OBJECT

		public:
			MainWindow(int argc, char** argv, QWidget *parent = 0);
			~MainWindow();

		void ReadSettings(); // Load up qt program settings at startup
		void WriteSettings(); // Save qt program settings when closing

		void closeEvent(QCloseEvent *event); // Overloaded function
        void showNoMasterMessage();
		void showButtonTestMessage();

		public Q_SLOTS:
		/******************************************
		** Auto-connections (connectSlotsByName())
		*******************************************/
		void on_actionAbout_triggered();
        void on_comm_connect_button_clicked(bool check);
                //void on_button_test_clicked(bool check);
        void on_comm_checkbox_use_environment_stateChanged(int state);

        void on_slam_reset_pushButton_pressed();
        void on_slam_reset_pushButton_released();
        void on_slam_help_pushButton_pressed();
        void on_slam_help_pushButton_released();
        void on_slam_save_pushButton_pressed();
        void on_slam_save_pushButton_released();
        void on_slam_gas_pushButton_pressed();
        void on_slam_gas_pushButton_released();

        void on_gas_pushButton_clicked();
        void on_capture_pushButton_clicked();

        void on_capture_button_pressed();
        void on_capture_button_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
//                void updateLoggingView(); // no idea why this can't connect automatically
    void mainUIupdate(const double);
    void updateCameraView(const QImage &);
    void updateRviz_topView(const QImage &);
    void updateRviz_tpfView(const QImage &);
    void updateMotorDataView();


    void updateBatteryDataView();

    void updateFanDataView();



    void RtoC_updateJDataView();
    void CtoR_updateJDataView();

    void gas_updateDataView();
    void capture_updateDataView();






        private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QStringListModel* logging_model;

    QString gasText_normal, gas0Text,gas1Text,gas2Text,gas3Text;
    QString cap1timetext;
    QPixmap m_lightimg[2];
    QPixmap m_switchimg[2];

    QPixmap slam_icon_img;
    QPixmap cam_icon_img;
    QPixmap top_header_img;
    QPixmap kiro_img;
    QPixmap kmu_img;
    QPixmap rclab_img;

    QPixmap comm_off_img;
    QPixmap comm_on_img;

    QPixmap power_off0_img;
    QPixmap power_00_img;
    QPixmap power_20_img;
    QPixmap power_40_img;
    QPixmap power_60_img;
    QPixmap power_80_img;
    QPixmap power_100_img;

    QPixmap motor_on_img;
    QPixmap motor_off_img;

    QPixmap fan_off_img;
    QPixmap fan_stop_img;
    QPixmap fan_step_1_img;
    QPixmap fan_step_2_img;
    QPixmap fan_step_3_img;
    QPixmap fan_step_4_img;

    QPixmap gas_led_red;
    QPixmap gas_led_yellow;
    QPixmap gas_led_blue;
    QPixmap gas_led_black;
    QPixmap gas_on_img;
    QPixmap gas_off_img;

    QPixmap capture_on_img;
    QPixmap capture_off_img;
    QPixmap slam_on_img;
    QPixmap slam_off_img;
    QPixmap slam_reset_img;
    QPixmap slam_help_img;
    QPixmap slam_save_img;
    QPixmap slam_gas_img;


    std::map<std::string,int> joy_tap_names;
    QPixmap joy_img[60];
    void init_joy_resource_map();
    void set_joy_pixmap_image(const QObject *joy_object, int on_off);
    void set_joy_lcd_number(const QObject *joy_object, int num);
    void set_joy_lcd_number(const QObject *joy_object, double num);
    void initialize_joy_tab();
	};

        }  // namespace kmu_gui

        #endif // kmu_gui_MAIN_WINDOW_H
