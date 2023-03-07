/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
//#include <QtWebKit/QWebView>

//#include <QDesktopWidget>
#include "qdesktopwidget.h"

#include <QApplication>
#include "../include/kmu_gui/main_window.h"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    ROS_INFO("START");
    QApplication app(argc, argv);
//    QWebView view;
//    view.show();
//    view.setUrl(QUrl("http://google.com"));

    kmu_gui::MainWindow w(argc,argv);

   /*
    QRect screenGeometry =w.screenGeometry();
    int w_height=screenGeometry.height();
    int w_width=screenGeometry.width();

    int x=(w_width-w.width())/2.0;
    int y=(w_height-w.height())/2.0;

    w.setGeometry(x,y,w.width(), w.height());

*/
    std::cout<<"jeon";
     QResource::registerResource("images.qrc");
    w.setGeometry(0,0,2560,1600);
    //std::cout<<"x,y"<<x<<" "<<y<<std::endl;

    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    int result = app.exec();

    return result;
}
