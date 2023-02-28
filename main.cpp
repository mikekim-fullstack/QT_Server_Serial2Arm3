#include <QCoreApplication>
#include <QtDebug>

#include <QFile>
#include <QDataStream>

#include <QStringList>
#include <QTextStream>
#include <QThread>
#include "mkserver_serial2arm3.h"
//#include "../sharedfiles/mkglobalclass.h"
//#include "../sharedfiles/mkRobotKin.h"
#include "helper.h"


#include <QtDebug>
#include <signal.h>


void myQuit()
{
    cout<<"I'm quiting!"<<endl;
}
int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    mkServer_Serial2Arm3 myMain;
    ////////////////////////////////////////////
    // So important to call the exit of app...
    signal(SIGTERM, [](int sig){ qApp->quit(); });
    QTimer::singleShot(10, &myMain, SLOT(startRun()));
    return app.exec();
}
