#include <iostream>
#include <QTextStream>
#include <string>
#include "mkSocketThread.h"
using namespace std;
mkSocketThread::mkSocketThread(qintptr ID, QObject *parent):
    QThread(parent)
{
    this->socketDescriptor = ID;
}

void mkSocketThread::readyRead()
{
    // get the information
    arrBuffer = socket->readAll();
    cout<<">>>>> Received::\033[1;36m"<< arrBuffer.toStdString().c_str()<<"\033[0m"<<endl<<flush;
    while(1)
    {
        size_t at = arrBuffer.toStdString().find('\n');
        if (at != std::string::npos)// found one command...
        {
            char *data = arrBuffer.data();
            char tmp[128];
            ringBuffer.write(data, at);
            //ringBuffer.print();
            strcpy(tmp,data+at+1);
            arrBuffer = QByteArray(tmp);

        }
        else break;
    }

   // socket->write(Data);
    ////////////////////////////////
    // transTcpBuffer
}

void mkSocketThread::disconnected()
{
    qDebug() << socketDescriptor << " Disconnected";


        socket->deleteLater();
        exit(0);
}

void mkSocketThread::run()
{
    // thread starts here
        qDebug() << " Thread started";

        socket = new QTcpSocket();

        // set the ID
        if(!socket->setSocketDescriptor(this->socketDescriptor))
        {
            // something's wrong, we just emit a signal
            emit error(socket->error());
            return;
        }

        // connect socket and signal
        // note - Qt::DirectConnection is used because it's multithreaded
        //        This makes the slot to be invoked immediately, when the signal is emitted.

        connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()), Qt::DirectConnection);
        connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected()));

        // We'll have multiple clients, we want to know which is which
        qDebug() << socketDescriptor << " Client connected";
        socket->write("Welcome to IStacker Server!;");

        // make this thread a loop,
        // thread will stay alive so that signal/slot to function properly
        // not dropped out in the middle when thread dies

        exec();
}

void mkSocketThread::write(char *str)
{
    if(socket)socket->write(str);
}
