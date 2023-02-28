#include <iostream>
#include <QTextStream>
#include <string>
#include "mksocketserver.h"

using namespace std;

mkSocketServer::mkSocketServer(QObject *parent) : QTcpServer(parent)
{


    // whenever a user connects, it will emit signal

}

void mkSocketServer::startServer(int port)
{
    if(!this->listen(QHostAddress::Any, port))
    {
        qDebug() << "Could not start server";
    }
    else
    {
        qDebug() << "Listening to port " << port << "...";
    }
}

mkSocketServer::~mkSocketServer()
{

}

void mkSocketServer::readyRead_slot()
{
    arrBuffer.append(socket->readAll());
   // qDebug() << "Reading... from socket serer"<<arrBuffer;//<<data;
    while(1)
    {
        size_t n = arrBuffer.toStdString().find('\n');
        if (n != std::string::npos)// found one command...
        {
            ringBuffer.write(arrBuffer.data(), n);
            //ringBuffer.print();
            arrBuffer.remove(0,n+1);
        }
        else break;
    }}



void mkSocketServer::disconnected_slot()
{
    isConnected=false;
    socket->deleteLater();
    onlyOneThread=true;
}


void mkSocketServer::incomingConnection(qintptr socketDescriptor)
{
    // We have a new connection
    if(onlyOneThread)
    {
        qDebug() << socketDescriptor << " Connecting...";
        socket = new QTcpSocket();

        // set the ID
        if(!socket->setSocketDescriptor(socketDescriptor))
        {
            // something's wrong, we just emit a signal
            emit error(socket->error());
            return;
        }

        connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead_slot()), Qt::DirectConnection);
        connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected_slot()));


        qDebug() << socketDescriptor << " Client connected";
        socket->write("Welcome to IStacker Server!\n");
        onlyOneThread=false;
        emit connected();
    }
}
