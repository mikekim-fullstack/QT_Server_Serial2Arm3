#include "mksocketclient.h"
#include <string>
#include <iostream>
mkSocketClient::mkSocketClient(QObject *parent) :
    QObject(parent)
{
    socket = new QTcpSocket(this);

    connect(socket, &QTcpSocket::connected, this, &mkSocketClient::connected);
    connect(socket,  &QTcpSocket::disconnected, this, &mkSocketClient::disconnected);
    connect(socket, &QTcpSocket::readyRead, this, &mkSocketClient::readyRead);
    connect(socket, &QTcpSocket::bytesWritten, this, &mkSocketClient::bytesWritten);

    std::cout << "\033[37;40m------------- Connecting to server ----------------\033[0m\n"<<std::flush;


}

void mkSocketClient::connect2Server(QString address)
{
    socket->connectToHost(address, 8888);
    serverAddress = address;
    if(!socket->waitForConnected(2000))
    {
        qDebug() << "Error--: " << socket->errorString();
    }
}

void mkSocketClient::write(QString str)
{
    socket->write(str.toStdString().c_str());
}


void mkSocketClient::connected()
{
     std::cout << "\033[37;40m\t!!!   Connected to server:"<<serverAddress.toStdString().c_str()<<" !!!\033[0m\n\n"<<std::flush;

   // socket->write("Hello mkServer from client;");
}

void mkSocketClient::disconnected()
{
    //qDebug() << "Disconnected!";
}

void mkSocketClient::bytesWritten(qint64 bytes)
{
    //qDebug() << "We wrote: " << bytes;
}

void mkSocketClient::readyRead()
{
    //qDebug() << "Reading... from socket serer";
    while(!socket->atEnd())
    {
       // qDebug() << socket->readAll();
        arrBuffer +=socket->readAll();
        size_t at = arrBuffer.toStdString().find(';');
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
}
