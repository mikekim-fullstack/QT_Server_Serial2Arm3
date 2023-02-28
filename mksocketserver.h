#ifndef MKSOCKETSERVER_H
#define MKSOCKETSERVER_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>
#include <QDebug>
#include "../sharedfiles/mkringbuffer.h"
class mkSocketThread;
class mkSocketServer : public QTcpServer
{
    Q_OBJECT
public:
    mkRingBuffer ringBuffer;
    QTcpSocket *socket=0;
    qintptr socketDescriptor;
    QByteArray arrBuffer;
   // mkSocketThread *socketHandler=0;
    bool onlyOneThread=true;
    volatile bool isConnected=false;
public:
    explicit mkSocketServer(QObject *parent = nullptr);
    void startServer(int port=8888);
    virtual ~mkSocketServer() override;
signals:
     void error(QTcpSocket::SocketError socketerror);
    void connected();
public slots:
    void readyRead_slot();
    void disconnected_slot();

    // QTcpServer interface
protected:
    virtual void incomingConnection(qintptr socketDescriptor) override;
};



#endif // MKSOCKETSERVER_H
