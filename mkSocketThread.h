#ifndef MKSOCKETTHREAD_H
#define MKSOCKETTHREAD_H
#include <QTcpSocket>
#include <QThread>
#include <QByteArray>
#include "../sharedfiles/mkringbuffer.h"
class mkSocketThread : public QThread
{
    Q_OBJECT
public:
   explicit mkSocketThread(qintptr ID, QObject *parent = 0);
public:
    mkRingBuffer ringBuffer;
    QTcpSocket *socket;
private:

    qintptr socketDescriptor;
    QByteArray arrBuffer;

signals:
    void error(QTcpSocket::SocketError socketerror);
public slots:
    void readyRead();
    void disconnected();
    // QThread interface
protected:
    virtual void run() override;
public:
    void write(char * str);
};



#endif // MKSOCKETTHREAD_H
