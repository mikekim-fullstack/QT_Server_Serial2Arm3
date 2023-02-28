#ifndef MKSOCKETCLIENT_H
#define MKSOCKETCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include "../sharedfiles/mkringbuffer.h"
class mkSocketClient: public QObject
{
    Q_OBJECT
public:
    explicit mkSocketClient(QObject *parent = 0);
    ~mkSocketClient()
    {
        if(socket)
        {

            socket->disconnect();
            socket->disconnectFromHost();
            socket->deleteLater();
            delete socket;
            qDebug()<<"deleting client socket!";
        }

    }
    void connect2Server(QString address="10.88.111.3");
    void write(QString str);

    signals:

    public slots:
        void connected();
        void disconnected();
        void bytesWritten(qint64 bytes);
        void readyRead();

public:
        mkRingBuffer ringBuffer;
private:
    QTcpSocket *socket=0;
    QString serverAddress="10.88.111.3";
    QByteArray arrBuffer;
};

#endif // MKSOCKETCLIENT_H
