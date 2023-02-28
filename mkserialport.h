#ifndef MKSERIALPORT_H
#define MKSERIALPORT_H



#include <QByteArray>
#include <QObject>
#include <QSerialPort>
#include <QTextStream>
#include <QTimer>
#include "../sharedfiles/mkringbuffer.h"
QT_BEGIN_NAMESPACE

QT_END_NAMESPACE

class mkSerialPort : public QObject
{
    Q_OBJECT

public:
    explicit mkSerialPort(QSerialPort *serialPort, QObject *parent = nullptr);
    void write(const QByteArray &writeData);

private slots:
    void handleBytesWritten(qint64 bytes);
    void handleTimeout();
    void handleError(QSerialPort::SerialPortError error);
    void readyRead();

public:
        mkRingBuffer ringBuffer;
private:
    QSerialPort *m_serialPort = nullptr;
    QByteArray m_writeData;
    QTextStream m_standardOutput;
    qint64 m_bytesWritten = 0;
    QTimer m_timer;
    QByteArray arrBuffer;
};




#endif // MKSERIALPORT_H
