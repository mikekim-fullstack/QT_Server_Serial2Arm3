#include "mkserialport.h"

/****************************************************************************
**
** Copyright (C) 2020 Mike Kim <ZoeRobotics>
** Added Aug. 21, 2020
**
****************************************************************************/
#include <string>
#include <iostream>
#include <QDebug>
#include "mkserialport.h"
using namespace std;

mkSerialPort::mkSerialPort(QSerialPort *serialPort, QObject *parent) :
    QObject(parent),
    m_serialPort(serialPort),
    m_standardOutput(stdout)
{
   // m_timer.setSingleShot(true);
    connect(m_serialPort, &QSerialPort::bytesWritten,this, &mkSerialPort::handleBytesWritten);
    connect(m_serialPort, &QSerialPort::errorOccurred,  this, &mkSerialPort::handleError);
    connect(m_serialPort, &QSerialPort::readyRead, this, &mkSerialPort::readyRead);
   // connect(&m_timer, &QTimer::timeout, this, &mkSerialPort::handleTimeout);
}

void mkSerialPort::handleBytesWritten(qint64 bytes)
{
    m_bytesWritten += bytes;
    if (m_bytesWritten == m_writeData.size()) {
        m_bytesWritten = 0;
//        m_standardOutput << QObject::tr("Data successfully sent to port %1")
//                            .arg(m_serialPort->portName()) << endl;
    }
}

void mkSerialPort::handleTimeout()
{
    m_standardOutput << QObject::tr("Operation timed out for port %1, error: %2")
                        .arg(m_serialPort->portName())
                        .arg(m_serialPort->errorString())
                     << Qt::endl;
}

void mkSerialPort::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::WriteError) {
        m_standardOutput << QObject::tr("An I/O error occurred while writing"
                                        " the data to port %1, error: %2")
                            .arg(m_serialPort->portName())
                            .arg(m_serialPort->errorString())
                         << Qt::endl;
    }
}

void mkSerialPort::readyRead()
{
    arrBuffer.append(m_serialPort->readAll());
    //qDebug() << "Reading... from socket serer"<<arrBuffer;//<<data;
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
    }
}

void mkSerialPort::write(const QByteArray &writeData)
{
    m_writeData = writeData;

    const qint64 bytesWritten = m_serialPort->write(writeData);

    if (bytesWritten == -1) {
        m_standardOutput << QObject::tr("Failed to write the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(),m_serialPort->errorString());


    } else if (bytesWritten != m_writeData.size()) {
        m_standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(), m_serialPort->errorString());

    }
}
