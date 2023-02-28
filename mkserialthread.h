#ifndef MKSERIALTHREAD_H
#define MKSERIALTHREAD_H
#include "mkserialport.h"
#include <QThread>
#include "mkringbuffer.h"
#include <QByteArray>
#include <iostream>
#include <QDebug>
#include <string>
#include "mkGlobalDefine.h"
//#include "mkclient_serial2arm3.h"
using namespace std;
class mkSerialThread : public QThread
{
    Q_OBJECT
public:
    QObject *parent=0;
    mkRingBuffer ringBuffer;

    QSerialPort *m_serialPort = nullptr;
    QByteArray m_writeData;
    QTextStream m_standardOutput;
    qint64 m_bytesWritten = 0;

     QByteArray arrBuffer;

public:
    // -------------------------------- mkSerialThread -------------------------------
    explicit mkSerialThread(QSerialPort *serialPort, QObject *parent = nullptr):
        QThread(nullptr), m_serialPort(serialPort) , m_standardOutput(stdout)
    {
        this->parent = parent;
    }

   // --------------------------------- write() -------------------------------------
    void write(const QByteArray &writeData)
    {
        m_writeData = writeData;

        const qint64 bytesWritten = m_serialPort->write(writeData);

        if (bytesWritten == -1) {
            m_standardOutput << QObject::tr("Failed to write the data to port %1, error: %2")
                                .arg(m_serialPort->portName())
                                .arg(m_serialPort->errorString())
                             << endl;
        } else if (bytesWritten != m_writeData.size()) {
            m_standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2")
                                .arg(m_serialPort->portName())
                                .arg(m_serialPort->errorString())
                             << endl;
        }

    }
    // ---------------------------------  -------------------------------------
public slots:
    //void readyRead();
  // void disconnected();
    // ---------------------------- readyRead ---------------------------------
    void readyRead()
    {

        while(!m_serialPort->atEnd())
        {
            arrBuffer +=m_serialPort->readAll();
            qDebug()<<"Original:"<<arrBuffer.size()<<arrBuffer;
            size_t at = arrBuffer.toStdString().find('\n');
            if (at != string::npos)// found one command...
            {
                char *data = arrBuffer.data();
                char tmp[128];
                ringBuffer.write(data,at);
                strcpy(tmp,data+at+1);
                arrBuffer = QByteArray(tmp);


            }
            else break;
        }
    }
    // ---------------------------- handleBytesWritten ---------------------------------
    void handleBytesWritten(qint64 bytes)
    {
        m_bytesWritten += bytes;
        if (m_bytesWritten == m_writeData.size()) {
            m_bytesWritten = 0;
        }
    }
    //----------------------------- handleError  ----------------------------------------------
    void handleError(QSerialPort::SerialPortError serialPortError)
    {
        if (serialPortError == QSerialPort::WriteError) {
            m_standardOutput << QObject::tr("An I/O error occurred while writing"
                                            " the data to port %1, error: %2")
                                .arg(m_serialPort->portName())
                                .arg(m_serialPort->errorString())
                             << endl;
        }
    }
    //----------------------------- serialAboutClose  ----------------------------------------------
    void serialAboutClose()
    {
        //static_cast<mkClient_Serial2Arm3*>(parent)->serialAboutClose();
    }
    // QThread interface
protected:
    virtual void run() override {
      //  connect(m_serialPort, &QSerialPort::bytesWritten, this, &mkSerialThread::handleBytesWritten);
        connect(m_serialPort, &QSerialPort::errorOccurred, this, &mkSerialThread::handleError);
        connect(m_serialPort, &QSerialPort::aboutToClose, this, &mkSerialThread::serialAboutClose);
        connect(m_serialPort, &QSerialPort::readyRead, this, &mkSerialThread::readyRead);

        //exec();

        while(1) {
            if(ringBuffer.isEmpty()) continue;
            cout<<"received cmd: "<<ringBuffer.getCmd()<<endl<<flush;
            ringBuffer.readDone();

            char packet[128];
            for(int i=0; i<1; i++) {
                sprintf(packet, "G%d M%d P%d;",SC_SETPOS, i, 0);
                QByteArray data = QByteArray((char*)packet, strlen(packet));
                qDebug()<<"Sending(set Position) data:"<<data;
                m_serialPort->write(data);
            }
        }
    }
};



#endif // MKSERIALTHREAD_H
