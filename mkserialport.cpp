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
#include <QTime>
#include <QLibraryInfo>
#include <QCoreApplication>
#include "mkserialport.h"
using namespace std;
static unsigned char DEC2HEX[]={
     0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
     0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
     0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
     0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
     0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
     0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
     0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
     0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
     0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
     0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
     0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
     0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
     0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
     0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
     0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};
static const unsigned short CRC16_XMODEM_TABLE[256] =
    { // CRC-16/XMODEM TABLE
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};
static inline unsigned short calculateCRC16Xmodem(const char *buf, int len)
{
  volatile int counter;
  volatile unsigned short crc = 0;
  for (counter = 0; counter < len; counter++)
    crc = (crc << 8) ^ CRC16_XMODEM_TABLE[((crc >> 8) ^ *(char *)buf++) & 0x00FF];
  return crc;
}
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
    connect(&serialWriteTimer, &QTimer::timeout, this, &mkSerialPort::processSerialPortWriteTimer);
    serialWriteTimer.start(1);
}

void mkSerialPort::processSerialPortWriteTimer()
{
//
    if(writeBuffer.isEmpty()) return;

//    if(!writeBuffer.bReadyToWrite) return;
//    writeBuffer.bReadyToWrite=false;
//    cout<<"processSerialPortWriteTimer"<<endl;

    QByteArray writeData;
    if(writeBuffer.bErrorOccered)
    {
        writeBuffer.bErrorOccered=false;
        cout<<"resending: cmd="<<writeBuffer.backupBuffer<<endl;
         writeData = QByteArray(writeBuffer.backupBuffer, strlen(writeBuffer.backupBuffer));
    }
    else {
         writeData = QByteArray(writeBuffer.getCmd(),writeBuffer.getLengthStr());
    }


    writeData.append(';');

    // ...Just print out...
    //qDebug()<<"write: "<<writeData;
    bool found13=false;
    if(writeBuffer.codeSeen('G') && writeBuffer.codeValue()==13){
        found13=true;
    }
    if(!found13)cout<<"write: "<<writeData.data()<<endl;
    //.....................

    const qint64 bytesWritten = m_serialPort->write(writeData);


    bool bFlush = m_serialPort->flush();
    if(!bFlush){

        bFlush = m_serialPort->flush();
    }

    if (bytesWritten == -1 || !bFlush) {
        m_standardOutput << QObject::tr("Failed to write the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(),m_serialPort->errorString());


    } else if (bytesWritten != writeData.size() || !bFlush) {
        m_standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(), m_serialPort->errorString());

    }
    writeBuffer.readDone();
//    writeBuffer.bReadyToWrite=false;

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
//    qDebug() << "Reading... from serial"<<arrBuffer;//<<data;
    while(1)
    {
        size_t n = arrBuffer.toStdString().find('\n');
        if (n != std::string::npos)// found one command...
        {
            readBuffer.write(arrBuffer.data(), n);
           if( QLibraryInfo::isDebugBuild())  readBuffer.print();
            arrBuffer.remove(0,n+1);
        }
        else break;
    }
}



void mkSerialPort::write(const QByteArray &writeData)
{
    m_writeData = writeData;

    const qint64 bytesWritten = m_serialPort->write(writeData);
    bool bFlush = m_serialPort->flush();
    if(!bFlush){

        bFlush = m_serialPort->flush();
    }

    if (bytesWritten == -1 || !bFlush) {
        m_standardOutput << QObject::tr("Failed to write the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(),m_serialPort->errorString());


    } else if (bytesWritten != m_writeData.size() || !bFlush) {
        m_standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(), m_serialPort->errorString());

    }
    // delay 20ms
//    QTime dieTime= QTime::currentTime().addMSecs(20);
//    while (QTime::currentTime() < dieTime)
//        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
}

void mkSerialPort::write_crc(const char *cmd, int len)
{
    writeBuffer.write((char*)cmd, len);
    QByteArray writeData = QByteArray(cmd,len);
    writeData.append(';');
//    qDebug()<<"write: "<<writeData;
return;
/*
//    m_writeData = writeData;

    // ....1. Beginning ...
       char frame[128] ;

       frame[0]=0xFF;
       frame[1]=(DEC2HEX[len+4]);
       int N=0;
       // ...2. Data ...
       for(N=0;N<len; N++){
           frame[2+N]=DEC2HEX[(unsigned char)(cmd[N])];
       }
       // ....3. add CRC to the end ...
       unsigned short crc = calculateCRC16Xmodem(frame, N+2);
       frame[N+2]=(DEC2HEX[crc>>8]);//0x06;
       frame[N+3]=(DEC2HEX[(crc<<8 & 0xFF00)>>8]);//0xCC;
       frame[N+4]='\0';

    QByteArray writeData=QByteArray::fromRawData((char*)frame,std::strlen(frame));
//     writeData.append('\n');
*/

    const qint64 bytesWritten = m_serialPort->write(writeData);


    bool bFlush = m_serialPort->flush();
    if(!bFlush){

        bFlush = m_serialPort->flush();
    }

    if (bytesWritten == -1 || !bFlush) {
        m_standardOutput << QObject::tr("Failed to write the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(),m_serialPort->errorString());


    } else if (bytesWritten != writeData.size() || !bFlush) {
        m_standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2\n")
                            .arg(m_serialPort->portName(), m_serialPort->errorString());

    }

    // delay 20ms
//    QTime dieTime= QTime::currentTime().addMSecs(20);
//    while (QTime::currentTime() < dieTime)
//        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);

}
