#ifndef __HIREDIS_MKREDIS_H
#define __HIREDIS_MKREDIS_H
#include <iostream>
#include "rediqtadapter.h"
#include <QDebug>
#include <QTimer>
#include <QCoreApplication>
using namespace std;
class MKRedis : public QObject {

    Q_OBJECT
private:
    const char* subscribeRedis = "istacker:order";
    private:
    QCoreApplication *app;

    public:
        QTimer *timer;
        MKRedis( QObject * parent = nullptr): QObject(parent) {
//            qDebug()<<"constructor: "<<m_value;
             app = QCoreApplication::instance();
        }

    signals:
        void finished();

    public slots:
        void run();
        void timerRedisSlot();

    private:
        void finish() {
            cout<<"finished"<<endl;
            emit finished();
        }

    private:
        const char * m_value;
        redisAsyncContext * m_ctx;
        RedisQtAdapter m_adapter;

    friend void getCallback(redisAsyncContext *, void *, void *);
    friend void subscribeRedisReply(redisAsyncContext *, void *, void *);
};

#endif /* !__HIREDIS_MKREDIS_H */
