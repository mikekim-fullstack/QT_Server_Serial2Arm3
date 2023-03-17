#include <iostream>
using namespace std;

#include <signal.h>
#include <unistd.h>

#include <async.h>

#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <QDebug>

#include "mkredis.h"
#include </usr/local/include/hiredis/adapters/libevent.h>

#include <async.h>



void getCallback(redisAsyncContext *, void * r, void * privdata) {

    redisReply * reply = static_cast<redisReply *>(r);
    MKRedis * ex = static_cast<MKRedis *>(privdata);
    if (reply == nullptr || ex == nullptr) return;

    if(reply->type==2){
        for(size_t i=0; i<reply->elements; i++){
            if(reply->element[i]->str) cout<<"key name: "<<reply->element[i]->str<<endl;
        }
    }
    else {
        cout << "key: " << reply->str << endl;
    }

    ex->finish();
}
void subscribeRedisReply(redisAsyncContext *, void *r, void *privdata) {
    redisReply * reply = static_cast<redisReply *>(r);
    MKRedis * ex = static_cast<MKRedis *>(privdata);
    if (reply == nullptr || ex == nullptr) return;

    if(reply->type==2){
        if(reply->elements==3 &&
                strcmp(reply->element[0]->str,"message")==0 &&
                strcmp(reply->element[1]->str,ex->subscribeRedis)==0) {


            QString str(reply->element[2]->str);
            QJsonDocument jsonResponse = QJsonDocument::fromJson(str.toUtf8());
            QJsonObject jsonObject = jsonResponse.object();
            int age = jsonObject["age"].toInt();
            QString sex = jsonObject["sex"].toString();
             cout<<"message: "<<reply->element[2]->str<<age<<","<<sex.toStdString()<<endl;

        }
        else {

            for(size_t i=0; i<reply->elements; i++){
                if(reply->element[i]->str) {

                    cout<<"key name: "<<reply->element[i]->str<<endl;
                }
            }
        }
    }
    else {
        cout << "key: " << reply->str << endl;
    }

    ex->finish();
}
void MKRedis::timerRedisSlot() {
// redisAsyncCommand(m_ctx, getCallback, this, "SUBSCRIBE app:notifications");
}


void MKRedis::run() {

//    struct event_base *base = event_base_new();
    signal(SIGPIPE, SIG_IGN);
    struct event_base *base = event_base_new();
    assert(base != nullptr);

    m_ctx = redisAsyncConnect("containers-us-west-108.railway.app",6658);

    if (m_ctx->err) {
        qDebug()<<"error";
        cerr << "Error: " << m_ctx->errstr << endl;
        redisAsyncFree(m_ctx);
        emit finished();
    }


    m_adapter.setContext(m_ctx);

    redisAsyncCommand(m_ctx, NULL, NULL, "AUTH Wz0m7LBEFTqdC8UZET1C");

//    redisAsyncCommand(m_ctx, NULL, this, "SET key %s", m_value);
//    redisAsyncCommand(m_ctx, getCallback, this, "keys *");
//    redisLibeventAttach(m_ctx, base);
    redisAsyncCommand(m_ctx, subscribeRedisReply, this, "SUBSCRIBE %s",subscribeRedis);
//    redisAsyncCommand(m_ctx, getCallback, this, "PUBLISH %s %s", "app:notifications", "qt");
//    event_base_dispatch(base);
////    redisAsyncCommand(m_ctx, onMessage, this, "SUBSCRIBE app:notifications");

}
/*
int main (int argc, char **argv) {

    QCoreApplication app(argc, argv);

    MKRedis example(argv[argc-1]);

    QObject::connect(&example, SIGNAL(finished()), &app, SLOT(quit()));
    QTimer::singleShot(0, &example, SLOT(run()));

    return app.exec();
}
*/
