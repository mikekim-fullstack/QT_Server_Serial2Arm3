#include "helper.h"
#define STD_OUT QTextStream(stdout)
Helper con;
Helper::Helper()
{

}

void Helper::print(QString str)
{
    QTextStream out(stdout);
//    STD_OUT << str<<"\u001b[33;1m"<<endl<<flush;
    STD_OUT << "\033[37;40m"<<str<<"\033[0m"<<flush;
    //\u001b[33;1m
}

void Helper::printC(QString str)
{
     STD_OUT << "\033[1;36m"<<str<<"\033[0m"<<flush;
}

void Helper::printR(QString str)
{
   STD_OUT << "\033[1;31m"<<str<<"\033[0m"<<flush;
}

void Helper::printG(QString str)
{
    STD_OUT << "\033[1;32m"<<str<<"\033[0m"<<flush;
}
