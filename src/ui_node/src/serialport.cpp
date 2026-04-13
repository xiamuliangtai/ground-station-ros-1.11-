#include "serialport.h"
#include "qdebug.h"
#include "qthread.h"

serialPort::serialPort(QObject *parent)
    : QObject{parent}
{}

void serialPort::run()
{
    qDebug()<<QThread::currentThread();
    serial = new QSerialPort(this);
    serial->setPortName("/dev/ttyS0");
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    if (serial->open(QIODevice::ReadOnly)) {
        qDebug() << "无线串口打开成功！";
    } else {
        qDebug() << "打开失败：" << serial->errorString();
    }
    connect(serial,&QSerialPort::readyRead,[=](){
        QByteArray data=serial->readAll();
        for(int i=0;i<m_data.size();i++){
            if(m_data[i]==data)
                return;
        }
        m_data.append(data);
        QList<QByteArray> list=data.split(',');
        if(list.size()!=3)
            return;
        int a=list[0].toInt();
        QString b=list[1];
        QString c=list[2];
        if(b.toInt()>=1&&b.toInt()<=9&&c.toInt()>=1&&c.toInt()<=7){
            emit sendData(a,b,c);
        }
    });
}

void serialPort::clearData()
{
    m_data.clear();
}
