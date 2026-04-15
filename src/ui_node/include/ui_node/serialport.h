#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QObject>
#include<QSerialPort>
#include<QByteArray>

class serialPort : public QObject
{
    Q_OBJECT
public:
    explicit serialPort(QObject *parent = nullptr);
    // TODO(stage-next): UI内串口逻辑后续应迁移到独立 bridge node（如 fc_bridge_node）。
public slots:
    void run();
    void clearData();
signals:
    void sendData(int animal,QString x,QString y);
private:
    QList<QByteArray> m_data;
    QSerialPort *serial;
};

#endif // SERIALPORT_H
