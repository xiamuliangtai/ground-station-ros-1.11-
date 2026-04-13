#include "simplepathgenerator.h"

SimplePathGenerator::SimplePathGenerator(QObject *parent)
    : QObject{parent}
{}

void SimplePathGenerator::run(QList<QPoint> block)
{
   QList<QPoint> path=generate(block);
    qDebug()<<path<<"\n"<<path.size();
    emit sendPath(path);
}
