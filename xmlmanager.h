#ifndef XMLMANAGER_H
#define XMLMANAGER_H
#include <QtXml>
#include <QString>
#include <vector>
#include <map>
#include <iterator>
#include <iostream>
#include <QFileDialog>
#include "realobject.h"

class XMLManager
{
public:
    XMLManager();
    ~XMLManager();

    void writeClickedObjects(std::vector<RealObject*> objects); //more like a "save"
    void writeInterpolatedObjects(std::vector<RealObject*> objects); //more like an "export"

    std::vector<RealObject*> readClickedObjects();
    void readClickedInterpolatedObjects();

    //for the moment, public attributes
    QString xmlWritePath;
    QString xmlInterpolatedWritePath;
    QString xmlReadPath;
    QDomDocument *doc;
};

#endif // XMLMANAGER_H
