#ifndef REALOBJECT_H
#define REALOBJECT_H

#include <map>
#include <iterator>
#include <iostream>
#include <vector>
#include <algorithm>
#include "poses.h"
#include <QString>

enum ObjectType { Unknown, Car, Pedestrian, Motorcycle, Bicycle, Bus, Truck};
QString objectTypeToString(ObjectType oType);
ObjectType stringToObject(QString strType);

class RealObject
{
public:
    RealObject();

    int id;
    ObjectType objectType;
    std::map<int, Pose2D> annotations2D;//int : frame number
    std::map<int, Pose3D> annotations3D;//int : frame number
    std::map<int, Pose3D> annotations3DInterpolated;//int : frame number

    bool complete;

    //In opposition with 2D annotations, 3D items are supposed to have a constant size
    float width;
    float height;
    float length;
    QString description;


    QString toString();
    void interpolate3DAnnotationsLinearly();

};

#endif // REALOBJECT_H
