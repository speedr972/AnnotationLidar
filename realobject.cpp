#include "realobject.h"

RealObject::RealObject()
{
}

QString RealObject::toString()
{
    QString str = QString("%1 - %2 - %3").arg(this->id).arg(objectTypeToString(this->objectType)).arg(this->description);
    return str;
}

void RealObject::interpolate3DAnnotationsLinearly()
{

    if(this->annotations3D.size()>=2){

        std::vector<int> keys;

        std::map<int, Pose3D>::iterator it;
        for(it = this->annotations3D.begin(); it != this->annotations3D.end(); ++it){
            keys.push_back(it->first);
        }

        for(int index = 0; index < keys.size()-1; index++){

            int frameStart =  keys[index];
            int frameEnd = keys[index+1];

            Pose3D poseStart = this->annotations3D[frameStart];
            Pose3D poseEnd = this->annotations3D[frameEnd];
            QString poseStartString = QString("frameStart : %1, x : %2, y : %3, z : %4").arg(frameStart).arg(poseStart.x).arg(poseStart.y).arg(poseStart.z);
            QString poseEndString = QString("frameEnd : %1, x : %2, y : %3, z : %4").arg(frameEnd).arg(poseEnd.x).arg(poseEnd.y).arg(poseEnd.z);
            std::cout << "------------------------------------------" << std::endl;
            std::cout << "PoseStart : " << poseStartString.toStdString() << std::endl;
            std::cout << "PoseEnd : " << poseEndString.toStdString() << std::endl;



            int nbFrames = frameEnd - frameStart + 1;

            //mapping [frameStart:frameEnd] -> [0:1] lineaire y=ax+b
            float a = 1.0/(frameEnd - frameStart);
            float b = - a * frameStart;
            for(int i = 0; i<nbFrames; i++){
                int currentFrameNumber = frameStart + i;
                float t = a * currentFrameNumber + b;
                std::cout << "t : " << t << ", currentFrameNumber : " << currentFrameNumber << std::endl;

                Pose3D currentPose;
                currentPose.x = poseStart.x * (1-t) + poseEnd.x * (t);
                currentPose.y = poseStart.y * (1-t) + poseEnd.y * (t);
                currentPose.z = poseStart.z * (1-t) + poseEnd.z * (t);
                currentPose.theta = poseStart.theta * (1-t) + poseEnd.theta * (t);
                currentPose.phi = poseStart.phi * (1-t) + poseEnd.phi * (t);
                currentPose.psi = poseStart.psi * (1-t) + poseEnd.psi * (t);

                this->annotations3DInterpolated[currentFrameNumber] = currentPose;
            }
        }
    }else{
        std::cerr << "Not enough points" << std::endl;
    }
}


QString objectTypeToString(ObjectType oType)
{
    switch(oType){
    case ObjectType::Unknown:
        return QString("Unknown");
    case ObjectType::Car:
        return QString("Car");
    case ObjectType::Pedestrian:
        return QString("Pedestrian");
    case ObjectType::Motorcycle:
        return QString("Motorcycle");
    case ObjectType::Bicycle:
        return QString("Bicycle");
    case ObjectType::Bus:
        return QString("Bus");
    case ObjectType::Truck:
        return QString("Truck");
    default:
        return QString("Unknown");
    }
}




ObjectType stringToObject(QString strType)
{
    if(QString::compare(strType, "Car", Qt::CaseInsensitive)==0){
        return ObjectType::Car;
    }else if(QString::compare(strType, "Pedestrian", Qt::CaseInsensitive)==0){
        return ObjectType::Pedestrian;
    }else if(QString::compare(strType, "Motorcycle", Qt::CaseInsensitive)==0){
        return ObjectType::Motorcycle;
    }else if(QString::compare(strType, "Bicycle", Qt::CaseInsensitive)==0){
        return ObjectType::Bicycle;
    }else if(QString::compare(strType, "Bus", Qt::CaseInsensitive)==0){
        return ObjectType::Bus;
    }else if(QString::compare(strType, "Truck", Qt::CaseInsensitive)==0){
        return ObjectType::Truck;
    }else{
        return ObjectType::Unknown;
    }
}
