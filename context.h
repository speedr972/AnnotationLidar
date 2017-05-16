#ifndef CONTEXT_H
#define CONTEXT_H

#include <vector>
//#include "loader.h"
#include "realobject.h"
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <iostream>
#include <QString>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QImage>


#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include "xmlmanager.h"
class Context
{
public:
    Context();
    ~Context();
    void loadDatabase();
    void loadDatabase(QString dbPath);
    void loadImage();
    void loadImage(QString imagePath);
    bool loadPointCloudsASCII(QString csvPath);
    bool loadPointCloudPCL();
    void closeDatabase();

    void initializeResources();
    void initializeObjects();
    void findCurrentObjects();

    void clearObjects();



    QString resourceFolder;
    QString imageNamePattern;
    QString dbFilepath;
    QString csvFilename;
    QString csvFilePath;

    //0-based
    int currentFrameNumber;
    int maxResources; //if <0; problem read
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud;
    QImage *currentImage;
    std::vector<RealObject*> objects;
    QSqlDatabase m_db;
    QStringList pointCloudsASCII;


    QStringList currentObjectsString;
    std::vector<RealObject> currentObjects;
    std::vector<Pose2D> current2DAnnotations; //int : id de l'objet
    //std::map<int, Pose3D> current3DAnnotations;
    RealObject* selectedObject;
    Pose3D current3DAnnotation;


    XMLManager xmlManager;

    bool imageLoaded;
    bool lidarLoaded;
    bool annotationsLoaded;

    float currentTx;
    float currentTy;
    float currentTz;
    float currentTheta;



    //default values


};

#endif // CONTEXT_H
