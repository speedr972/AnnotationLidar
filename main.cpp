#include <QApplication>
#include "mainwindow.h"
#include <iostream>
#include <QString>

#include "context.h"
#include "poses.h"
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlRecord>
#include <QFile>
#include <map>
#include <iterator>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
void testContext(){
    Context c;
    QString dbFilepath("polytrack.sqlite");
    QFile file(dbFilepath);
    if(!file.exists()){
        std::cerr << "FILE NOT FOUND" << std::endl;
    }else{
        std::cout << "FILE EXISTS" << std::endl;
    }
    c.loadDatabase(dbFilepath);
    QStringList listTables = c.m_db.tables();
    std::cout << "liste des tables présentes dans la base : " << std::endl;
    for(int i  = 0; i< listTables.size(); i++){
        std::cout << listTables.at(i).toStdString() << std::endl;
    }
    QSqlQuery query("SELECT * FROM objects;");
    int idName = query.record().indexOf("object_id");
    int userType = query.record().indexOf("road_user_type");
    int description = query.record().indexOf("description");
    while(query.next()){

        std::cout << query.value(idName).toString().toStdString() << ", ";
        //std::cout << query.value(userType).toString().toStdString() << ", ";
        std::cout << query.value(userType).toString().toStdString() << ", ";
        std::cout << query.value(description).toString().toStdString();
        std::cout << std::endl;
    }
    c.currentFrameNumber = 0;
    c.resourceFolder = QString("/home/ruddy/sharedData/datasets/SHERPA_Roulages_splitImagesLidar/20170320_102448_ResampledResources");
    QString csvName("ResampledResources_20170424_092042_resampler_2_output_1.csv");
    QString imageName = QString("ResampledResources_20170424_092042_Extraction__pointgrey_flycapture_ipl_image_%1.png").arg(c.currentFrameNumber);
    QString csvPath = QDir::cleanPath(c.resourceFolder +QDir::separator() + csvName);
    QString imagePath = QDir::cleanPath(c.resourceFolder +QDir::separator() + imageName);


    std::cout << "csv : " << csvPath.toStdString() << std::endl;
    std::cout << "image : " << imagePath.toStdString() << std::endl;
    c.loadPointCloudsASCII(csvPath);
    std::cout << "sizePointCloud : " << c.pointCloudsASCII.size() << std::endl;
    std::cout << "---------------------------------------------" << std::endl;

    c.loadPointCloudPCL();
    c.initializeObjects();



    std::map<int, Pose2D>::iterator it;
    std::cout << "nbObjects : " << c.objects.size() << std::endl;
    if(c.objects.size()<=0){
        std::cerr << "NO OBJECTS CREATED" << std::endl;
    }else{
        for(int o = 0; o<c.objects.size(); o++){
            RealObject *obj =  c.objects.at(o);
            std::cout << objectTypeToString(obj->objectType).toStdString() << std::endl;
            for(it=obj->annotations2D.begin(); it!=obj->annotations2D.end(); ++it){
                int frameNumber = it->first;
                Pose2D pose2d =it->second;
                std::cout << "frame number : "<< frameNumber << ", (x, y) : " << pose2d.x << ", " << pose2d.y << std::endl;
            }
        }
        std::cout << "---------------------------------------------" << std::endl;

    }

    c.closeDatabase();

}

struct callback_args {
    MainWindow *mw_struct;
};

void areaEventOccuredMain(const pcl::visualization::AreaPickingEvent& event, void *args){
    //std::cout << "hello area event" << std::endl;
    struct callback_args* data = (struct callback_args *)args;



    if(data->mw_struct->c->selectedObject){
        std::vector< int > indices_callback;

        event.getPointsIndices(indices_callback);
        //std::cout << "sze indices_callback : " << indices_callback.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_selected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers->indices = indices_callback;
        extract.setInputCloud (data->mw_struct->c->currentCloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_selected);


        data->mw_struct->viewer->removeShape("AABB");
        data->mw_struct->viewer->removePointCloud("selected_cloud");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_selected_color(cloud_selected, 255,0,0);
        data->mw_struct->viewer->addPointCloud(cloud_selected, cloud_selected_color, "selected_cloud");


        //BOUNDING BOX PART
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud (cloud_selected);
        feature_extractor.compute ();

        std::vector <float> moment_of_inertia;
        std::vector <float> eccentricity;
        pcl::PointXYZ min_point_AABB;
        pcl::PointXYZ max_point_AABB;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getMomentOfInertia (moment_of_inertia);
        feature_extractor.getEccentricity (eccentricity);
        feature_extractor.getAABB (min_point_AABB, max_point_AABB);
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues (major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter (mass_center);
        data->mw_struct->viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");


        Pose3D pose3D;
        pose3D.x = (max_point_AABB.x - min_point_AABB.x)/2.0;
        pose3D.y = (max_point_AABB.y - min_point_AABB.y)/2.0;
        pose3D.z = (max_point_AABB.z - min_point_AABB.z)/2.0;

        //pose3D.height = max_point_AABB.z - min_point_AABB.z;
        //pose3D.length = max_point_AABB.x - min_point_AABB.x;
        //pose3D.width = max_point_AABB.y - min_point_AABB.y;

        pose3D.phi = 0.0;
        pose3D.psi = 0.0;
        pose3D.theta = 0.0;

        data->mw_struct->c->selectedObject->annotations3D[data->mw_struct->c->currentFrameNumber] = pose3D;
    }
    //boost::shared_ptr< wrapperObject > wrapperObj = *static_cast< wrapperObject** >(wrapper);
}

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);

    //testContext();
    std::cout << "hello" << std::endl;
    //MainWidget *mw = new MainWidget;
    MainWindow *mw = new MainWindow;

    struct callback_args cb_args;
    cb_args.mw_struct = mw;
    mw->viewer->registerAreaPickingCallback(areaEventOccuredMain, (void *)&cb_args);

    mw->show();
    return a.exec();

    /*
    MainWindow w;
    w.show();
    return a.exec();
    */
}
