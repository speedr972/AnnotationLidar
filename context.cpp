#include "context.h"

Context::Context()
{
    this->currentTx = 0.0;
    this->currentTy = 0.0;
    this->currentTz = 0.0;
    this->currentTheta = 0.0;

    this->resourceFolder = "/home/ruddy/sharedData/datasets/SHERPA_Roulages_splitImagesLidar";
    this->dbFilepath = "/home/ruddy/Documents/workspace_c++/AnnotationImageLidar/build/polytrack.sqlite";
    /*
    //this->loader = Loader(this);
    this->dbFilepath = "/home/ruddy/Documents/workspace_c++/AnnotationImageLidar/build/polytrack.sqlite";
    this->resourceFolder = "/home/ruddy/sharedData/datasets/SHERPA_Roulages_splitImagesLidar/20170320_102448_ResampledResources";
    this->imageNamePattern = "ResampledResources_20170424_092042_Extraction__pointgrey_flycapture_ipl_image_";
    this->csvFilename = "ResampledResources_20170424_092042_resampler_2_output_1.csv";
    this->currentFrameNumber = 1;

    QString currentImageName = QString("%1%2.png").arg(this->imageNamePattern).arg(this->currentFrameNumber);
    QString currentImagePath = QDir::cleanPath(this->resourceFolder + QDir::separator() + currentImageName);
    QString currentLidarPath = QDir::cleanPath(this->resourceFolder + QDir::separator() + this->csvFilename);

    std::cout << "currentImageName : " << currentImageName.toStdString() << std::endl;
    std::cout << "currentImagePath : " << currentImagePath.toStdString() << std::endl;
    std::cout << "currentLidarPath : " << currentLidarPath.toStdString() << std::endl;
    //TODO : faire les verifications d'existence



    this->selectedObject = nullptr;


    QDir export_folder(this->resourceFolder);
    export_folder.setNameFilters(QStringList()<<"*.png");
    QStringList fileList = export_folder.entryList();


    lidarLoaded = false;
    this->maxResources = -1;
    if(loadPointCloudsASCII(currentLidarPath)){
        lidarLoaded = loadPointCloudPCL();
        this->maxResources = qMin(this->pointCloudsASCII.size()-1, fileList.size()-1);
    }
    this->currentImage = new QImage;
    imageLoaded = this->currentImage->load(currentImagePath);

    std::cout << "imageLoaded : " << imageLoaded << ", lidarLoaded : " << lidarLoaded << std::endl;

    //TODO : faire les verifications dnecessaires

    loadDatabase(this->dbFilepath);

    initializeObjects();
    */
}


Context::~Context()
{
    closeDatabase();
}

void Context::loadDatabase()
{

}

void Context::loadDatabase(QString dbPath)
{
    m_db = QSqlDatabase::addDatabase("QSQLITE");
    m_db.setDatabaseName(dbPath);

    if (!m_db.open())
    {
        //qDebug() << "Error: connection with database fail";
        std::cerr << "Error: connection with database fail" << std::endl;
    }
    else
    {
        //qDebug() << "Database: connection ok";
        std::cout << "Database: connection ok" << std::endl;
    }
}

void Context::loadImage()
{
    QString currentImageName = QString("%1%2.png").arg(this->imageNamePattern).arg(this->currentFrameNumber);
    QString currentImagePath = QDir::cleanPath(this->resourceFolder + QDir::separator() + currentImageName);
    imageLoaded = this->currentImage->load(currentImagePath);
}


bool Context::loadPointCloudsASCII(QString csvPath)
{
    QFile file(csvPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        std::cerr << "Unable to open file" << std::endl;
        return false;
    }
    std::cout << "(Context::loadPointCloudsASCII) csv file opened" << std::endl;
    QTextStream flux(&file);
    QString allText = flux.readAll();
    this->pointCloudsASCII = allText.split("\n");
    std::cout << "(Context::loadPointCloudsASCII) string splited : " << this->pointCloudsASCII.size() << std::endl;
    return true;
}

bool Context::loadPointCloudPCL()
{
    std::cout << "(Context::loadPointCloudPCL) this->currentFrameNumber : "<< this->currentFrameNumber << ", this->pointCloudsASCII.size : " << this->pointCloudsASCII.size() << std::endl;
    if(this->currentFrameNumber < this->pointCloudsASCII.size()){
        std::cout << "(Context::loadPointCloudPCL) this->currentFrameNumber < this->pointCloudsASCII.size" << std::endl;
        QString currentLine = this->pointCloudsASCII.at(this->currentFrameNumber);

        QStringList dataStr = currentLine.split(";", QString::SkipEmptyParts);

        dataStr.pop_front();
        int nbPoints = dataStr.size()/3;
        //std::cout << "size data : " << dataStr.size() << ", nbPoints : " << nbPoints << std::endl;

        std::cout << "(Context::loadPointCloudPCL) before resetPointCloud" << std::endl;
        currentCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        currentCloud->width = nbPoints;
        currentCloud->height = 1;
        currentCloud->points.resize(nbPoints);
        std::cout << "(Context::loadPointCloudPCL) point cloud set up" << std::endl;

        for(int i = 0; i<nbPoints; i++){
            currentCloud->points[i].x = dataStr.at(i*3).toFloat();
            currentCloud->points[i].y = dataStr.at(i*3+1).toFloat();
            currentCloud->points[i].z = dataStr.at(i*3+2).toFloat();
        }
        std::cout << "(Context::loadPointCloudPCL) end_loop" << std::endl;
        this->lidarLoaded = true;
        return true;
    }else{
        std::cerr << "(Context::loadPointCloudPCL) lidar not loaded" << std::endl;
        this->lidarLoaded = false;
        return false;
    }
}

void Context::closeDatabase()
{
    m_db.close();
}

void Context::initializeResources()
{

    QString currentImageName = QString("%1%2.png").arg(this->imageNamePattern).arg(this->currentFrameNumber);
    QString currentImagePath = QDir::cleanPath(this->resourceFolder + QDir::separator() + currentImageName);
    QString currentLidarPath = QDir::cleanPath(this->resourceFolder + QDir::separator() + this->csvFilename);

    lidarLoaded = false;
    if(loadPointCloudsASCII(currentLidarPath)){
        lidarLoaded = loadPointCloudPCL();
        //this->maxResources = qMin(this->pointCloudsASCII.size()-1, fileList.size()-1);
    }
    this->currentImage = new QImage;
    imageLoaded = this->currentImage->load(currentImagePath);
    this->currentFrameNumber = 0;
}

void Context::initializeObjects()
{
    if(m_db.isOpen()){
        this->clearObjects();
        std::cout << "m_db is open in initializeObjects" << std::endl;

        QStringList tablesNames = m_db.tables();
        for(int i = 0; i<tablesNames.size(); i++){
            std::cout << tablesNames.at(i).toStdString() << ", ";
        }
        std::cout << std::endl;

        QSqlQuery query("SELECT * FROM objects;");
        query.exec();
        //QSqlQuery query = m_db.exec("SELECT * FROM objects;");

        //std::cout << "query size (SELECT * FROM OBJECTS) : " << query.size() << std::endl;
        //size() NON COMPATIBLE AVEC SQLITE !!!!!!!!!!!!!!!!!!!!!!!!!!
        while(query.next()){
            RealObject *obj = new RealObject;
            int id = query.value(0).toInt();
            int typeNumber = query.value(1).toInt();
            QString description = query.value(2).toString();

            std::cout << "id : " << id << ", typeNumber : " << typeNumber << ", description : " << description.toStdString()<<std::endl;
            obj->id = id;
            obj->description = description;
            if(typeNumber>6 || typeNumber<1){
                obj->objectType = ObjectType::Unknown;
            }else{
                obj->objectType = (ObjectType)typeNumber;
            }

            //add annotations
            QSqlQuery queryAnnotations;
            queryAnnotations.prepare("SELECT * FROM bounding_boxes WHERE object_id=:object_id");
            queryAnnotations.bindValue(":object_id", id);
            queryAnnotations.exec();
            while(queryAnnotations.next()){
                float x_top_left = queryAnnotations.value(2).toFloat();
                float y_top_left = queryAnnotations.value(3).toFloat();
                float x_bottom_right = queryAnnotations.value(4).toFloat();
                float y_bottom_right = queryAnnotations.value(5).toFloat();
                int frameNumber = queryAnnotations.value(1).toInt();
                Pose2D pose2d;
                pose2d.x = x_top_left;
                pose2d.y = y_top_left;
                pose2d.width = x_bottom_right - x_top_left;
                pose2d.height = y_bottom_right - y_top_left;

                obj->annotations2D[frameNumber] = pose2d;

            }



            this->objects.push_back(obj);
        }
    }else{
        std::cerr << "m_db is CLOSED in initializeObjects" << std::endl;
    }
}

void Context::findCurrentObjects()
{
    currentObjects.clear();
    currentObjectsString.clear();
    current2DAnnotations.clear();


    for(int i = 0; i<objects.size(); i++){
        RealObject *robj = objects.at(i);
        /*
        if(robj->annotations2D.find(currentFrameNumber) == robj->annotations2D.end()){
        }else{
            currentObjects.push_back(*robj);
            current2DAnnotations.push_back(robj->annotations2D[currentFrameNumber]);
            currentObjectsString << QString("%1").arg(robj->id);
        }
        */
        currentObjectsString << QString("%1").arg(robj->id);
    }
}

void Context::clearObjects()
{
    this->selectedObject = nullptr;
    for(int i = 0; i<this->objects.size(); i++){
        delete this->objects.at(i);
    }
    this->objects.clear();
}
