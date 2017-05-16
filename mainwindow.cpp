#include "mainwindow.h"
//#include "build/ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{

    QMenu *fileMenu = this->menuBar()->addMenu("&File");
    this->openImageFolderAction = new QAction("Open Image Folder", this);
    this->openCSVFileAction = new QAction("Open Point Cloud CSV", this);
    this->load2DSQLAnnotationsAction = new QAction("Load 2D SQL Annotations", this);
    this->save3DXMLAnnotationsAction = new QAction("Save 3D XML Annotations", this);
    this->save3DXMLAnnotationsInterpolatedAction = new QAction("Save 3D XML Interpolated Annotations", this);
    this->load3DXMLAnnotationsAction = new QAction("Load 3D XML annotations", this);

    this->save3DXMLAnnotationsAction->setShortcut(QKeySequence("Ctrl+S"));


    std::cout << "before addActions" << std::endl;
    fileMenu->addAction(this->openImageFolderAction);
    fileMenu->addAction(this->openCSVFileAction);
    fileMenu->addAction(this->load2DSQLAnnotationsAction);
    fileMenu->addAction(this->save3DXMLAnnotationsAction);
    fileMenu->addAction(this->save3DXMLAnnotationsInterpolatedAction);
    fileMenu->addAction(this->load3DXMLAnnotationsAction);

    this->c = new Context();
    std::cout << "Context created" << std::endl;

    QWidget *singleDocumentInterfaceWidget = new QWidget;
    QHBoxLayout *mainLayout = new QHBoxLayout;
    singleDocumentInterfaceWidget->setLayout(mainLayout);
    this->setCentralWidget(singleDocumentInterfaceWidget);

    QVBoxLayout *mainResourcesLayout = new QVBoxLayout;
    QVBoxLayout *viewersLayout = new QVBoxLayout;
    QVBoxLayout *objectsLayout = new QVBoxLayout;
    mainLayout->addLayout(mainResourcesLayout);
    mainLayout->addLayout(viewersLayout);
    mainLayout->addLayout(objectsLayout);


    ////////////////////////////////////////////////////////////////////////////////////////
    QVBoxLayout *resourcesLayout = new QVBoxLayout;
    mainResourcesLayout->addLayout(resourcesLayout);

    this->resourceFolderEdit = new QLineEdit;
    this->imageNamePatternEdit = new QLineEdit;
    this->csvFileEdit = new QLineEdit;
    this->resourceNumberEdit = new QLineEdit;


    QHBoxLayout *navigationLayout = new QHBoxLayout;

    this->prevResButton = new QPushButton("<");
    this->nextResButton = new QPushButton(">");
    this->frameStepEdit = new QLineEdit;
    navigationLayout->addWidget(this->prevResButton);
    navigationLayout->addWidget(this->frameStepEdit);
    navigationLayout->addWidget(this->nextResButton);


    this->frameStep = 1;
    this->frameStepEdit->setText(QString("%1").arg(this->frameStep));
    this->frameStepEdit->setValidator(new QIntValidator);

    resourcesLayout->addLayout(navigationLayout);


    this->frameNumberLabel = new QLabel;
    QString frameNumberText = QString("%1/%2").arg(c->currentFrameNumber).arg(c->maxResources);
    this->frameNumberLabel->setText(frameNumberText);
    this->frameNumberLabel->setWordWrap(true);
    QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    this->frameNumberLabel->setSizePolicy(sizePolicy);
    resourcesLayout->addWidget(this->frameNumberLabel);

    this->reset3DViewButton = new QPushButton("Reset 3D View");
    resourcesLayout->addWidget(this->reset3DViewButton);

    this->loadFolderButton = new QPushButton("Load Resource Folder");
    this->loadAnnotationsButton = new QPushButton("Load Annotation");
    resourcesLayout->addWidget(this->loadFolderButton);
    resourcesLayout->addWidget(this->loadAnnotationsButton);



    ////////////////////////////////////////////////////////////////////////////////////////

    QTabWidget *viewerTabs = new QTabWidget;
    viewersLayout->addWidget(viewerTabs);
    this->imageLabel = new QLabel;
    this->imageLabel->setScaledContents( true );
    //this->imageLabel->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );


    this->viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer", false));
    ////////////////////////////////////////////////////////////
    //boost::function<void (const pcl::visualization::AreaPickingEvent& event, void*)> f = boost::bind (&MainWidget::areaEventOccured, this, _1);
    //this->viewer->registerAreaPickingCallback(areaEventOccured);
    ////////////////////////////////////////////////////////////
    vtkViewer = new QVTKWidget;
    vtkViewer->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkViewer->GetInteractor(), vtkViewer->GetRenderWindow ());
    vtkViewer->update();
    //vtkViewer->resize(imageViewer->size());
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();



    objInfoWidget = new RealObjectInformationWidget;


    viewerTabs->addTab(this->imageLabel, "Image");
    viewerTabs->addTab(this->vtkViewer, "LIDAR");
    viewerTabs->addTab(this->objInfoWidget, "Info");


    ////////////////////////////////////////////////////////////////////////////////////////
    this->objectListView = new QListView;
    this->objectListView->setMaximumHeight(500);
    this->objectListView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    this->model = new QStringListModel;
    objectsLayout->addWidget(this->objectListView);
    c->findCurrentObjects();
    this->initObjectListView();



    this->selectedObjectLabel = new QLabel;
    QString selectedObjectText;
    if(c->selectedObject){
        selectedObjectText = QString("Selected object : %1").arg(c->selectedObject->id);
    }else{
        selectedObjectText = QString("Selected object : NULL");
    }
    this->selectedObjectLabel->setText(selectedObjectText);
    this->selectedObjectLabel->setWordWrap(true);
    this->selectedObjectLabel->setSizePolicy(sizePolicy);
    objectsLayout->addWidget(this->selectedObjectLabel);

    this->txSpinbox = new QDoubleSpinBox;
    this->txSpinbox->setSingleStep(0.1);
    this->txSpinbox->setMinimum(-99.0);
    this->txSpinbox->setSizePolicy(sizePolicy);
    objectsLayout->addWidget(this->txSpinbox);
    this->tySpinbox = new QDoubleSpinBox;
    this->tySpinbox->setSingleStep(0.1);
    this->tySpinbox->setMinimum(-99.0);
    this->tySpinbox->setSizePolicy(sizePolicy);
    objectsLayout->addWidget(this->tySpinbox);
    this->tzSpinbox = new QDoubleSpinBox;
    this->tzSpinbox->setSingleStep(0.1);
    this->tzSpinbox->setMinimum(-99.0);
    this->tzSpinbox->setSizePolicy(sizePolicy);
    objectsLayout->addWidget(this->tzSpinbox);

    this->thetaSpinbox = new QDoubleSpinBox;
    this->thetaSpinbox->setSingleStep(1);
    this->thetaSpinbox->setMinimum(-360.0);
    this->thetaSpinbox->setMaximum(360.0);
    this->thetaSpinbox->setSizePolicy(sizePolicy);
    objectsLayout->addWidget(this->thetaSpinbox);

    this->addAnnotationButton = new QPushButton("Add 3D annotation");
    objectsLayout->addWidget(this->addAnnotationButton);

    //this->writeXMLButton = new QPushButton("Write XML");
    //objectsLayout->addWidget(this->writeXMLButton);

    this->interpolatePosesButton = new QPushButton("Interpolate");
    objectsLayout->addWidget(this->interpolatePosesButton);

    //this->writeXMLInterpolatedButton = new QPushButton("Write Interpolated XML");
    //objectsLayout->addWidget(this->writeXMLInterpolatedButton);

    ////////////////////////////////////////////////////////////////////////////////////////

    /*
    if(c->imageLoaded && c->lidarLoaded){
        this->imageLabel->setPixmap(QPixmap::fromImage(*c->currentImage));
        //this->viewer
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(c->currentCloud, 255,255,255);
        this->viewer->addPointCloud<pcl::PointXYZ> (c->currentCloud, cloud_color, "cloud");
    }
    */

    selectionBox = new QRubberBand(QRubberBand::Rectangle, this->imageLabel);
    QPalette palette;
    palette.setBrush(QPalette::Foreground, QBrush(Qt::green));
    palette.setBrush(QPalette::Base, QBrush(Qt::red));
    selectionBox->setPalette(palette);

    connect(this->objectListView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(listViewSlot(QModelIndex)));
    connect(this->frameStepEdit, SIGNAL(textChanged(QString)), this, SLOT(frameStepSlot()));
    connect(this->nextResButton, SIGNAL(clicked()), this, SLOT(nextFrameSlot()));
    connect(this->prevResButton, SIGNAL(clicked()), this, SLOT(prevFrameSlot()));
    connect(this->reset3DViewButton, SIGNAL(clicked()), this, SLOT(reset3DView()));
    //connect(this->writeXMLButton, SIGNAL(clicked()), this, SLOT(writeXMLSlot()));
    //connect(this->writeXMLInterpolatedButton, SIGNAL(clicked()), this, SLOT(writeXMLInterpolatedSlot()));

    connect(this->interpolatePosesButton, SIGNAL(clicked()), this, SLOT(interpolatePosesSlot()));
    connect(this->txSpinbox, SIGNAL(valueChanged(double)), this, SLOT(txChanged(double)));
    connect(this->tySpinbox, SIGNAL(valueChanged(double)), this, SLOT(tyChanged(double)));
    connect(this->tzSpinbox, SIGNAL(valueChanged(double)), this, SLOT(tzChanged(double)));
    connect(this->thetaSpinbox, SIGNAL(valueChanged(double)), this, SLOT(thetaChanged(double)));
    connect(this->addAnnotationButton, SIGNAL(clicked()), this, SLOT(addAnnotationSlot()));

    connect(this->load2DSQLAnnotationsAction, SIGNAL(triggered()), this, SLOT(load2DSQLAnnotationsSlot()));
    connect(this->openImageFolderAction, SIGNAL(triggered()), this, SLOT(openImageFolderSlot()));
    connect(this->openCSVFileAction, SIGNAL(triggered()), this, SLOT(openCSVFileSlot()));
    connect(this->save3DXMLAnnotationsAction, SIGNAL(triggered()), this, SLOT(writeXMLSlot()));
    connect(this->save3DXMLAnnotationsInterpolatedAction, SIGNAL(triggered()), this, SLOT(writeXMLInterpolatedSlot()));
    connect(this->load3DXMLAnnotationsAction, SIGNAL(triggered()), this, SLOT(load3DXMLAnnotationsSlot()));

    delta = 40;
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(refreshVtkViewer()));
    timer->start(delta);
}




MainWindow::~MainWindow()
{
    delete this->c;
}

void MainWindow::initObjectListView()
{
    model->removeRows(0, model->rowCount());

    model->setStringList(c->currentObjectsString);
    this->objectListView->setModel(this->model);
}



void MainWindow::refreshVtkViewer()
{
    this->vtkViewer->update();
}

void MainWindow::updateViewers()
{
    //c->currentFrameNumber = nextPossibleFrame;
    std::cout << "(updateViewers) before loadImage" << std::endl;
    c->loadImage();
    std::cout << "(updateViewers) before loadPointCloudPCL" << std::endl;
    c->loadPointCloudPCL();
    std::cout << "(updateViewers) after loads" << std::endl;
    if(c->imageLoaded && c->lidarLoaded){
        //std::cout << "Image loaded in slot"<< std::endl;
        this->imageLabel->setPixmap(QPixmap::fromImage(*c->currentImage));
        std::cout << "(updateViewers) after setPixmap" << std::endl;
        QString frameNumberText = QString("%1/%2").arg(c->currentFrameNumber).arg(c->maxResources);
        this->frameNumberLabel->setText(frameNumberText);
        this->frameNumberLabel->adjustSize();
        this->c->findCurrentObjects();
        this->selectionBox->hide();
        this->initObjectListView();
        std::cout << "(updateViewers) after initObjectListView" << std::endl;
        //std::cout << "process point cloud" << std::endl;
        this->viewer->removePointCloud("cloud");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(c->currentCloud, 255,255,255);
        this->viewer->addPointCloud<pcl::PointXYZ> (c->currentCloud, cloud_color, "cloud");
        std::cout << "END process point cloud" << std::endl;
        drawInterpolatedBoundingBoxes();
        drawClickedBoundingBox();
    }else{
        QString err = QString("error : imageLoaded : %1, lidarLoaded : %2").arg(c->imageLoaded).arg(c->lidarLoaded);
        std::cerr << err.toStdString() <<  std::endl;
    }
}

void MainWindow::listViewSlot(QModelIndex index)
{

    this->viewer->removeShape("AABB");
    this->viewer->removePointCloud("selected_cloud");
    int chosenIndex = index.row();

    /*
    Pose2D chosenAnnotation;
    chosenAnnotation = c->current2DAnnotations.at(chosenIndex);
    this->selectionBox->setGeometry(chosenAnnotation.x, chosenAnnotation.y, chosenAnnotation.width, chosenAnnotation.height);
    this->selectionBox->show();
    */

    QVariant elementSelectionne = this->model->data(index, Qt::DisplayRole);
    int idSelected = elementSelectionne.toInt();
    for(int i = 0; i<c->objects.size(); i++){
        RealObject *robj = c->objects.at(i);
        if(idSelected==robj->id){
            c->selectedObject = robj;

            if(c->selectedObject->annotations2D.find(c->currentFrameNumber) != c->selectedObject->annotations2D.end()){
                Pose2D chosenAnnotation;
                chosenAnnotation = c->selectedObject->annotations2D.at(c->currentFrameNumber);
                this->selectionBox->setGeometry(chosenAnnotation.x, chosenAnnotation.y, chosenAnnotation.width, chosenAnnotation.height);
                this->selectionBox->show();
            }

            QString selectedObjectText = QString("Selected object : %1").arg(idSelected);
            this->selectedObjectLabel->setText(selectedObjectText);

            this->objInfoWidget->robj = c->selectedObject;
            this->objInfoWidget->updateWidget(c->selectedObject);
            drawInterpolatedBoundingBoxes();
            drawClickedBoundingBox();
            drawBoundingBox();

            break;
        }
    }


}


void MainWindow::nextFrameSlot()
{
    //std::cout << "nextFrameSlot();" << std::endl;
    if(c->maxResources>=0){
        this->viewer->removeShape("AABB");
        this->viewer->removePointCloud("selected_cloud");
        int nextPossibleFrame = c->currentFrameNumber + this->frameStep;
        if(nextPossibleFrame>=0  && nextPossibleFrame<=c->maxResources){
            c->currentFrameNumber = nextPossibleFrame;
            c->loadImage();
            c->loadPointCloudPCL();
            if(c->imageLoaded && c->lidarLoaded){
                //std::cout << "Image loaded in slot"<< std::endl;
                this->imageLabel->setPixmap(QPixmap::fromImage(*c->currentImage));
                QString frameNumberText = QString("%1/%2").arg(c->currentFrameNumber).arg(c->maxResources);
                this->frameNumberLabel->setText(frameNumberText);
                this->frameNumberLabel->adjustSize();
                this->c->findCurrentObjects();
                this->selectionBox->hide();
                this->initObjectListView();

                //std::cout << "process point cloud" << std::endl;
                this->viewer->removePointCloud("cloud");
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(c->currentCloud, 255,255,255);
                this->viewer->addPointCloud<pcl::PointXYZ> (c->currentCloud, cloud_color, "cloud");
                //std::cout << "END process point cloud" << std::endl;

                drawInterpolatedBoundingBoxes();
                drawClickedBoundingBox();
            }
        }
    }else{
        std::cerr << "maxResources not correctly defined" << std::endl;
    }
}

void MainWindow::prevFrameSlot()
{
    //std::cout << "prevFrameSlot();" << std::endl;
    if(c->maxResources>=0){
        this->viewer->removeShape("AABB");
        this->viewer->removePointCloud("selected_cloud");
        int nextPossibleFrame = c->currentFrameNumber - this->frameStep;
        if(nextPossibleFrame>=0  && nextPossibleFrame<=c->maxResources){
            c->currentFrameNumber = nextPossibleFrame;
            c->loadImage();
            c->loadPointCloudPCL();
            if(c->imageLoaded && c->lidarLoaded){

                //std::cout << "Image loaded in slot"<< std::endl;
                this->imageLabel->setPixmap(QPixmap::fromImage(*c->currentImage));
                QString frameNumberText = QString("%1/%2").arg(c->currentFrameNumber).arg(c->maxResources);
                this->frameNumberLabel->setText(frameNumberText);
                this->frameNumberLabel->adjustSize();
                this->c->findCurrentObjects();
                this->selectionBox->hide();
                this->initObjectListView();

                //std::cout << "process point cloud" << std::endl;
                this->viewer->removePointCloud("cloud");
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(c->currentCloud, 255,255,255);
                this->viewer->addPointCloud<pcl::PointXYZ> (c->currentCloud, cloud_color, "cloud");
                //std::cout << "END process point cloud" << std::endl;

                drawInterpolatedBoundingBoxes();
                drawClickedBoundingBox();
            }
        }
    }else{
        std::cerr << "maxResources not correctly defined" << std::endl;
    }

}

void MainWindow::frameStepSlot()
{
    std::cout << "frameStepSlot();" << std::endl;
    this->frameStep = this->frameStepEdit->text().toInt();
}

void MainWindow::reset3DView()
{
    viewer->setCameraPosition(
                0.0,0.0,10.0,//position
                2.0,0.0,0.0,//viewpoint
                0.0,0.0,1.0);//up
}

void MainWindow::writeXMLSlot()
{
    QString filename  = QFileDialog::getSaveFileName(this, "Save XML annotation file (clicked)", this->c->resourceFolder);

    if(!filename.isEmpty()){
        std::cout << "filename : " << filename.toStdString() << std::endl;
        this->c->xmlManager.xmlWritePath = filename;
        this->c->xmlManager.writeClickedObjects(this->c->objects);
    }
}

void MainWindow::writeXMLInterpolatedSlot()
{
    QString filename  = QFileDialog::getSaveFileName(this, "Save XML annotation file (Interpolated)", this->c->resourceFolder);
    if(!filename.isEmpty()){
        std::cout << "filename : " << filename.toStdString() << std::endl;
        this->c->xmlManager.xmlInterpolatedWritePath = filename;
        this->c->xmlManager.writeInterpolatedObjects(this->c->objects);
    }
}

void MainWindow::interpolatePosesSlot()
{
    c->selectedObject->interpolate3DAnnotationsLinearly();
}

void MainWindow::load3DXMLAnnotationsSlot()
{
    QString selectedXMLFile = QFileDialog::getOpenFileName(this, "Select XML 3D Annotation File", QString(),  "XML Annotation 3D (*.xml)");
    if(!selectedXMLFile.isEmpty()){
        this->c->xmlManager.xmlReadPath = selectedXMLFile;
        //this->c->clearObjects();
        //this->c->objects = this->c->xmlManager.readClickedObjects();
        this->c->objects = this->c->xmlManager.readClickedObjects();
    }
}

void MainWindow::load2DSQLAnnotationsSlot()
{
    QString selectedSQLFile = QFileDialog::getOpenFileName(this, "Select SQL 2D Annotation File", QString(),  "XML Annotation 3D (*.sqlite)");
    if(!selectedSQLFile.isEmpty()){
        this->c->dbFilepath = selectedSQLFile;
        this->c->loadDatabase(this->c->dbFilepath);
        this->c->initializeObjects();
    }
}

void MainWindow::txChanged(double o)
{
    c->currentTx = o;
    this->drawBoundingBox();

}

void MainWindow::tyChanged(double o)
{
    c->currentTy = o;
    this->drawBoundingBox();

}

void MainWindow::tzChanged(double o)
{
    c->currentTz = o;
    this->drawBoundingBox();

}

void MainWindow::thetaChanged(double o)
{
    c->currentTheta = o;
    this->drawBoundingBox();

}

void MainWindow::addAnnotationSlot()
{
    Pose3D pose;
    pose.x = this->c->currentTx;
    pose.y = this->c->currentTy;
    pose.z = this->c->currentTz;
    pose.theta = this->c->currentTheta;
    pose.phi = 0.0;
    pose.psi = 0.0;

    this->c->selectedObject->annotations3D[this->c->currentFrameNumber] = pose;
    QString debugMessage = QString("annotationAdded : object id %1, frame %2").arg(c->selectedObject->id).arg(c->currentFrameNumber);
    std::cout << debugMessage.toStdString() << std::endl;
}

void MainWindow::openImageFolderSlot()
{
    QString selectedFolder = QFileDialog::getExistingDirectory(this, "Select Image folder");
    if(!selectedFolder.isEmpty()){
        QDir export_folder(selectedFolder);
        export_folder.setNameFilters(QStringList()<<"*.png");
        QStringList fileList = export_folder.entryList();
        if(!fileList.isEmpty()){
            QString firstFile = fileList.at(0);
            QStringList splitedFileName = firstFile.split("_");
            QString lastPart = splitedFileName.last();
            int lastPartSize = lastPart.size();
            QString prefix = firstFile.left(firstFile.length()-lastPartSize);
            std::cout << "(MainWindow::openImageFolderSlot) folder chosen : " << selectedFolder.toStdString() << std::endl;
            std::cout << "(MainWindow::openImageFolderSlot) prefix found : " << prefix.toStdString() << std::endl;


            QMessageBox::StandardButton reply, reply2;
            QString questionString = QString("The found image prefix is :\n").append(prefix);
            reply = QMessageBox::question(this, "Test", questionString,QMessageBox::Yes|QMessageBox::No);
            if(reply==QMessageBox::Yes){
                c->resourceFolder = selectedFolder;
                c->imageNamePattern =  prefix;
                c->maxResources = fileList.size()-1; //TODO verify the -1
                reply2 = QMessageBox::question(this, "Test", "Do you want to load the CSV ?",QMessageBox::Yes|QMessageBox::No);
                if(reply2 == QMessageBox::Yes){
                    openCSVFileSlot();
                }
                c->currentFrameNumber = 0;
                initInterface();
                updateViewers();

            }else{
                //TODO Complete with dialog "set the pattern"
                //QString QInputDialog::getText ( QWidget * parent, const QString & title, const QString & label, QLineEdit::EchoMode mode = QLineEdit::Normal, const QString & text = QString(), bool * ok = 0, Qt::WindowFlags f = 0 );
                QString newImagePattern = QInputDialog::getText(this, "New image name pattern", "Enter the new pattern for image names", QLineEdit::Normal, prefix);
                if(!newImagePattern.isEmpty()){
                    QString testImageName = QDir::cleanPath(selectedFolder + QDir::separator() + newImagePattern.append("0.png"));
                    std::cout << "testImageName : " << testImageName.toStdString() << std::endl;
                    QFile testFile(testImageName);
                    if(testFile.exists()){
                        c->resourceFolder = selectedFolder;
                        c->imageNamePattern =  newImagePattern;
                        c->maxResources = fileList.size()-1; //TODO verify the -1
                        reply2 = QMessageBox::question(this, "Test", "Do you want to load the CSV ?",QMessageBox::Yes|QMessageBox::No);
                        if(reply2 == QMessageBox::Yes){
                            openCSVFileSlot();
                        }
                        c->currentFrameNumber = 0;
                        initInterface();
                        updateViewers();
                    }else{
                        std::cerr << "(MainWindow::openImageFolderSlot) file with new pattern does not exist" << std::endl;
                    }
                }
            }
        }
    }
}

void MainWindow::openCSVFileSlot()
{
    QString csvfileNameSelected = QFileDialog::getOpenFileName(this, "Select CSV File", c->resourceFolder,  "Point Cloud CSV (*.csv)");
    std::cout << "(openCSVFileSlot) csvfileNameSelected : " << csvfileNameSelected.toStdString() << std::endl;
    if(!csvfileNameSelected.isEmpty()){
        c->csvFilename = csvfileNameSelected;
        c->loadPointCloudsASCII(csvfileNameSelected);
        c->currentFrameNumber = 0;
        c->loadPointCloudPCL();

    }
}

void MainWindow::drawBoundingBox()
{
    this->viewer->removeShape ("1 edge");
    this->viewer->removeShape ("2 edge");
    this->viewer->removeShape ("3 edge");
    this->viewer->removeShape ("4 edge");
    this->viewer->removeShape ("5 edge");
    this->viewer->removeShape ("6 edge");
    this->viewer->removeShape ("7 edge");
    this->viewer->removeShape ("8 edge");
    this->viewer->removeShape ("9 edge");
    this->viewer->removeShape ("10 edge");
    this->viewer->removeShape ("11 edge");
    this->viewer->removeShape ("12 edge");

    float w = c->selectedObject->width;
    float h = c->selectedObject->height;
    float l = c->selectedObject->length;
    Eigen::Matrix3f rotational_matrix = Eigen::Matrix3f::Identity();
    Eigen::Vector3f position;

    position(0) = c->currentTx;
    position(1) = c->currentTy;
    position(2) = c->currentTz;

    double radDegrees = ( c->currentTheta * M_PI ) / 180.0 ;

    rotational_matrix(0,0) = cos(radDegrees);
    rotational_matrix(0,1) = -sin(radDegrees);
    rotational_matrix(1,0) = sin(radDegrees);
    rotational_matrix(1,1) = cos(radDegrees);

    Eigen::Vector3f p1 (-l/2.0, w/2.0, -h/2.0);
    Eigen::Vector3f p2 (-l/2.0, w/2.0, h/2.0);
    Eigen::Vector3f p3 (l/2.0, w/2.0, h/2.0);
    Eigen::Vector3f p4 (l/2.0, w/2.0, -h/2.0);
    Eigen::Vector3f p5 (-l/2.0, -w/2.0, -h/2.0);
    Eigen::Vector3f p6 (-l/2.0, -w/2.0, h/2.0);
    Eigen::Vector3f p7 (l/2.0, -w/2.0, h/2.0);
    Eigen::Vector3f p8 (l/2.0, -w/2.0, -h/2.0);

    p1 = rotational_matrix * p1 + position;
    p2 = rotational_matrix * p2 + position;
    p3 = rotational_matrix * p3 + position;
    p4 = rotational_matrix * p4 + position;
    p5 = rotational_matrix * p5 + position;
    p6 = rotational_matrix * p6 + position;
    p7 = rotational_matrix * p7 + position;
    p8 = rotational_matrix * p8 + position;

    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

    this->viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
    this->viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
    this->viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
    this->viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
    this->viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
    this->viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
    this->viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
    this->viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
    this->viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
    this->viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
    this->viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
    this->viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");
}

void MainWindow::drawClickedBoundingBox()
{
    std::cout <<"(drawClickedBoundingBoxes) begin" << std::endl;
    this->viewer->removeShape ("1 edge clicked");
    this->viewer->removeShape ("2 edge clicked");
    this->viewer->removeShape ("3 edge clicked");
    this->viewer->removeShape ("4 edge clicked");
    this->viewer->removeShape ("5 edge clicked");
    this->viewer->removeShape ("6 edge clicked");
    this->viewer->removeShape ("7 edge clicked");
    this->viewer->removeShape ("8 edge clicked");
    this->viewer->removeShape ("9 edge clicked");
    this->viewer->removeShape ("10 edge clicked");
    this->viewer->removeShape ("11 edge clicked");
    this->viewer->removeShape ("12 edge clicked");
    if(c->selectedObject){
        if(c->selectedObject->annotations3D.find(c->currentFrameNumber) != c->selectedObject->annotations3D.end()){
            float w = c->selectedObject->width;
            float h = c->selectedObject->height;
            float l = c->selectedObject->length;
            Eigen::Matrix3f rotational_matrix = Eigen::Matrix3f::Identity();
            Eigen::Vector3f position;


            Pose3D clickedPose = c->selectedObject->annotations3D[c->currentFrameNumber];
            position(0) = clickedPose.x;
            position(1) = clickedPose.y;
            position(2) = clickedPose.z;

            double radDegrees = ( clickedPose.theta * M_PI ) / 180.0 ;

            rotational_matrix(0,0) = cos(radDegrees);
            rotational_matrix(0,1) = -sin(radDegrees);
            rotational_matrix(1,0) = sin(radDegrees);
            rotational_matrix(1,1) = cos(radDegrees);

            Eigen::Vector3f p1 (-l/2.0, w/2.0, -h/2.0);
            Eigen::Vector3f p2 (-l/2.0, w/2.0, h/2.0);
            Eigen::Vector3f p3 (l/2.0, w/2.0, h/2.0);
            Eigen::Vector3f p4 (l/2.0, w/2.0, -h/2.0);
            Eigen::Vector3f p5 (-l/2.0, -w/2.0, -h/2.0);
            Eigen::Vector3f p6 (-l/2.0, -w/2.0, h/2.0);
            Eigen::Vector3f p7 (l/2.0, -w/2.0, h/2.0);
            Eigen::Vector3f p8 (l/2.0, -w/2.0, -h/2.0);

            p1 = rotational_matrix * p1 + position;
            p2 = rotational_matrix * p2 + position;
            p3 = rotational_matrix * p3 + position;
            p4 = rotational_matrix * p4 + position;
            p5 = rotational_matrix * p5 + position;
            p6 = rotational_matrix * p6 + position;
            p7 = rotational_matrix * p7 + position;
            p8 = rotational_matrix * p8 + position;

            pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
            pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
            pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
            pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
            pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
            pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
            pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
            pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

            this->viewer->addLine (pt1, pt2, 0.0, 1.0, 0.0, "1 edge clicked");
            this->viewer->addLine (pt1, pt4, 0.0, 1.0, 0.0, "2 edge clicked");
            this->viewer->addLine (pt1, pt5, 0.0, 1.0, 0.0, "3 edge clicked");
            this->viewer->addLine (pt5, pt6, 0.0, 1.0, 0.0, "4 edge clicked");
            this->viewer->addLine (pt5, pt8, 0.0, 1.0, 0.0, "5 edge clicked");
            this->viewer->addLine (pt2, pt6, 0.0, 1.0, 0.0, "6 edge clicked");
            this->viewer->addLine (pt6, pt7, 0.0, 1.0, 0.0, "7 edge clicked");
            this->viewer->addLine (pt7, pt8, 0.0, 1.0, 0.0, "8 edge clicked");
            this->viewer->addLine (pt2, pt3, 0.0, 1.0, 0.0, "9 edge clicked");
            this->viewer->addLine (pt4, pt8, 0.0, 1.0, 0.0, "10 edge clicked");
            this->viewer->addLine (pt3, pt4, 0.0, 1.0, 0.0, "11 edge clicked");
            this->viewer->addLine (pt3, pt7, 0.0, 1.0, 0.0, "12 edge clicked");
        }
    }
    std::cout <<"(drawClickedBoundingBoxes) end" << std::endl;
}



void MainWindow::drawInterpolatedBoundingBoxes()
{
    std::cout <<"(drawInterpolatedBoundingBoxes) begin" << std::endl;
    this->viewer->removeShape ("1 edge inter");
    this->viewer->removeShape ("2 edge inter");
    this->viewer->removeShape ("3 edge inter");
    this->viewer->removeShape ("4 edge inter");
    this->viewer->removeShape ("5 edge inter");
    this->viewer->removeShape ("6 edge inter");
    this->viewer->removeShape ("7 edge inter");
    this->viewer->removeShape ("8 edge inter");
    this->viewer->removeShape ("9 edge inter");
    this->viewer->removeShape ("10 edge inter");
    this->viewer->removeShape ("11 edge inter");
    this->viewer->removeShape ("12 edge inter");
    if(c->selectedObject){
        if(c->selectedObject->annotations3DInterpolated.find(c->currentFrameNumber) != c->selectedObject->annotations3DInterpolated.end()){
            float w = c->selectedObject->width;
            float h = c->selectedObject->height;
            float l = c->selectedObject->length;
            Eigen::Matrix3f rotational_matrix = Eigen::Matrix3f::Identity();
            Eigen::Vector3f position;


            Pose3D interpolatedPose = c->selectedObject->annotations3DInterpolated[c->currentFrameNumber];
            position(0) = interpolatedPose.x;
            position(1) = interpolatedPose.y;
            position(2) = interpolatedPose.z;

            double radDegrees = ( interpolatedPose.theta * M_PI ) / 180.0 ;

            rotational_matrix(0,0) = cos(radDegrees);
            rotational_matrix(0,1) = -sin(radDegrees);
            rotational_matrix(1,0) = sin(radDegrees);
            rotational_matrix(1,1) = cos(radDegrees);

            Eigen::Vector3f p1 (-l/2.0, w/2.0, -h/2.0);
            Eigen::Vector3f p2 (-l/2.0, w/2.0, h/2.0);
            Eigen::Vector3f p3 (l/2.0, w/2.0, h/2.0);
            Eigen::Vector3f p4 (l/2.0, w/2.0, -h/2.0);
            Eigen::Vector3f p5 (-l/2.0, -w/2.0, -h/2.0);
            Eigen::Vector3f p6 (-l/2.0, -w/2.0, h/2.0);
            Eigen::Vector3f p7 (l/2.0, -w/2.0, h/2.0);
            Eigen::Vector3f p8 (l/2.0, -w/2.0, -h/2.0);

            p1 = rotational_matrix * p1 + position;
            p2 = rotational_matrix * p2 + position;
            p3 = rotational_matrix * p3 + position;
            p4 = rotational_matrix * p4 + position;
            p5 = rotational_matrix * p5 + position;
            p6 = rotational_matrix * p6 + position;
            p7 = rotational_matrix * p7 + position;
            p8 = rotational_matrix * p8 + position;

            pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
            pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
            pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
            pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
            pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
            pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
            pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
            pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

            this->viewer->addLine (pt1, pt2, 0.0, 0.0, 1.0, "1 edge inter");
            this->viewer->addLine (pt1, pt4, 0.0, 0.0, 1.0, "2 edge inter");
            this->viewer->addLine (pt1, pt5, 0.0, 0.0, 1.0, "3 edge inter");
            this->viewer->addLine (pt5, pt6, 0.0, 0.0, 1.0, "4 edge inter");
            this->viewer->addLine (pt5, pt8, 0.0, 0.0, 1.0, "5 edge inter");
            this->viewer->addLine (pt2, pt6, 0.0, 0.0, 1.0, "6 edge inter");
            this->viewer->addLine (pt6, pt7, 0.0, 0.0, 1.0, "7 edge inter");
            this->viewer->addLine (pt7, pt8, 0.0, 0.0, 1.0, "8 edge inter");
            this->viewer->addLine (pt2, pt3, 0.0, 0.0, 1.0, "9 edge inter");
            this->viewer->addLine (pt4, pt8, 0.0, 0.0, 1.0, "10 edge inter");
            this->viewer->addLine (pt3, pt4, 0.0, 0.0, 1.0, "11 edge inter");
            this->viewer->addLine (pt3, pt7, 0.0, 0.0, 1.0, "12 edge inter");
        }
    }
    std::cout <<"(drawInterpolatedBoundingBoxes) end" << std::endl;
}

void MainWindow::initInterface()
{
    this->c->initializeResources();
    if(c->imageLoaded && c->lidarLoaded){
        this->imageLabel->setPixmap(QPixmap::fromImage(*c->currentImage));
        //this->viewer
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(c->currentCloud, 255,255,255);
        this->viewer->addPointCloud<pcl::PointXYZ> (c->currentCloud, cloud_color, "cloud");
    }
}
