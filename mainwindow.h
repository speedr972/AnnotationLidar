#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QTimer>
#include <QTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QLabel>
#include <QListView>
#include <QStringListModel>
#include <QDoubleSpinBox>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QDebug>
#include <QShortcut>
#include <qmath.h>
#include <cmath>

#include "context.h"
#include "realobject.h"
#include "realobjectinformationwidget.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>


#include <vtkRenderWindow.h>
#include <QVTKWidget.h>



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void initObjectListView();

    //boost::function<void (int values[], int n, int& sum, float& avg)> sum_avg;
    //boost::function<void (const pcl::visualization::AreaPickingEvent& event)> selectedObjectCallback;
    void areaEventOccured(const pcl::visualization::AreaPickingEvent& event, void *nothing);

    Context *c;

    QLineEdit *resourceFolderEdit;
    QLineEdit *imageNamePatternEdit;
    QLineEdit *csvFileEdit;
    QLineEdit *resourceNumberEdit;



    QLabel *imageLabel;
    QLabel *frameNumberLabel;
    QVTKWidget *vtkViewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    QPushButton *nextResButton;
    QPushButton *prevResButton;
    QLineEdit *frameStepEdit;
    QPushButton *loadFolderButton;
    QPushButton *loadAnnotationsButton;
    QPushButton *reset3DViewButton;
    QPushButton *writeXMLButton;
    QPushButton *interpolatePosesButton;
    QPushButton *writeXMLInterpolatedButton;

    QListView *objectListView;
    QStringListModel *model;
    QRubberBand *selectionBox;
    QLabel *selectedObjectLabel;
    QPushButton *addAnnotationButton;
    QDoubleSpinBox *thetaSpinbox; //degrees
    QDoubleSpinBox *txSpinbox;
    QDoubleSpinBox *tySpinbox;
    QDoubleSpinBox *tzSpinbox;

    RealObjectInformationWidget *objInfoWidget;


    QAction *openImageFolderAction;
    QAction *openCSVFileAction;
    QAction *load2DSQLAnnotationsAction;
    QAction *load3DXMLAnnotationsAction;
    QAction *load3DXMLAnnotationsInterpolatedAction;
    QAction *save3DXMLAnnotationsAction;
    QAction *save3DXMLAnnotationsInterpolatedAction;

    //QShortcut *save3DXMLAnnotationsShortcut;









    //pcl::visualization::PCLVisualizer viewer;


    QTimer *timer;
    int delta;
    int frameStep;

signals:

public slots:
    void refreshVtkViewer();
    void updateViewers();
    void listViewSlot(QModelIndex index);
    void nextFrameSlot();
    void prevFrameSlot();
    void frameStepSlot();
    void reset3DView();
    void writeXMLSlot();
    void writeXMLInterpolatedSlot();
    void interpolatePosesSlot();
    void load3DXMLAnnotationsSlot();
    void load2DSQLAnnotationsSlot();


    void txChanged(double o);
    void tyChanged(double o);
    void tzChanged(double o);
    void thetaChanged(double o);
    void addAnnotationSlot();

    void openImageFolderSlot();
    void openCSVFileSlot();

    void drawBoundingBox();
    void drawClickedBoundingBox();
    void drawInterpolatedBoundingBoxes();

    void initInterface();







};

#endif // MAINWINDOW_H
