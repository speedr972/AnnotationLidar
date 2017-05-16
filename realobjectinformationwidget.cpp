#include "realobjectinformationwidget.h"

RealObjectInformationWidget::RealObjectInformationWidget(QWidget *parent) :
    QWidget(parent)
{
    this->idLabel = new QLabel;
    this->typeLabel = new QLabel;
    this->descriptionLabel = new QLabel;
    this->listAnnotations2DLabel = new QLabel;
    this->listAnnotations3DLabel = new QLabel;
    this->listAnnotations3DLabel->setWordWrap(true);
    this->listAnnotations3DInterpolationLabel = new QLabel;
    this->widthLabel = new QLineEdit;
    this->heightLabel = new QLineEdit;
    this->lengthLabel = new QLineEdit;

    QFormLayout *formLayout = new QFormLayout;
    this->setLayout(formLayout);
    formLayout->addRow("ID : ", this->idLabel);
    formLayout->addRow("Type : ", this->typeLabel);
    formLayout->addRow("Description : ", this->descriptionLabel);
    formLayout->addRow("width : ", this->widthLabel);
    formLayout->addRow("height : ", this->heightLabel);
    formLayout->addRow("length : ", this->lengthLabel);

    formLayout->addRow("List annotation 2D : ", this->listAnnotations2DLabel);
    formLayout->addRow("List annotation 3D : ", this->listAnnotations3DLabel);
    formLayout->addRow("List annotation 3D Interpolated: ", this->listAnnotations3DInterpolationLabel);


    connect(this->widthLabel, SIGNAL(textEdited(QString)), this, SLOT(widthChanged(QString)));
    connect(this->lengthLabel, SIGNAL(textEdited(QString)), this, SLOT(lengthChanged(QString)));
    connect(this->heightLabel, SIGNAL(textEdited(QString)), this, SLOT(heightChanged(QString)));

}



void RealObjectInformationWidget::updateWidget(RealObject *robj)
{
    this->idLabel->setText(QString::number(robj->id));
    this->typeLabel->setText(objectTypeToString(robj->objectType));
    this->descriptionLabel->setText(robj->description);
    this->widthLabel->setText(QString::number(robj->width));
    this->heightLabel->setText(QString::number(robj->height));
    this->lengthLabel->setText(QString::number(robj->length));

    QString annotation2DString;
    int cpt = 0;
    std::map<int, Pose2D>::iterator it2d;
    for(it2d=robj->annotations2D.begin(); it2d!=robj->annotations2D.end(); ++it2d){
        annotation2DString.append(QString("%1, ").arg(it2d->first));
        if(cpt>10){
            annotation2DString.append(QString("..."));
            break;
        }else{
            cpt++;
        }
    }
    this->listAnnotations2DLabel->setText(annotation2DString);

    QString annotation3DString;
    std::map<int, Pose3D>::iterator it3d;
    for(it3d=robj->annotations3D.begin(); it3d!=robj->annotations3D.end(); ++it3d){
        annotation3DString.append(QString("%1, ").arg(it3d->first));
    }
    this->listAnnotations3DLabel->setText(annotation3DString);

}

void RealObjectInformationWidget::widthChanged(QString newWidth)
{
     if(this->robj){
         this->robj->width =  newWidth.toFloat();
     }
}

void RealObjectInformationWidget::heightChanged(QString newHeight)
{
    if(this->robj){
        this->robj->height =  newHeight.toFloat();
    }
}

void RealObjectInformationWidget::lengthChanged(QString newLength)
{
    if(this->robj){
        this->robj->length =  newLength.toFloat();
    }
}




