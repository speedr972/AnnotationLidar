#ifndef REALOBJECTINFORMATIONWIDGET_H
#define REALOBJECTINFORMATIONWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QFormLayout>
#include "poses.h"
#include <map>
#include <iostream>
#include <vector>
#include <iterator>
#include "realobject.h"
#include <QLineEdit>


class RealObjectInformationWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RealObjectInformationWidget(QWidget *parent = 0);

    QLabel *idLabel;
    QLabel *typeLabel;
    QLabel *descriptionLabel;
    QLabel *listAnnotations2DLabel;
    QLabel *listAnnotations3DLabel;
    QLabel *listAnnotations3DInterpolationLabel;
    QLineEdit *widthLabel;
    QLineEdit *heightLabel;
    QLineEdit *lengthLabel;


    RealObject *robj;


signals:

public slots:
    void updateWidget(RealObject *robj);
    void widthChanged(QString newWidth);
    void heightChanged(QString newHeight);
    void lengthChanged(QString newLength);

};

#endif // REALOBJECTINFORMATIONWIDGET_H
