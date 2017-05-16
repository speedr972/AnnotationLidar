/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      31,   11,   11,   11, 0x0a,
      53,   47,   11,   11, 0x0a,
      79,   11,   11,   11, 0x0a,
      95,   11,   11,   11, 0x0a,
     111,   11,   11,   11, 0x0a,
     127,   11,   11,   11, 0x0a,
     141,   11,   11,   11, 0x0a,
     156,   11,   11,   11, 0x0a,
     183,   11,   11,   11, 0x0a,
     206,   11,   11,   11, 0x0a,
     233,   11,   11,   11, 0x0a,
     262,  260,   11,   11, 0x0a,
     280,  260,   11,   11, 0x0a,
     298,  260,   11,   11, 0x0a,
     316,  260,   11,   11, 0x0a,
     337,   11,   11,   11, 0x0a,
     357,   11,   11,   11, 0x0a,
     379,   11,   11,   11, 0x0a,
     397,   11,   11,   11, 0x0a,
     415,   11,   11,   11, 0x0a,
     440,   11,   11,   11, 0x0a,
     472,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0refreshVtkViewer()\0"
    "updateViewers()\0index\0listViewSlot(QModelIndex)\0"
    "nextFrameSlot()\0prevFrameSlot()\0"
    "frameStepSlot()\0reset3DView()\0"
    "writeXMLSlot()\0writeXMLInterpolatedSlot()\0"
    "interpolatePosesSlot()\0"
    "load3DXMLAnnotationsSlot()\0"
    "load2DSQLAnnotationsSlot()\0o\0"
    "txChanged(double)\0tyChanged(double)\0"
    "tzChanged(double)\0thetaChanged(double)\0"
    "addAnnotationSlot()\0openImageFolderSlot()\0"
    "openCSVFileSlot()\0drawBoundingBox()\0"
    "drawClickedBoundingBox()\0"
    "drawInterpolatedBoundingBoxes()\0"
    "initInterface()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->refreshVtkViewer(); break;
        case 1: _t->updateViewers(); break;
        case 2: _t->listViewSlot((*reinterpret_cast< QModelIndex(*)>(_a[1]))); break;
        case 3: _t->nextFrameSlot(); break;
        case 4: _t->prevFrameSlot(); break;
        case 5: _t->frameStepSlot(); break;
        case 6: _t->reset3DView(); break;
        case 7: _t->writeXMLSlot(); break;
        case 8: _t->writeXMLInterpolatedSlot(); break;
        case 9: _t->interpolatePosesSlot(); break;
        case 10: _t->load3DXMLAnnotationsSlot(); break;
        case 11: _t->load2DSQLAnnotationsSlot(); break;
        case 12: _t->txChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->tyChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 14: _t->tzChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 15: _t->thetaChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 16: _t->addAnnotationSlot(); break;
        case 17: _t->openImageFolderSlot(); break;
        case 18: _t->openCSVFileSlot(); break;
        case 19: _t->drawBoundingBox(); break;
        case 20: _t->drawClickedBoundingBox(); break;
        case 21: _t->drawInterpolatedBoundingBoxes(); break;
        case 22: _t->initInterface(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
