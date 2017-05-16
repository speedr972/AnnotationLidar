/****************************************************************************
** Meta object code from reading C++ file 'realobjectinformationwidget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../realobjectinformationwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'realobjectinformationwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_RealObjectInformationWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      34,   29,   28,   28, 0x0a,
      69,   60,   28,   28, 0x0a,
     101,   91,   28,   28, 0x0a,
     134,  124,   28,   28, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_RealObjectInformationWidget[] = {
    "RealObjectInformationWidget\0\0robj\0"
    "updateWidget(RealObject*)\0newWidth\0"
    "widthChanged(QString)\0newHeight\0"
    "heightChanged(QString)\0newLength\0"
    "lengthChanged(QString)\0"
};

void RealObjectInformationWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        RealObjectInformationWidget *_t = static_cast<RealObjectInformationWidget *>(_o);
        switch (_id) {
        case 0: _t->updateWidget((*reinterpret_cast< RealObject*(*)>(_a[1]))); break;
        case 1: _t->widthChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->heightChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->lengthChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData RealObjectInformationWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject RealObjectInformationWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_RealObjectInformationWidget,
      qt_meta_data_RealObjectInformationWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &RealObjectInformationWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *RealObjectInformationWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *RealObjectInformationWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RealObjectInformationWidget))
        return static_cast<void*>(const_cast< RealObjectInformationWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int RealObjectInformationWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
