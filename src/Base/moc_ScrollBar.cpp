/****************************************************************************
** Meta object code from reading C++ file 'ScrollBar.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ScrollBar.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ScrollBar.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cnoid__ScrollBar_t {
    QByteArrayData data[4];
    char stringdata0[39];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__ScrollBar_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__ScrollBar_t qt_meta_stringdata_cnoid__ScrollBar = {
    {
QT_MOC_LITERAL(0, 0, 16), // "cnoid::ScrollBar"
QT_MOC_LITERAL(1, 17, 14), // "onValueChanged"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 5) // "value"

    },
    "cnoid::ScrollBar\0onValueChanged\0\0value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__ScrollBar[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,

       0        // eod
};

void cnoid::ScrollBar::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ScrollBar *_t = static_cast<ScrollBar *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::ScrollBar::staticMetaObject = {
    { &QScrollBar::staticMetaObject, qt_meta_stringdata_cnoid__ScrollBar.data,
      qt_meta_data_cnoid__ScrollBar,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::ScrollBar::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::ScrollBar::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__ScrollBar.stringdata0))
        return static_cast<void*>(const_cast< ScrollBar*>(this));
    return QScrollBar::qt_metacast(_clname);
}

int cnoid::ScrollBar::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QScrollBar::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
struct qt_meta_stringdata_cnoid__DoubleScrollBar_t {
    QByteArrayData data[4];
    char stringdata0[45];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__DoubleScrollBar_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__DoubleScrollBar_t qt_meta_stringdata_cnoid__DoubleScrollBar = {
    {
QT_MOC_LITERAL(0, 0, 22), // "cnoid::DoubleScrollBar"
QT_MOC_LITERAL(1, 23, 14), // "onValueChanged"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 5) // "value"

    },
    "cnoid::DoubleScrollBar\0onValueChanged\0"
    "\0value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__DoubleScrollBar[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,

       0        // eod
};

void cnoid::DoubleScrollBar::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DoubleScrollBar *_t = static_cast<DoubleScrollBar *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::DoubleScrollBar::staticMetaObject = {
    { &QScrollBar::staticMetaObject, qt_meta_stringdata_cnoid__DoubleScrollBar.data,
      qt_meta_data_cnoid__DoubleScrollBar,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::DoubleScrollBar::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::DoubleScrollBar::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__DoubleScrollBar.stringdata0))
        return static_cast<void*>(const_cast< DoubleScrollBar*>(this));
    return QScrollBar::qt_metacast(_clname);
}

int cnoid::DoubleScrollBar::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QScrollBar::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
