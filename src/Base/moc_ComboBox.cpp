/****************************************************************************
** Meta object code from reading C++ file 'ComboBox.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ComboBox.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ComboBox.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cnoid__ComboBox_t {
    QByteArrayData data[8];
    char stringdata0[94];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__ComboBox_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__ComboBox_t qt_meta_stringdata_cnoid__ComboBox = {
    {
QT_MOC_LITERAL(0, 0, 15), // "cnoid::ComboBox"
QT_MOC_LITERAL(1, 16, 11), // "onActivated"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 5), // "index"
QT_MOC_LITERAL(4, 35, 21), // "onCurrentIndexChanged"
QT_MOC_LITERAL(5, 57, 17), // "onEditTextChanged"
QT_MOC_LITERAL(6, 75, 4), // "text"
QT_MOC_LITERAL(7, 80, 13) // "onHighlighted"

    },
    "cnoid::ComboBox\0onActivated\0\0index\0"
    "onCurrentIndexChanged\0onEditTextChanged\0"
    "text\0onHighlighted"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__ComboBox[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x08 /* Private */,
       4,    1,   37,    2, 0x08 /* Private */,
       5,    1,   40,    2, 0x08 /* Private */,
       7,    1,   43,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::Int,    3,

       0        // eod
};

void cnoid::ComboBox::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ComboBox *_t = static_cast<ComboBox *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onActivated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->onCurrentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->onEditTextChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->onHighlighted((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::ComboBox::staticMetaObject = {
    { &QComboBox::staticMetaObject, qt_meta_stringdata_cnoid__ComboBox.data,
      qt_meta_data_cnoid__ComboBox,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::ComboBox::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::ComboBox::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__ComboBox.stringdata0))
        return static_cast<void*>(const_cast< ComboBox*>(this));
    return QComboBox::qt_metacast(_clname);
}

int cnoid::ComboBox::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QComboBox::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
