/****************************************************************************
** Meta object code from reading C++ file 'Buttons.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Buttons.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Buttons.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cnoid__PushButton_t {
    QByteArrayData data[5];
    char stringdata0[47];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__PushButton_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__PushButton_t qt_meta_stringdata_cnoid__PushButton = {
    {
QT_MOC_LITERAL(0, 0, 17), // "cnoid::PushButton"
QT_MOC_LITERAL(1, 18, 9), // "onClicked"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 7), // "checked"
QT_MOC_LITERAL(4, 37, 9) // "onToggled"

    },
    "cnoid::PushButton\0onClicked\0\0checked\0"
    "onToggled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__PushButton[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x08 /* Private */,
       4,    1,   27,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,

       0        // eod
};

void cnoid::PushButton::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PushButton *_t = static_cast<PushButton *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->onToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::PushButton::staticMetaObject = {
    { &QPushButton::staticMetaObject, qt_meta_stringdata_cnoid__PushButton.data,
      qt_meta_data_cnoid__PushButton,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::PushButton::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::PushButton::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__PushButton.stringdata0))
        return static_cast<void*>(const_cast< PushButton*>(this));
    return QPushButton::qt_metacast(_clname);
}

int cnoid::PushButton::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QPushButton::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
struct qt_meta_stringdata_cnoid__RadioButton_t {
    QByteArrayData data[4];
    char stringdata0[38];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__RadioButton_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__RadioButton_t qt_meta_stringdata_cnoid__RadioButton = {
    {
QT_MOC_LITERAL(0, 0, 18), // "cnoid::RadioButton"
QT_MOC_LITERAL(1, 19, 9), // "onToggled"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 7) // "checked"

    },
    "cnoid::RadioButton\0onToggled\0\0checked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__RadioButton[] = {

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
    QMetaType::Void, QMetaType::Bool,    3,

       0        // eod
};

void cnoid::RadioButton::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RadioButton *_t = static_cast<RadioButton *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::RadioButton::staticMetaObject = {
    { &QRadioButton::staticMetaObject, qt_meta_stringdata_cnoid__RadioButton.data,
      qt_meta_data_cnoid__RadioButton,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::RadioButton::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::RadioButton::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__RadioButton.stringdata0))
        return static_cast<void*>(const_cast< RadioButton*>(this));
    return QRadioButton::qt_metacast(_clname);
}

int cnoid::RadioButton::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QRadioButton::qt_metacall(_c, _id, _a);
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
struct qt_meta_stringdata_cnoid__ToolButton_t {
    QByteArrayData data[7];
    char stringdata0[68];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__ToolButton_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__ToolButton_t qt_meta_stringdata_cnoid__ToolButton = {
    {
QT_MOC_LITERAL(0, 0, 17), // "cnoid::ToolButton"
QT_MOC_LITERAL(1, 18, 9), // "onClicked"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 7), // "checked"
QT_MOC_LITERAL(4, 37, 9), // "onToggled"
QT_MOC_LITERAL(5, 47, 9), // "onPressed"
QT_MOC_LITERAL(6, 57, 10) // "onReleased"

    },
    "cnoid::ToolButton\0onClicked\0\0checked\0"
    "onToggled\0onPressed\0onReleased"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__ToolButton[] = {

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
       5,    0,   40,    2, 0x08 /* Private */,
       6,    0,   41,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void cnoid::ToolButton::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ToolButton *_t = static_cast<ToolButton *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->onToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->onPressed(); break;
        case 3: _t->onReleased(); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::ToolButton::staticMetaObject = {
    { &QToolButton::staticMetaObject, qt_meta_stringdata_cnoid__ToolButton.data,
      qt_meta_data_cnoid__ToolButton,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::ToolButton::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::ToolButton::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__ToolButton.stringdata0))
        return static_cast<void*>(const_cast< ToolButton*>(this));
    return QToolButton::qt_metacast(_clname);
}

int cnoid::ToolButton::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QToolButton::qt_metacall(_c, _id, _a);
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
