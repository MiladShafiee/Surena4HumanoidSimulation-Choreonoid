/****************************************************************************
** Meta object code from reading C++ file 'LineEdit.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "LineEdit.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LineEdit.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cnoid__LineEdit_t {
    QByteArrayData data[11];
    char stringdata0[140];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__LineEdit_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__LineEdit_t qt_meta_stringdata_cnoid__LineEdit = {
    {
QT_MOC_LITERAL(0, 0, 15), // "cnoid::LineEdit"
QT_MOC_LITERAL(1, 16, 23), // "onCursorPositionChanged"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 6), // "oldpos"
QT_MOC_LITERAL(4, 48, 6), // "newpos"
QT_MOC_LITERAL(5, 55, 17), // "onEditingFinished"
QT_MOC_LITERAL(6, 73, 15), // "onReturnPressed"
QT_MOC_LITERAL(7, 89, 18), // "onSelectionChanged"
QT_MOC_LITERAL(8, 108, 13), // "onTextChanged"
QT_MOC_LITERAL(9, 122, 4), // "text"
QT_MOC_LITERAL(10, 127, 12) // "onTextEdited"

    },
    "cnoid::LineEdit\0onCursorPositionChanged\0"
    "\0oldpos\0newpos\0onEditingFinished\0"
    "onReturnPressed\0onSelectionChanged\0"
    "onTextChanged\0text\0onTextEdited"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__LineEdit[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   44,    2, 0x08 /* Private */,
       5,    0,   49,    2, 0x08 /* Private */,
       6,    0,   50,    2, 0x08 /* Private */,
       7,    0,   51,    2, 0x08 /* Private */,
       8,    1,   52,    2, 0x08 /* Private */,
      10,    1,   55,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    9,
    QMetaType::Void, QMetaType::QString,    9,

       0        // eod
};

void cnoid::LineEdit::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        LineEdit *_t = static_cast<LineEdit *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onCursorPositionChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->onEditingFinished(); break;
        case 2: _t->onReturnPressed(); break;
        case 3: _t->onSelectionChanged(); break;
        case 4: _t->onTextChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->onTextEdited((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::LineEdit::staticMetaObject = {
    { &QLineEdit::staticMetaObject, qt_meta_stringdata_cnoid__LineEdit.data,
      qt_meta_data_cnoid__LineEdit,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::LineEdit::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::LineEdit::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__LineEdit.stringdata0))
        return static_cast<void*>(const_cast< LineEdit*>(this));
    return QLineEdit::qt_metacast(_clname);
}

int cnoid::LineEdit::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLineEdit::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
