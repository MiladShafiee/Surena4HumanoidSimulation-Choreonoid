/****************************************************************************
** Meta object code from reading C++ file 'TextEdit.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "TextEdit.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TextEdit.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cnoid__PlainTextEdit_t {
    QByteArrayData data[3];
    char stringdata0[46];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__PlainTextEdit_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__PlainTextEdit_t qt_meta_stringdata_cnoid__PlainTextEdit = {
    {
QT_MOC_LITERAL(0, 0, 20), // "cnoid::PlainTextEdit"
QT_MOC_LITERAL(1, 21, 23), // "onCursorPositionChanged"
QT_MOC_LITERAL(2, 45, 0) // ""

    },
    "cnoid::PlainTextEdit\0onCursorPositionChanged\0"
    ""
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__PlainTextEdit[] = {

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
       1,    0,   19,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void cnoid::PlainTextEdit::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PlainTextEdit *_t = static_cast<PlainTextEdit *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onCursorPositionChanged(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject cnoid::PlainTextEdit::staticMetaObject = {
    { &QPlainTextEdit::staticMetaObject, qt_meta_stringdata_cnoid__PlainTextEdit.data,
      qt_meta_data_cnoid__PlainTextEdit,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::PlainTextEdit::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::PlainTextEdit::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__PlainTextEdit.stringdata0))
        return static_cast<void*>(const_cast< PlainTextEdit*>(this));
    return QPlainTextEdit::qt_metacast(_clname);
}

int cnoid::PlainTextEdit::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QPlainTextEdit::qt_metacall(_c, _id, _a);
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
struct qt_meta_stringdata_cnoid__TextEdit_t {
    QByteArrayData data[8];
    char stringdata0[101];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__TextEdit_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__TextEdit_t qt_meta_stringdata_cnoid__TextEdit = {
    {
QT_MOC_LITERAL(0, 0, 15), // "cnoid::TextEdit"
QT_MOC_LITERAL(1, 16, 26), // "onCurrentCharFormatChanged"
QT_MOC_LITERAL(2, 43, 0), // ""
QT_MOC_LITERAL(3, 44, 15), // "QTextCharFormat"
QT_MOC_LITERAL(4, 60, 1), // "f"
QT_MOC_LITERAL(5, 62, 23), // "onCursorPositionChanged"
QT_MOC_LITERAL(6, 86, 8), // "onScroll"
QT_MOC_LITERAL(7, 95, 5) // "value"

    },
    "cnoid::TextEdit\0onCurrentCharFormatChanged\0"
    "\0QTextCharFormat\0f\0onCursorPositionChanged\0"
    "onScroll\0value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__TextEdit[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x08 /* Private */,
       5,    0,   32,    2, 0x08 /* Private */,
       6,    1,   33,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    7,

       0        // eod
};

void cnoid::TextEdit::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TextEdit *_t = static_cast<TextEdit *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onCurrentCharFormatChanged((*reinterpret_cast< const QTextCharFormat(*)>(_a[1]))); break;
        case 1: _t->onCursorPositionChanged(); break;
        case 2: _t->onScroll((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::TextEdit::staticMetaObject = {
    { &QTextEdit::staticMetaObject, qt_meta_stringdata_cnoid__TextEdit.data,
      qt_meta_data_cnoid__TextEdit,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::TextEdit::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::TextEdit::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__TextEdit.stringdata0))
        return static_cast<void*>(const_cast< TextEdit*>(this));
    return QTextEdit::qt_metacast(_clname);
}

int cnoid::TextEdit::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTextEdit::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
