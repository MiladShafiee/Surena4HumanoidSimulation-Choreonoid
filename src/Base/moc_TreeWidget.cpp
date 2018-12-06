/****************************************************************************
** Meta object code from reading C++ file 'TreeWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "TreeWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TreeWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cnoid__TreeWidget_t {
    QByteArrayData data[17];
    char stringdata0[232];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__TreeWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__TreeWidget_t qt_meta_stringdata_cnoid__TreeWidget = {
    {
QT_MOC_LITERAL(0, 0, 17), // "cnoid::TreeWidget"
QT_MOC_LITERAL(1, 18, 20), // "onCurrentItemChanged"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(4, 57, 7), // "current"
QT_MOC_LITERAL(5, 65, 8), // "previous"
QT_MOC_LITERAL(6, 74, 15), // "onItemActivated"
QT_MOC_LITERAL(7, 90, 4), // "item"
QT_MOC_LITERAL(8, 95, 6), // "column"
QT_MOC_LITERAL(9, 102, 13), // "onItemChanged"
QT_MOC_LITERAL(10, 116, 13), // "onItemClicked"
QT_MOC_LITERAL(11, 130, 15), // "onItemCollapsed"
QT_MOC_LITERAL(12, 146, 19), // "onItemDoubleClicked"
QT_MOC_LITERAL(13, 166, 13), // "onItemEntered"
QT_MOC_LITERAL(14, 180, 14), // "onItemExpanded"
QT_MOC_LITERAL(15, 195, 13), // "onItemPressed"
QT_MOC_LITERAL(16, 209, 22) // "onItemSelectionChanged"

    },
    "cnoid::TreeWidget\0onCurrentItemChanged\0"
    "\0QTreeWidgetItem*\0current\0previous\0"
    "onItemActivated\0item\0column\0onItemChanged\0"
    "onItemClicked\0onItemCollapsed\0"
    "onItemDoubleClicked\0onItemEntered\0"
    "onItemExpanded\0onItemPressed\0"
    "onItemSelectionChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__TreeWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   64,    2, 0x08 /* Private */,
       6,    2,   69,    2, 0x08 /* Private */,
       9,    2,   74,    2, 0x08 /* Private */,
      10,    2,   79,    2, 0x08 /* Private */,
      11,    1,   84,    2, 0x08 /* Private */,
      12,    2,   87,    2, 0x08 /* Private */,
      13,    2,   92,    2, 0x08 /* Private */,
      14,    1,   97,    2, 0x08 /* Private */,
      15,    2,  100,    2, 0x08 /* Private */,
      16,    0,  105,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,    5,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    7,    8,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    7,    8,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    7,    8,
    QMetaType::Void, 0x80000000 | 3,    7,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    7,    8,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    7,    8,
    QMetaType::Void, 0x80000000 | 3,    7,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    7,    8,
    QMetaType::Void,

       0        // eod
};

void cnoid::TreeWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TreeWidget *_t = static_cast<TreeWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onCurrentItemChanged((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< QTreeWidgetItem*(*)>(_a[2]))); break;
        case 1: _t->onItemActivated((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->onItemChanged((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->onItemClicked((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->onItemCollapsed((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1]))); break;
        case 5: _t->onItemDoubleClicked((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 6: _t->onItemEntered((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 7: _t->onItemExpanded((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1]))); break;
        case 8: _t->onItemPressed((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 9: _t->onItemSelectionChanged(); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::TreeWidget::staticMetaObject = {
    { &QTreeWidget::staticMetaObject, qt_meta_stringdata_cnoid__TreeWidget.data,
      qt_meta_data_cnoid__TreeWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::TreeWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::TreeWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__TreeWidget.stringdata0))
        return static_cast<void*>(const_cast< TreeWidget*>(this));
    return QTreeWidget::qt_metacast(_clname);
}

int cnoid::TreeWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTreeWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
