/****************************************************************************
** Meta object code from reading C++ file 'LinkTreeWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "LinkTreeWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LinkTreeWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_cnoid__LinkTreeWidget_t {
    QByteArrayData data[13];
    char stringdata0[187];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__LinkTreeWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__LinkTreeWidget_t qt_meta_stringdata_cnoid__LinkTreeWidget = {
    {
QT_MOC_LITERAL(0, 0, 21), // "cnoid::LinkTreeWidget"
QT_MOC_LITERAL(1, 22, 13), // "onItemChanged"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(4, 54, 4), // "item"
QT_MOC_LITERAL(5, 59, 6), // "column"
QT_MOC_LITERAL(6, 66, 18), // "onSelectionChanged"
QT_MOC_LITERAL(7, 85, 28), // "onCustomContextMenuRequested"
QT_MOC_LITERAL(8, 114, 3), // "pos"
QT_MOC_LITERAL(9, 118, 14), // "onItemExpanded"
QT_MOC_LITERAL(10, 133, 14), // "treeWidgetItem"
QT_MOC_LITERAL(11, 148, 15), // "onItemCollapsed"
QT_MOC_LITERAL(12, 164, 22) // "onHeaderSectionResized"

    },
    "cnoid::LinkTreeWidget\0onItemChanged\0"
    "\0QTreeWidgetItem*\0item\0column\0"
    "onSelectionChanged\0onCustomContextMenuRequested\0"
    "pos\0onItemExpanded\0treeWidgetItem\0"
    "onItemCollapsed\0onHeaderSectionResized"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__LinkTreeWidget[] = {

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
       6,    0,   49,    2, 0x08 /* Private */,
       7,    1,   50,    2, 0x08 /* Private */,
       9,    1,   53,    2, 0x08 /* Private */,
      11,    1,   56,    2, 0x08 /* Private */,
      12,    0,   59,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    4,    5,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QPoint,    8,
    QMetaType::Void, 0x80000000 | 3,   10,
    QMetaType::Void, 0x80000000 | 3,   10,
    QMetaType::Void,

       0        // eod
};

void cnoid::LinkTreeWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        LinkTreeWidget *_t = static_cast<LinkTreeWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onItemChanged((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->onSelectionChanged(); break;
        case 2: _t->onCustomContextMenuRequested((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 3: _t->onItemExpanded((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1]))); break;
        case 4: _t->onItemCollapsed((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1]))); break;
        case 5: _t->onHeaderSectionResized(); break;
        default: ;
        }
    }
}

const QMetaObject cnoid::LinkTreeWidget::staticMetaObject = {
    { &TreeWidget::staticMetaObject, qt_meta_stringdata_cnoid__LinkTreeWidget.data,
      qt_meta_data_cnoid__LinkTreeWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *cnoid::LinkTreeWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::LinkTreeWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__LinkTreeWidget.stringdata0))
        return static_cast<void*>(const_cast< LinkTreeWidget*>(this));
    return TreeWidget::qt_metacast(_clname);
}

int cnoid::LinkTreeWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = TreeWidget::qt_metacall(_c, _id, _a);
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
