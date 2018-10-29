/****************************************************************************
** Meta object code from reading C++ file 'BilletMeasurement.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../BilletMeasurement.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'BilletMeasurement.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_BilletMeasurement_t {
    QByteArrayData data[21];
    char stringdata0[280];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BilletMeasurement_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BilletMeasurement_t qt_meta_stringdata_BilletMeasurement = {
    {
QT_MOC_LITERAL(0, 0, 17), // "BilletMeasurement"
QT_MOC_LITERAL(1, 18, 3), // "aaa"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 6), // "onOpen"
QT_MOC_LITERAL(4, 30, 14), // "SlotCamConnect"
QT_MOC_LITERAL(5, 45, 17), // "SlotCamDisconnect"
QT_MOC_LITERAL(6, 63, 13), // "SlotStartGrab"
QT_MOC_LITERAL(7, 77, 13), // "SlotPauseGrab"
QT_MOC_LITERAL(8, 91, 12), // "SlotOpenSync"
QT_MOC_LITERAL(9, 104, 13), // "SlotCloseSync"
QT_MOC_LITERAL(10, 118, 11), // "SlotSaveImg"
QT_MOC_LITERAL(11, 130, 20), // "TreeWidgetOpenEditor"
QT_MOC_LITERAL(12, 151, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(13, 168, 4), // "item"
QT_MOC_LITERAL(14, 173, 3), // "col"
QT_MOC_LITERAL(15, 177, 21), // "TreeWidgetCloseEditor"
QT_MOC_LITERAL(16, 199, 21), // "SlotSetExTriggerParam"
QT_MOC_LITERAL(17, 221, 16), // "SlotTriggerBtnOk"
QT_MOC_LITERAL(18, 238, 20), // "SlotTriggerBtnCancel"
QT_MOC_LITERAL(19, 259, 11), // "GetUserData"
QT_MOC_LITERAL(20, 271, 8) // "InitSlot"

    },
    "BilletMeasurement\0aaa\0\0onOpen\0"
    "SlotCamConnect\0SlotCamDisconnect\0"
    "SlotStartGrab\0SlotPauseGrab\0SlotOpenSync\0"
    "SlotCloseSync\0SlotSaveImg\0"
    "TreeWidgetOpenEditor\0QTreeWidgetItem*\0"
    "item\0col\0TreeWidgetCloseEditor\0"
    "SlotSetExTriggerParam\0SlotTriggerBtnOk\0"
    "SlotTriggerBtnCancel\0GetUserData\0"
    "InitSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BilletMeasurement[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   95,    2, 0x0a /* Public */,
       4,    0,   96,    2, 0x0a /* Public */,
       5,    0,   97,    2, 0x0a /* Public */,
       6,    0,   98,    2, 0x0a /* Public */,
       7,    0,   99,    2, 0x0a /* Public */,
       8,    0,  100,    2, 0x0a /* Public */,
       9,    0,  101,    2, 0x0a /* Public */,
      10,    0,  102,    2, 0x0a /* Public */,
      11,    2,  103,    2, 0x0a /* Public */,
      15,    0,  108,    2, 0x0a /* Public */,
      16,    0,  109,    2, 0x0a /* Public */,
      17,    0,  110,    2, 0x0a /* Public */,
      18,    0,  111,    2, 0x0a /* Public */,
      19,    0,  112,    2, 0x0a /* Public */,
      20,    0,  113,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12, QMetaType::Int,   13,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void BilletMeasurement::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BilletMeasurement *_t = static_cast<BilletMeasurement *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->aaa(); break;
        case 1: _t->onOpen(); break;
        case 2: _t->SlotCamConnect(); break;
        case 3: _t->SlotCamDisconnect(); break;
        case 4: _t->SlotStartGrab(); break;
        case 5: _t->SlotPauseGrab(); break;
        case 6: _t->SlotOpenSync(); break;
        case 7: _t->SlotCloseSync(); break;
        case 8: _t->SlotSaveImg(); break;
        case 9: _t->TreeWidgetOpenEditor((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 10: _t->TreeWidgetCloseEditor(); break;
        case 11: _t->SlotSetExTriggerParam(); break;
        case 12: _t->SlotTriggerBtnOk(); break;
        case 13: _t->SlotTriggerBtnCancel(); break;
        case 14: _t->GetUserData(); break;
        case 15: _t->InitSlot(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (BilletMeasurement::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BilletMeasurement::aaa)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject BilletMeasurement::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_BilletMeasurement.data,
      qt_meta_data_BilletMeasurement,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BilletMeasurement::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BilletMeasurement::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BilletMeasurement.stringdata0))
        return static_cast<void*>(const_cast< BilletMeasurement*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int BilletMeasurement::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void BilletMeasurement::aaa()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
