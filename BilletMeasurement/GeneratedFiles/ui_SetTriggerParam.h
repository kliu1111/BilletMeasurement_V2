/********************************************************************************
** Form generated from reading UI file 'SetTriggerParam.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETTRIGGERPARAM_H
#define UI_SETTRIGGERPARAM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SetTriggerParam
{
public:
    QPushButton *Btn_Cancel;
    QPushButton *Btn_Ok;
    QSplitter *splitter;
    QLabel *label;
    QLineEdit *lineEdit;

    void setupUi(QWidget *SetTriggerParam)
    {
        if (SetTriggerParam->objectName().isEmpty())
            SetTriggerParam->setObjectName(QStringLiteral("SetTriggerParam"));
        SetTriggerParam->resize(400, 300);
        Btn_Cancel = new QPushButton(SetTriggerParam);
        Btn_Cancel->setObjectName(QStringLiteral("Btn_Cancel"));
        Btn_Cancel->setGeometry(QRect(210, 150, 73, 23));
        Btn_Ok = new QPushButton(SetTriggerParam);
        Btn_Ok->setObjectName(QStringLiteral("Btn_Ok"));
        Btn_Ok->setGeometry(QRect(80, 150, 73, 23));
        splitter = new QSplitter(SetTriggerParam);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setGeometry(QRect(90, 110, 182, 20));
        splitter->setOrientation(Qt::Horizontal);
        label = new QLabel(splitter);
        label->setObjectName(QStringLiteral("label"));
        splitter->addWidget(label);
        lineEdit = new QLineEdit(splitter);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        splitter->addWidget(lineEdit);

        retranslateUi(SetTriggerParam);

        QMetaObject::connectSlotsByName(SetTriggerParam);
    } // setupUi

    void retranslateUi(QWidget *SetTriggerParam)
    {
        SetTriggerParam->setWindowTitle(QApplication::translate("SetTriggerParam", "SetTriggerParam", 0));
        Btn_Cancel->setText(QApplication::translate("SetTriggerParam", "\345\217\226\346\266\210", 0));
        Btn_Ok->setText(QApplication::translate("SetTriggerParam", "\347\241\256\345\256\232", 0));
        Btn_Ok->setShortcut(QApplication::translate("SetTriggerParam", "Return", 0));
        label->setText(QApplication::translate("SetTriggerParam", "\347\233\270\346\234\272\346\213\215\346\221\204\345\270\247\347\216\207", 0));
    } // retranslateUi

};

namespace Ui {
    class SetTriggerParam: public Ui_SetTriggerParam {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTRIGGERPARAM_H
