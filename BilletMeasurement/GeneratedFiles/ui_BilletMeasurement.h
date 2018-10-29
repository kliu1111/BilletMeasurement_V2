/********************************************************************************
** Form generated from reading UI file 'BilletMeasurement.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BILLETMEASUREMENT_H
#define UI_BILLETMEASUREMENT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_BilletMeasurementClass
{
public:
    QAction *action_connect;
    QAction *action_disconnect;
    QAction *action_start;
    QAction *action_pause;
    QAction *action_setcamparam;
    QAction *action_syncing;
    QAction *actionOpen;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_4;
    QTabWidget *tabWidget;
    QWidget *tab_2;
    QGridLayout *gridLayout_3;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_4;
    QHBoxLayout *horizontalLayout_8;
    QTreeWidget *treeWidget_2;
    QGroupBox *groupBox_7;
    QHBoxLayout *horizontalLayout_9;
    QTreeWidget *treeWidget_5;
    QGroupBox *groupBox_3;
    QHBoxLayout *horizontalLayout_6;
    QHBoxLayout *horizontalLayout_10;
    QGroupBox *groupBox_8;
    QSplitter *splitter;
    QGroupBox *groupBox_9;
    QHBoxLayout *horizontalLayout_7;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QVTKWidget *qvtkWidget;
    QGroupBox *groupBox_2;
    QWidget *tab;
    QGroupBox *groupBox_5;
    QHBoxLayout *horizontalLayout_2;
    QTreeWidget *treeWidget_3;
    QGroupBox *groupBox_6;
    QSplitter *splitter_4;
    QSpinBox *spinBox;
    QSpinBox *spinBox_2;
    QLabel *label_5;
    QSplitter *splitter_2;
    QLabel *label_3;
    QComboBox *comboBox;
    QLabel *label_4;
    QWidget *tabWidgetPage1;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout;
    QToolButton *toolButton_Connect;
    QToolButton *toolButton_Disconnect;
    QToolButton *toolButton_Start;
    QToolButton *toolButton_Pause;
    QToolButton *toolButton_OpenSync;
    QToolButton *toolButton_CloseSync;
    QToolButton *toolButton_SetSync;
    QToolButton *toolButton_SaveImg;
    QSpacerItem *horizontalSpacer_10;
    QSpacerItem *horizontalSpacer_9;
    QSpacerItem *horizontalSpacer_12;
    QSpacerItem *horizontalSpacer_13;
    QSpacerItem *horizontalSpacer_15;
    QSpacerItem *horizontalSpacer_14;
    QHBoxLayout *horizontalLayout_3;
    QTreeWidget *treeWidget;
    QGridLayout *Layout_ImageShow;
    QLabel *label_RightUp;
    QLabel *label_7;
    QLabel *label_LeftUp;
    QLabel *label_8;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_RightDown;
    QLabel *label_LeftDown;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menu;
    QMenu *menu_2;

    void setupUi(QMainWindow *BilletMeasurementClass)
    {
        if (BilletMeasurementClass->objectName().isEmpty())
            BilletMeasurementClass->setObjectName(QStringLiteral("BilletMeasurementClass"));
        BilletMeasurementClass->setEnabled(true);
        BilletMeasurementClass->resize(996, 723);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(BilletMeasurementClass->sizePolicy().hasHeightForWidth());
        BilletMeasurementClass->setSizePolicy(sizePolicy);
        QFont font;
        font.setKerning(true);
        BilletMeasurementClass->setFont(font);
        BilletMeasurementClass->setLayoutDirection(Qt::LeftToRight);
        action_connect = new QAction(BilletMeasurementClass);
        action_connect->setObjectName(QStringLiteral("action_connect"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/image/ico/connect.ico"), QSize(), QIcon::Normal, QIcon::Off);
        action_connect->setIcon(icon);
        action_disconnect = new QAction(BilletMeasurementClass);
        action_disconnect->setObjectName(QStringLiteral("action_disconnect"));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/image/ico/disconnect.ico"), QSize(), QIcon::Normal, QIcon::Off);
        action_disconnect->setIcon(icon1);
        action_start = new QAction(BilletMeasurementClass);
        action_start->setObjectName(QStringLiteral("action_start"));
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/image/ico/start.ico"), QSize(), QIcon::Normal, QIcon::Off);
        action_start->setIcon(icon2);
        action_pause = new QAction(BilletMeasurementClass);
        action_pause->setObjectName(QStringLiteral("action_pause"));
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/image/ico/pasue.ico"), QSize(), QIcon::Normal, QIcon::Off);
        action_pause->setIcon(icon3);
        action_setcamparam = new QAction(BilletMeasurementClass);
        action_setcamparam->setObjectName(QStringLiteral("action_setcamparam"));
        QIcon icon4;
        icon4.addFile(QStringLiteral(":/image/ico/settings.ico"), QSize(), QIcon::Normal, QIcon::Off);
        action_setcamparam->setIcon(icon4);
        action_syncing = new QAction(BilletMeasurementClass);
        action_syncing->setObjectName(QStringLiteral("action_syncing"));
        QIcon icon5;
        icon5.addFile(QStringLiteral(":/image/ico/syncing.ico"), QSize(), QIcon::Normal, QIcon::Off);
        action_syncing->setIcon(icon5);
        actionOpen = new QAction(BilletMeasurementClass);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        centralWidget = new QWidget(BilletMeasurementClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        verticalLayout_4 = new QVBoxLayout(centralWidget);
        verticalLayout_4->setSpacing(0);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setAutoFillBackground(false);
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        tab_2->setEnabled(true);
        gridLayout_3 = new QGridLayout(tab_2);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        groupBox_4 = new QGroupBox(tab_2);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        horizontalLayout_8 = new QHBoxLayout(groupBox_4);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        treeWidget_2 = new QTreeWidget(groupBox_4);
        treeWidget_2->setObjectName(QStringLiteral("treeWidget_2"));
        treeWidget_2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        treeWidget_2->header()->setVisible(false);
        treeWidget_2->header()->setDefaultSectionSize(50);
        treeWidget_2->header()->setMinimumSectionSize(70);
        treeWidget_2->header()->setStretchLastSection(true);

        horizontalLayout_8->addWidget(treeWidget_2);


        verticalLayout_2->addWidget(groupBox_4);

        groupBox_7 = new QGroupBox(tab_2);
        groupBox_7->setObjectName(QStringLiteral("groupBox_7"));
        horizontalLayout_9 = new QHBoxLayout(groupBox_7);
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        treeWidget_5 = new QTreeWidget(groupBox_7);
        treeWidget_5->setObjectName(QStringLiteral("treeWidget_5"));
        treeWidget_5->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        treeWidget_5->header()->setVisible(false);
        treeWidget_5->header()->setDefaultSectionSize(50);
        treeWidget_5->header()->setMinimumSectionSize(70);
        treeWidget_5->header()->setStretchLastSection(true);

        horizontalLayout_9->addWidget(treeWidget_5);


        verticalLayout_2->addWidget(groupBox_7);

        groupBox_3 = new QGroupBox(tab_2);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));

        verticalLayout_2->addWidget(groupBox_3);

        verticalLayout_2->setStretch(0, 3);
        verticalLayout_2->setStretch(1, 3);
        verticalLayout_2->setStretch(2, 2);

        gridLayout_3->addLayout(verticalLayout_2, 0, 0, 2, 1);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        groupBox_8 = new QGroupBox(tab_2);
        groupBox_8->setObjectName(QStringLiteral("groupBox_8"));

        horizontalLayout_10->addWidget(groupBox_8);


        horizontalLayout_6->addLayout(horizontalLayout_10);

        splitter = new QSplitter(tab_2);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        groupBox_9 = new QGroupBox(splitter);
        groupBox_9->setObjectName(QStringLiteral("groupBox_9"));
        splitter->addWidget(groupBox_9);

        horizontalLayout_6->addWidget(splitter);

        horizontalLayout_6->setStretch(0, 1);
        horizontalLayout_6->setStretch(1, 1);

        gridLayout_3->addLayout(horizontalLayout_6, 0, 1, 1, 1);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        groupBox = new QGroupBox(tab_2);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        qvtkWidget = new QVTKWidget(groupBox);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));

        verticalLayout->addWidget(qvtkWidget);


        horizontalLayout_7->addWidget(groupBox);

        groupBox_2 = new QGroupBox(tab_2);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));

        horizontalLayout_7->addWidget(groupBox_2);


        gridLayout_3->addLayout(horizontalLayout_7, 1, 1, 1, 1);

        gridLayout_3->setRowStretch(0, 2);
        gridLayout_3->setRowStretch(1, 3);
        gridLayout_3->setColumnStretch(0, 2);
        gridLayout_3->setColumnStretch(1, 11);
        tabWidget->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        tab->setInputMethodHints(Qt::ImhNone);
        groupBox_5 = new QGroupBox(tab);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setGeometry(QRect(10, 130, 211, 521));
        horizontalLayout_2 = new QHBoxLayout(groupBox_5);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        treeWidget_3 = new QTreeWidget(groupBox_5);
        new QTreeWidgetItem(treeWidget_3);
        treeWidget_3->setObjectName(QStringLiteral("treeWidget_3"));

        horizontalLayout_2->addWidget(treeWidget_3);

        groupBox_6 = new QGroupBox(tab);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        groupBox_6->setGeometry(QRect(230, 130, 741, 531));
        splitter_4 = new QSplitter(groupBox_6);
        splitter_4->setObjectName(QStringLiteral("splitter_4"));
        splitter_4->setGeometry(QRect(250, 20, 103, 20));
        splitter_4->setOrientation(Qt::Horizontal);
        spinBox = new QSpinBox(groupBox_6);
        spinBox->setObjectName(QStringLiteral("spinBox"));
        spinBox->setGeometry(QRect(190, 20, 101, 20));
        spinBox_2 = new QSpinBox(groupBox_6);
        spinBox_2->setObjectName(QStringLiteral("spinBox_2"));
        spinBox_2->setGeometry(QRect(370, 20, 91, 20));
        label_5 = new QLabel(groupBox_6);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(309, 20, 51, 20));
        splitter_2 = new QSplitter(groupBox_6);
        splitter_2->setObjectName(QStringLiteral("splitter_2"));
        splitter_2->setGeometry(QRect(20, 20, 103, 20));
        splitter_2->setOrientation(Qt::Horizontal);
        label_3 = new QLabel(splitter_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        splitter_2->addWidget(label_3);
        comboBox = new QComboBox(splitter_2);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        splitter_2->addWidget(comboBox);
        label_4 = new QLabel(groupBox_6);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(140, 20, 51, 20));
        tabWidget->addTab(tab, QString());
        tabWidgetPage1 = new QWidget();
        tabWidgetPage1->setObjectName(QStringLiteral("tabWidgetPage1"));
        verticalLayout_5 = new QVBoxLayout(tabWidgetPage1);
        verticalLayout_5->setSpacing(1);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(1);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        toolButton_Connect = new QToolButton(tabWidgetPage1);
        toolButton_Connect->setObjectName(QStringLiteral("toolButton_Connect"));
        toolButton_Connect->setMinimumSize(QSize(80, 0));
        toolButton_Connect->setMaximumSize(QSize(80, 16777215));
        toolButton_Connect->setFocusPolicy(Qt::TabFocus);
        toolButton_Connect->setIcon(icon);
        toolButton_Connect->setIconSize(QSize(24, 24));
        toolButton_Connect->setPopupMode(QToolButton::DelayedPopup);
        toolButton_Connect->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_Connect->setAutoRaise(true);
        toolButton_Connect->setArrowType(Qt::NoArrow);

        horizontalLayout->addWidget(toolButton_Connect);

        toolButton_Disconnect = new QToolButton(tabWidgetPage1);
        toolButton_Disconnect->setObjectName(QStringLiteral("toolButton_Disconnect"));
        toolButton_Disconnect->setMinimumSize(QSize(80, 0));
        toolButton_Disconnect->setMaximumSize(QSize(80, 16777215));
        toolButton_Disconnect->setIcon(icon1);
        toolButton_Disconnect->setIconSize(QSize(24, 24));
        toolButton_Disconnect->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_Disconnect->setAutoRaise(true);

        horizontalLayout->addWidget(toolButton_Disconnect);

        toolButton_Start = new QToolButton(tabWidgetPage1);
        toolButton_Start->setObjectName(QStringLiteral("toolButton_Start"));
        toolButton_Start->setMinimumSize(QSize(80, 0));
        toolButton_Start->setMaximumSize(QSize(80, 16777215));
        toolButton_Start->setCursor(QCursor(Qt::ArrowCursor));
        toolButton_Start->setMouseTracking(false);
        toolButton_Start->setAcceptDrops(true);
        toolButton_Start->setIcon(icon2);
        toolButton_Start->setIconSize(QSize(24, 24));
        toolButton_Start->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_Start->setAutoRaise(true);

        horizontalLayout->addWidget(toolButton_Start);

        toolButton_Pause = new QToolButton(tabWidgetPage1);
        toolButton_Pause->setObjectName(QStringLiteral("toolButton_Pause"));
        toolButton_Pause->setMinimumSize(QSize(80, 0));
        toolButton_Pause->setMaximumSize(QSize(80, 16777215));
        toolButton_Pause->setIcon(icon3);
        toolButton_Pause->setIconSize(QSize(24, 24));
        toolButton_Pause->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_Pause->setAutoRaise(true);

        horizontalLayout->addWidget(toolButton_Pause);

        toolButton_OpenSync = new QToolButton(tabWidgetPage1);
        toolButton_OpenSync->setObjectName(QStringLiteral("toolButton_OpenSync"));
        toolButton_OpenSync->setEnabled(true);
        toolButton_OpenSync->setMinimumSize(QSize(80, 0));
        toolButton_OpenSync->setMaximumSize(QSize(80, 16777215));
        toolButton_OpenSync->setAcceptDrops(false);
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/image/ico/\345\274\200\345\205\263 \345\274\200(1).png"), QSize(), QIcon::Normal, QIcon::Off);
        toolButton_OpenSync->setIcon(icon6);
        toolButton_OpenSync->setIconSize(QSize(24, 24));
        toolButton_OpenSync->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_OpenSync->setAutoRaise(true);

        horizontalLayout->addWidget(toolButton_OpenSync);

        toolButton_CloseSync = new QToolButton(tabWidgetPage1);
        toolButton_CloseSync->setObjectName(QStringLiteral("toolButton_CloseSync"));
        toolButton_CloseSync->setMinimumSize(QSize(80, 0));
        toolButton_CloseSync->setMaximumSize(QSize(80, 16777215));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/image/ico/\345\274\200\345\205\263 \345\205\263(1).png"), QSize(), QIcon::Normal, QIcon::Off);
        toolButton_CloseSync->setIcon(icon7);
        toolButton_CloseSync->setIconSize(QSize(24, 24));
        toolButton_CloseSync->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_CloseSync->setAutoRaise(true);

        horizontalLayout->addWidget(toolButton_CloseSync);

        toolButton_SetSync = new QToolButton(tabWidgetPage1);
        toolButton_SetSync->setObjectName(QStringLiteral("toolButton_SetSync"));
        toolButton_SetSync->setMinimumSize(QSize(80, 0));
        toolButton_SetSync->setMaximumSize(QSize(80, 16777215));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/image/ico/\350\256\276\347\275\256\345\217\202\346\225\260.png"), QSize(), QIcon::Normal, QIcon::Off);
        toolButton_SetSync->setIcon(icon8);
        toolButton_SetSync->setIconSize(QSize(24, 24));
        toolButton_SetSync->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_SetSync->setAutoRaise(true);
        toolButton_SetSync->setArrowType(Qt::NoArrow);

        horizontalLayout->addWidget(toolButton_SetSync);

        toolButton_SaveImg = new QToolButton(tabWidgetPage1);
        toolButton_SaveImg->setObjectName(QStringLiteral("toolButton_SaveImg"));
        toolButton_SaveImg->setMinimumSize(QSize(80, 0));
        toolButton_SaveImg->setMaximumSize(QSize(80, 16777215));
        QIcon icon9;
        icon9.addFile(QStringLiteral(":/image/ico/save.ico"), QSize(), QIcon::Normal, QIcon::Off);
        toolButton_SaveImg->setIcon(icon9);
        toolButton_SaveImg->setIconSize(QSize(24, 24));
        toolButton_SaveImg->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_SaveImg->setAutoRaise(true);
        toolButton_SaveImg->setArrowType(Qt::NoArrow);

        horizontalLayout->addWidget(toolButton_SaveImg);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_10);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_9);

        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_12);

        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_13);

        horizontalSpacer_15 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_15);

        horizontalSpacer_14 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_14);


        verticalLayout_5->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        treeWidget = new QTreeWidget(tabWidgetPage1);
        treeWidget->setObjectName(QStringLiteral("treeWidget"));
        QFont font1;
        font1.setFamily(QStringLiteral("ADMUI3Lg"));
        font1.setPointSize(10);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(50);
        treeWidget->setFont(font1);
        treeWidget->setStyleSheet(QStringLiteral("font: 10pt \"ADMUI3Lg\";"));
        treeWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        treeWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        treeWidget->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        treeWidget->setEditTriggers(QAbstractItemView::DoubleClicked);
        treeWidget->setUniformRowHeights(true);
        treeWidget->header()->setCascadingSectionResizes(false);

        horizontalLayout_3->addWidget(treeWidget);


        verticalLayout_5->addLayout(horizontalLayout_3);

        Layout_ImageShow = new QGridLayout();
        Layout_ImageShow->setSpacing(6);
        Layout_ImageShow->setObjectName(QStringLiteral("Layout_ImageShow"));
        label_RightUp = new QLabel(tabWidgetPage1);
        label_RightUp->setObjectName(QStringLiteral("label_RightUp"));
        label_RightUp->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));

        Layout_ImageShow->addWidget(label_RightUp, 1, 1, 1, 1);

        label_7 = new QLabel(tabWidgetPage1);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setAlignment(Qt::AlignBottom|Qt::AlignLeading|Qt::AlignLeft);

        Layout_ImageShow->addWidget(label_7, 2, 0, 1, 1);

        label_LeftUp = new QLabel(tabWidgetPage1);
        label_LeftUp->setObjectName(QStringLiteral("label_LeftUp"));
        label_LeftUp->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        label_LeftUp->setTextFormat(Qt::AutoText);

        Layout_ImageShow->addWidget(label_LeftUp, 1, 0, 1, 1);

        label_8 = new QLabel(tabWidgetPage1);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setAlignment(Qt::AlignBottom|Qt::AlignLeading|Qt::AlignLeft);

        Layout_ImageShow->addWidget(label_8, 2, 1, 1, 1);

        label = new QLabel(tabWidgetPage1);
        label->setObjectName(QStringLiteral("label"));
        label->setAlignment(Qt::AlignBottom|Qt::AlignLeading|Qt::AlignLeft);

        Layout_ImageShow->addWidget(label, 0, 0, 1, 1);

        label_2 = new QLabel(tabWidgetPage1);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setAlignment(Qt::AlignBottom|Qt::AlignLeading|Qt::AlignLeft);

        Layout_ImageShow->addWidget(label_2, 0, 1, 1, 1);

        label_RightDown = new QLabel(tabWidgetPage1);
        label_RightDown->setObjectName(QStringLiteral("label_RightDown"));
        label_RightDown->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        label_RightDown->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        Layout_ImageShow->addWidget(label_RightDown, 3, 1, 1, 1);

        label_LeftDown = new QLabel(tabWidgetPage1);
        label_LeftDown->setObjectName(QStringLiteral("label_LeftDown"));
        label_LeftDown->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));
        label_LeftDown->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

        Layout_ImageShow->addWidget(label_LeftDown, 3, 0, 1, 1);

        Layout_ImageShow->setRowStretch(0, 1);
        Layout_ImageShow->setRowStretch(1, 20);
        Layout_ImageShow->setRowStretch(2, 1);
        Layout_ImageShow->setRowStretch(3, 20);

        verticalLayout_5->addLayout(Layout_ImageShow);

        verticalLayout_5->setStretch(0, 1);
        verticalLayout_5->setStretch(1, 2);
        verticalLayout_5->setStretch(2, 50);
        tabWidget->addTab(tabWidgetPage1, QString());

        verticalLayout_4->addWidget(tabWidget);

        BilletMeasurementClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(BilletMeasurementClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 996, 23));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        menu_2 = new QMenu(menuBar);
        menu_2->setObjectName(QStringLiteral("menu_2"));
        BilletMeasurementClass->setMenuBar(menuBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menu->menuAction());
        menuBar->addAction(menu_2->menuAction());
        menuFile->addAction(actionOpen);

        retranslateUi(BilletMeasurementClass);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(BilletMeasurementClass);
    } // setupUi

    void retranslateUi(QMainWindow *BilletMeasurementClass)
    {
        BilletMeasurementClass->setWindowTitle(QApplication::translate("BilletMeasurementClass", "\345\234\250\347\272\277\350\275\256\345\273\223\346\265\213\351\207\217\347\263\273\347\273\237", 0));
        action_connect->setText(QApplication::translate("BilletMeasurementClass", "\350\277\236\346\216\245\347\233\270\346\234\272", 0));
        action_disconnect->setText(QApplication::translate("BilletMeasurementClass", "\346\226\255\345\274\200\347\233\270\346\234\272", 0));
        action_start->setText(QApplication::translate("BilletMeasurementClass", "\345\274\200\345\247\213\351\207\207\351\233\206", 0));
        action_pause->setText(QApplication::translate("BilletMeasurementClass", "\345\201\234\346\255\242\351\207\207\351\233\206", 0));
        action_setcamparam->setText(QApplication::translate("BilletMeasurementClass", "\350\256\276\347\275\256\347\233\270\346\234\272\345\217\202\346\225\260", 0));
        action_syncing->setText(QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272\345\220\214\346\255\245\351\207\207\351\233\206", 0));
        actionOpen->setText(QApplication::translate("BilletMeasurementClass", "Open", 0));
#ifndef QT_NO_ACCESSIBILITY
        tabWidget->setAccessibleDescription(QString());
#endif // QT_NO_ACCESSIBILITY
        groupBox_4->setTitle(QApplication::translate("BilletMeasurementClass", "\351\222\242\345\235\257ID", 0));
        QTreeWidgetItem *___qtreewidgetitem = treeWidget_2->headerItem();
        ___qtreewidgetitem->setText(1, QApplication::translate("BilletMeasurementClass", "\350\277\233\345\205\245\346\227\266\351\227\264", 0));
        ___qtreewidgetitem->setText(0, QApplication::translate("BilletMeasurementClass", "\351\222\242\350\275\250ID", 0));
        groupBox_7->setTitle(QApplication::translate("BilletMeasurementClass", "\345\275\223\345\211\215\351\222\242\345\235\257\344\277\241\346\201\257", 0));
        QTreeWidgetItem *___qtreewidgetitem1 = treeWidget_5->headerItem();
        ___qtreewidgetitem1->setText(1, QApplication::translate("BilletMeasurementClass", "\344\277\241\346\201\257", 0));
        ___qtreewidgetitem1->setText(0, QApplication::translate("BilletMeasurementClass", "\345\220\215\347\247\260", 0));
        groupBox_3->setTitle(QApplication::translate("BilletMeasurementClass", "\347\263\273\347\273\237\344\277\241\346\201\257", 0));
        groupBox_8->setTitle(QApplication::translate("BilletMeasurementClass", "\344\274\240\346\204\237\345\231\250\347\212\266\346\200\201", 0));
        groupBox_9->setTitle(QApplication::translate("BilletMeasurementClass", "\346\225\260\346\215\256\350\257\273\345\217\226\343\200\201\345\255\230\345\202\250", 0));
        groupBox->setTitle(QApplication::translate("BilletMeasurementClass", "\351\222\242\345\235\257\350\275\256\345\273\223\346\230\276\347\244\272", 0));
        groupBox_2->setTitle(QApplication::translate("BilletMeasurementClass", "\351\222\242\345\235\257\350\275\256\345\273\223\346\265\213\351\207\217\344\277\241\346\201\257", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("BilletMeasurementClass", "\350\275\256\345\273\223\346\265\213\351\207\217", 0));
        groupBox_5->setTitle(QApplication::translate("BilletMeasurementClass", "\345\216\206\345\217\262\350\256\260\345\275\225\346\237\245\350\257\242", 0));
        QTreeWidgetItem *___qtreewidgetitem2 = treeWidget_3->headerItem();
        ___qtreewidgetitem2->setText(0, QApplication::translate("BilletMeasurementClass", "\345\216\206\345\217\262\351\222\242\345\235\257\344\277\241\346\201\257", 0));

        const bool __sortingEnabled = treeWidget_3->isSortingEnabled();
        treeWidget_3->setSortingEnabled(false);
        QTreeWidgetItem *___qtreewidgetitem3 = treeWidget_3->topLevelItem(0);
        ___qtreewidgetitem3->setText(0, QApplication::translate("BilletMeasurementClass", "\351\222\242\345\235\257\345\234\250\345\275\225\346\227\245\346\234\237", 0));
        treeWidget_3->setSortingEnabled(__sortingEnabled);

        groupBox_6->setTitle(QApplication::translate("BilletMeasurementClass", "\346\225\260\346\215\256\350\247\206\345\233\276", 0));
        label_5->setText(QApplication::translate("BilletMeasurementClass", "\347\273\223\346\235\237\346\227\266\351\227\264", 0));
        label_3->setText(QApplication::translate("BilletMeasurementClass", "\351\222\242\345\235\257\347\261\273\345\236\213", 0));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("BilletMeasurementClass", "\346\226\271\345\235\257", 0)
         << QApplication::translate("BilletMeasurementClass", "\346\235\277\345\235\257", 0)
        );
        label_4->setText(QApplication::translate("BilletMeasurementClass", "\350\265\267\345\247\213\346\227\266\351\227\264", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("BilletMeasurementClass", "\346\237\245\347\234\213\345\216\206\345\217\262", 0));
        toolButton_Connect->setText(QApplication::translate("BilletMeasurementClass", "\350\277\236\346\216\245\347\233\270\346\234\272", 0));
        toolButton_Disconnect->setText(QApplication::translate("BilletMeasurementClass", "\346\226\255\345\274\200\347\233\270\346\234\272", 0));
        toolButton_Start->setText(QApplication::translate("BilletMeasurementClass", "\345\274\200\345\247\213\351\207\207\351\233\206", 0));
        toolButton_Pause->setText(QApplication::translate("BilletMeasurementClass", "\345\201\234\346\255\242\351\207\207\351\233\206", 0));
        toolButton_OpenSync->setText(QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272\345\220\214\346\255\245\351\207\207\351\233\206", 0));
        toolButton_CloseSync->setText(QApplication::translate("BilletMeasurementClass", "\345\205\263\351\227\255\345\220\214\346\255\245\351\207\207\351\233\206", 0));
        toolButton_SetSync->setText(QApplication::translate("BilletMeasurementClass", "\350\256\276\347\275\256\350\247\246\345\217\221\345\217\202\346\225\260", 0));
        toolButton_SaveImg->setText(QApplication::translate("BilletMeasurementClass", "\344\277\235\345\255\230\345\233\276\345\203\217", 0));
        QTreeWidgetItem *___qtreewidgetitem4 = treeWidget->headerItem();
        ___qtreewidgetitem4->setText(8, QApplication::translate("BilletMeasurementClass", "\345\233\276\345\203\217\351\253\230\345\272\246", 0));
        ___qtreewidgetitem4->setText(7, QApplication::translate("BilletMeasurementClass", "\345\233\276\345\203\217\345\256\275\345\272\246", 0));
        ___qtreewidgetitem4->setText(6, QApplication::translate("BilletMeasurementClass", "\346\233\235\345\205\211\346\227\266\351\227\264", 0));
        ___qtreewidgetitem4->setText(5, QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272\345\270\247\351\242\221", 0));
        ___qtreewidgetitem4->setText(4, QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272\347\212\266\346\200\201", 0));
        ___qtreewidgetitem4->setText(3, QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272IP", 0));
        ___qtreewidgetitem4->setText(2, QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272MAC", 0));
        ___qtreewidgetitem4->setText(1, QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272ID", 0));
        ___qtreewidgetitem4->setText(0, QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272\345\236\213\345\217\267", 0));
        label_RightUp->setText(QString());
        label_7->setText(QApplication::translate("BilletMeasurementClass", "Cam3", 0));
        label_LeftUp->setText(QString());
        label_8->setText(QApplication::translate("BilletMeasurementClass", "Cam4", 0));
        label->setText(QApplication::translate("BilletMeasurementClass", "Cam1", 0));
        label_2->setText(QApplication::translate("BilletMeasurementClass", "Cam2", 0));
        label_RightDown->setText(QString());
        label_LeftDown->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(tabWidgetPage1), QApplication::translate("BilletMeasurementClass", "\347\233\270\346\234\272\351\207\207\351\233\206", 0));
        menuFile->setTitle(QApplication::translate("BilletMeasurementClass", "\346\226\207\344\273\266", 0));
        menu->setTitle(QApplication::translate("BilletMeasurementClass", "\347\274\226\350\276\221", 0));
        menu_2->setTitle(QApplication::translate("BilletMeasurementClass", "\345\270\256\345\212\251", 0));
    } // retranslateUi

};

namespace Ui {
    class BilletMeasurementClass: public Ui_BilletMeasurementClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BILLETMEASUREMENT_H
