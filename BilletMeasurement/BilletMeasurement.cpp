#include "BilletMeasurement.h"
#include <windows.h>
#include <iostream>
#include <BaslerCamera.h>
#include "tinystr.h"
#include "tinyxml.h"
#include <QFileDialog>
using namespace std;

BilletMeasurement::BilletMeasurement(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//setWindowState(Qt::WindowMaximized);
	//setWindowState(Qt::WindowMinimized);
	//AllocConsole();
	//freopen("CONOUT$", "w+t", stdout);
	LoadIniCamParam();

	cam.GetLeftUpLabelAddress(*ui.label_LeftUp);
	cam.GetRightUpLabelAddress(*ui.label_RightUp);
	//取得主窗口的Qtreewidget地址
	cam.GetCamInfoListAddress(*ui.treeWidget);
	//cam.GetLeftDownLabelAddress(*ui.label_LeftDown);
	//cam.GetRightDownLabelAddress(*ui.label_RightDown);
	cam.Connect();
	cam.SetIniCamParam(CamPara);
	//相机信息显示在Qtreewidget中
	cam.GetCamInitPara();
	cam.StartGrabbing();
	WidgetItem = NULL;


	SetTriggerWindow = new SetTriggerParam(this);
	SetTriggerWindow->setWindowFlags(Qt::Window);
	InitSlot();

	// Set up the QVTK window
	initialVtkWidget();

	
}

BilletMeasurement::~BilletMeasurement()
{
	cam.DetachDevice();
	PylonTerminate();
}

void BilletMeasurement::onOpen()
{
	//只能打开PCD文件
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PCD files(*.pcd)"));

	if (!fileName.isEmpty())
	{
		std::string file_name = fileName.toStdString();
		//sensor_msgs::PointCloud2 cloud2;
		pcl::PCLPointCloud2 cloud2;
		//pcl::PointCloud<Eigen::MatrixXf> cloud2;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int pcd_version;
		int data_type;
		unsigned int data_idx;
		int offset = 0;
		pcl::PCDReader rd;
		rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);

		if (data_type == 0)
		{
			pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
		}
		else if (data_type == 2)
		{
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud);
		}

		viewer->updatePointCloud(cloud, "cloud");
		viewer->resetCamera();
		ui.qvtkWidget->update();
	}
}

void BilletMeasurement::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud, "cloud");

	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();
}

void BilletMeasurement::SlotCamConnect()
{
	cam.Connect();
	if (cam.IsConnected())
	{
		int n = ui.treeWidget->topLevelItemCount();
		if (n)
		{
			for (int i = 0; i < n; i++)
			{
				ui.treeWidget->topLevelItem(i)->setText(4, QStringLiteral("已连接"));
			}
		} 
		else
		{
			//相机信息显示在Qtreewidget中
			cam.GetCamInitPara();
		}

	}
}

void BilletMeasurement::SlotCamDisconnect()
{
	cam.Disconnect();
	//当前treeWidget中的所有可见行数
	int n = ui.treeWidget->topLevelItemCount();
	for (int i = 0; i < n;i++)
	{
		ui.treeWidget->topLevelItem(i)->setText(4, QStringLiteral("已断开"));
	}
}

void BilletMeasurement::SlotStartGrab()
{
	cam.StartGrabbing();
}

void BilletMeasurement::SlotPauseGrab()
{
	cam.StopGrabbing();
}

void BilletMeasurement::TreeWidgetOpenEditor(QTreeWidgetItem *item, int col)
{
	ui.treeWidget->openPersistentEditor(item, col);
	WidgetItem = item;
	ItemCol = col;
}

void BilletMeasurement::TreeWidgetCloseEditor()
{
	if (WidgetItem != NULL)
	{
		//CameraPara.LeftUpCamIp = ui.treeWidget->topLevelItem(0)->text(3);
		ui.treeWidget->closePersistentEditor(WidgetItem, ItemCol);
	}
}

void BilletMeasurement::GetUserData()
{
	if (cam.IsGrabbing())
	{
		QMessageBox::about(NULL, QStringLiteral("提示"), QStringLiteral("请关闭相机采集后在进行相机参数的设置！！！"));
		return;
	}
	if (WidgetItem != NULL)
	{
		CamPara.LeftImageFrequency = ui.treeWidget->topLevelItem(0)->text(5).toInt();
		CamPara.LeftExposureTime = ui.treeWidget->topLevelItem(0)->text(6).toInt();
		CamPara.LeftImageWidth = ui.treeWidget->topLevelItem(0)->text(7).toInt();
		CamPara.LeftImageHeight = ui.treeWidget->topLevelItem(0)->text(8).toInt();

		CamPara.RightImageFrequency = ui.treeWidget->topLevelItem(1)->text(5).toInt();
		CamPara.RightExposureTime = ui.treeWidget->topLevelItem(1)->text(6).toInt();
		CamPara.RightImageWidth = ui.treeWidget->topLevelItem(1)->text(7).toInt();
		CamPara.RightImageHeight = ui.treeWidget->topLevelItem(1)->text(8).toInt();
		//emit aaa();
		cam.SetCamPara(CamPara);
	}

}

void BilletMeasurement::SlotOpenSync()
{
	if (myCom == nullptr)
	{
		myCom = new QextSerialPort("COM5", QextSerialPort::Polling);
		IsOpen = myCom->open(QIODevice::ReadWrite);
	}
	if (IsOpen)
	{
		myCom->write("EnableOutput 1\n");

	}
}

void BilletMeasurement::SlotCloseSync()
{
	if (myCom == nullptr)
	{
		myCom = new QextSerialPort("COM5", QextSerialPort::Polling);
		IsOpen = myCom->open(QIODevice::ReadWrite);
	}
	if (IsOpen)
	{
		myCom->write("EnableOutput 0\n");
	}
}

//void BilletMeasurement::SetUserInputPara()
//{
//	cam.SetCamPara(para);
//}

void BilletMeasurement::SlotSetExTriggerParam()
{
	SetTriggerWindow->show();
}

void BilletMeasurement::SlotTriggerBtnOk()
{
	if (myCom == nullptr)
	{
		myCom = new QextSerialPort("COM4", QextSerialPort::Polling);
		IsOpen = myCom->open(QIODevice::ReadWrite);
	}
	if (IsOpen)
	{
		int TextData = (SetTriggerWindow->ui.lineEdit->text()).toInt();
		int InputData = TextData * 4;
		QString CmdData = "Line " + QString::number(InputData) + "\n";
		char CharCmdData[9];
		strcpy(CharCmdData, CmdData.toStdString().c_str());
		myCom->write(CharCmdData);
	}
	SetTriggerWindow->close();

}

void BilletMeasurement::SlotTriggerBtnCancel()
{
	SetTriggerWindow->close();
}

void BilletMeasurement::LoadIniCamParam()
{
	TiXmlDocument *inXml = new TiXmlDocument();
	if (!inXml->LoadFile("IniCamPara.xml"))
	{
		cerr << inXml->ErrorDesc() << endl;
	}

	//定义根节点，记录xml文件的起始节点
	TiXmlElement *inRoot = inXml->FirstChildElement(); //root指向xml文档的第一个节点

	if (NULL == inRoot) //判断文件是否有内容
	{
		cerr << "No root element ！！！" << endl;
		inXml->Clear();
	}

	for (TiXmlElement *inElem = inRoot->FirstChildElement(); inElem != NULL; inElem = inElem->NextSiblingElement())
	{
		string str = inElem->Value();
		//两个string进行比较时，使用compare，若相等返回0
		if (!str.compare("Cam1"))
		{
			TiXmlElement  *Frequency = inElem->FirstChildElement();
			CamPara.LeftImageFrequency = atof(Frequency->FirstChild()->Value());

			TiXmlElement  *ExposureTime = Frequency->NextSiblingElement();
			CamPara.LeftExposureTime = atof(ExposureTime->FirstChild()->Value());

			TiXmlElement  *ImageWidth = ExposureTime->NextSiblingElement();
			CamPara.LeftImageWidth = atof(ImageWidth->FirstChild()->Value());
					
			TiXmlElement  *ImageHeight = ImageWidth->NextSiblingElement();
			CamPara.LeftImageHeight = atof(ImageHeight->FirstChild()->Value());
		}

		if (!str.compare("Cam2"))
		{
			TiXmlElement  *Frequency = inElem->FirstChildElement();
			CamPara.RightImageFrequency = atof(Frequency->FirstChild()->Value());

			TiXmlElement  *ExposureTime = Frequency->NextSiblingElement();
			CamPara.RightExposureTime = atof(ExposureTime->FirstChild()->Value());

			TiXmlElement  *ImageWidth = ExposureTime->NextSiblingElement();
			CamPara.RightImageWidth = atof(ImageWidth->FirstChild()->Value());

			TiXmlElement  *ImageHeight = ImageWidth->NextSiblingElement();
			CamPara.RightImageHeight = atof(ImageHeight->FirstChild()->Value());
		}
	}
}

void BilletMeasurement::LoadCalibParam()
{
	for (int i = 0; i < 4; i++)
	{
		TiXmlDocument *inXml = new TiXmlDocument();
		if (!inXml->LoadFile("./CalibParams/CamParam_[i].xml"))
		{
			cerr << inXml->ErrorDesc() << endl;
		}

		//定义根节点，记录xml文件的起始节点
		TiXmlElement *inRoot = inXml->FirstChildElement(); //root指向xml文档的第一个节点

		if (NULL == inRoot) //判断文件是否有内容
		{
			cerr << "No root element ！！！" << endl;
			inXml->Clear();
		}

		for (TiXmlElement *inElem = inRoot->FirstChildElement(); inElem != NULL; inElem = inElem->NextSiblingElement())
		{
			string str = inElem->Value();
			//两个string进行比较时，使用compare，若相等返回0
			if (!str.compare("Cam1"))
			{
				TiXmlElement  *Frequency = inElem->FirstChildElement();
				CamPara.LeftImageFrequency = atof(Frequency->FirstChild()->Value());

				TiXmlElement  *ExposureTime = Frequency->NextSiblingElement();
				CamPara.LeftExposureTime = atof(ExposureTime->FirstChild()->Value());

				TiXmlElement  *ImageWidth = ExposureTime->NextSiblingElement();
				CamPara.LeftImageWidth = atof(ImageWidth->FirstChild()->Value());

				TiXmlElement  *ImageHeight = ImageWidth->NextSiblingElement();
				CamPara.LeftImageHeight = atof(ImageHeight->FirstChild()->Value());
			}
		}
	}
}

void BilletMeasurement::InitSlot()
{
	connect(ui.toolButton_Connect, SIGNAL(clicked()), this, SLOT(SlotCamConnect()));
	connect(ui.toolButton_Disconnect, SIGNAL(clicked()), this, SLOT(SlotCamDisconnect()));
	connect(ui.toolButton_Start, SIGNAL(clicked()), this, SLOT(SlotStartGrab()));
	connect(ui.toolButton_Pause, SIGNAL(clicked()), this, SLOT(SlotPauseGrab()));
	//connect(ui.toolButton_SetCamPara, SIGNAL(clicked()), this, SLOT(SlotSetCamPara()));
	connect(ui.toolButton_OpenSync, SIGNAL(clicked()), this, SLOT(SlotOpenSync()));
	connect(ui.toolButton_CloseSync, SIGNAL(clicked()), this, SLOT(SlotCloseSync()));
	connect(ui.toolButton_SetSync, SIGNAL(clicked()), this, SLOT(SlotSetSync()));
	//双击可编辑的状态
	connect(ui.treeWidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(TreeWidgetOpenEditor(QTreeWidgetItem*, int)));
	connect(ui.treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(TreeWidgetCloseEditor()));
	connect(ui.treeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(GetUserData()));
	//同步板同步
	connect(ui.toolButton_SetSync, SIGNAL(clicked()), this, SLOT(SlotSetExTriggerParam()));
	connect(SetTriggerWindow->ui.Btn_Ok, SIGNAL(clicked()), this, SLOT(SlotTriggerBtnOk()));
	connect(SetTriggerWindow->ui.Btn_Cancel, SIGNAL(clicked()), this, SLOT(SlotTriggerBtnCancel()));
	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	//connect(this, SIGNAL(aaa()), this, SLOT(SetUserInputPara()));
}