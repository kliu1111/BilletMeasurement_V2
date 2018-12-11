#pragma once
#include <QDateTime>
#include <QObject>
#include <QMetaType>
#include <QString>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QString>
#include <QTreeWidget>
#include <QCoreApplication>
#include <pylon/PylonIncludes.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/gige/BaslerGigEInstantCameraArray.h>
#include <thread>
#include <vector>
#include <opencv2\opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;
using namespace Pylon;

struct CameraPara
{
	unsigned int LeftExposureTime;
	unsigned int LeftImageHeight;
	unsigned int LeftImageWidth;
	unsigned int LeftImageFrequency;

	unsigned int RightExposureTime;
	unsigned int RightImageHeight;
	unsigned int RightImageWidth;
	unsigned int RightImageFrequency;
};

//Q_DECLARE_OPAQUE_POINTER(CamPara);

class BaslerCamera 
{
public:

public:
	BaslerCamera();
	~BaslerCamera();
	QImage Mat2Image(cv::Mat);
	//相机数组
	CBaslerGigEInstantCameraArray CamList;
	QImage LeftImg;
	QImage RightImg;
	const QPixmap *Lpixmap;
	const QPixmap *Rpixmap;
	QString str_time;
	QDateTime time;
	uint32_t rows = 1236;
	uint32_t cols = 1626;

	//连接检测到的所有的相机
	void Connect();

	//检查所有的相机是否已连接
	bool IsConnected();

	//卸载所有的设备防止非正常关闭时报错
	void DetachDevice();

	//断开所有的相机，如需重新采集，需先连接相机。
	void Disconnect();

	//连接相机后，进行采集工作。
	void StartGrabbing();

	//检查所有的相机是否都处于采集状态
	bool IsGrabbing();

	//停止采集，如需重新采集，直接StartGrabbing。
	void StopGrabbing();

	//取得相机的初始参数
	void GetCamInitPara();

	//保存图像
	void SaveImg();

	//设置相机参数
	void SetCamPara(CameraPara CamPara);
	void SetIniCamParam(CameraPara CamPara);

	//采集线程
	static void GrabThread(BaslerCamera* pClass);
	void GrabThreadFunction();

	//主窗口中label(图像显示)控件的地址
	QLabel* LeftUpLabelAdd;
	QLabel* RightUpLabelAdd;
	QLabel* LeftDownLabelAdd;
	QLabel* RightDownLabelAdd;
	void GetLeftUpLabelAddress(QLabel &LeftUpLabelPointer);
	void GetRightUpLabelAddress(QLabel &RightUpLabelPointer);
	void GetLeftDownLabelAddress(QLabel &LeftDownLabelPointer);
	void GetRightDownLabelAddress(QLabel &RightDownLabelPointer);
	//treewidget地址
	QTreeWidget* CamInfo;
	void GetCamInfoListAddress(QTreeWidget &CamInfoListPointer);
	//vector<QTreeWidgetItem*> CamInfoList;


private:
	CGrabResultPtr LeftUpptrGrabResult;
	CGrabResultPtr RightUpptrGrabResult;
	CGrabResultPtr LeftDownptrGrabResult;
	CGrabResultPtr RightDownptrGrabResult;
	//vector<QTreeWidgetItem*> CamInfoList;
	QTreeWidgetItem* CamInfoList[4];
	QTreeWidgetItem *Cam1Info;
	QTreeWidgetItem *Cam2Info;
	QTreeWidgetItem *Cam3Info;
	QTreeWidgetItem *Cam4Info;

};

