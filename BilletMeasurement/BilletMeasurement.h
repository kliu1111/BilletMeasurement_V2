#ifndef BILLETMEASUREMENT_H
#define BILLETMEASUREMENT_H

#include <QtWidgets/QMainWindow>
#include "ui_BilletMeasurement.h"
#include "CalibParam.h"
#include "BaslerCamera.h"
#include "qextserialport/qextserialport.h"
#include "SetTriggerParam.h"

// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
class BilletMeasurement : public QMainWindow
{
	Q_OBJECT

public:
	BilletMeasurement(QWidget *parent = 0);
	~BilletMeasurement();

public:
	void LoadIniCamParam();
	//加载所有的标定数据
	void LoadCalibParam();
	//初始化vtk部件
	void initialVtkWidget();
private:
	Ui::BilletMeasurementClass ui;
	BaslerCamera cam;
	CameraPara CamPara;

	//BaslerCamera::CamPara CameraPara;
	QTreeWidgetItem *WidgetItem;
	int ItemCol;
	SetTriggerParam* SetTriggerWindow;
	QextSerialPort *myCom = nullptr;
	bool IsOpen;
	//相机内参
	CamInnerPara CamInner[4];
	//CamInnerPara CamInner2;
	//CamInnerPara CamInner3;
	//CamInnerPara CamInner4;
	//相机外参
	CamPosePara CamPose[4];
	//CamPosePara CamPose2;
	//CamPosePara CamPose3;
	//CamPosePara CamPose4;
	//激光平面
	LaserPosePara LaserPose[2];
	//LaserPosePara LaserPose2;
protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PointCloudT::Ptr cloud;

public slots :
     void onOpen();
	//连接相机
	void SlotCamConnect();
	//断开相机
	void SlotCamDisconnect();
	//开始相机采集
	void SlotStartGrab();
	//停止相机采集
	void SlotPauseGrab();
	//控制帧同步板进行相机同步采集
	void SlotOpenSync();
	//控制帧同步板关闭相机采集
	void SlotCloseSync();
	//双击控件时开启编辑状态
	void TreeWidgetOpenEditor(QTreeWidgetItem *item, int col);
	//控件变化时关闭编辑状态
	void TreeWidgetCloseEditor();

	//void SetUserInputPara();
	void SlotSetExTriggerParam();
	void SlotTriggerBtnOk();
	void SlotTriggerBtnCancel();

	void GetUserData();

	void InitSlot();

signals:
	void aaa();

};

#endif // BILLETMEASUREMENT_H
