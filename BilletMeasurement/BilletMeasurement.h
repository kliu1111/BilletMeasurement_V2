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
	//�������еı궨����
	void LoadCalibParam();
	//��ʼ��vtk����
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
	//����ڲ�
	CamInnerPara CamInner[4];
	//CamInnerPara CamInner2;
	//CamInnerPara CamInner3;
	//CamInnerPara CamInner4;
	//������
	CamPosePara CamPose[4];
	//CamPosePara CamPose2;
	//CamPosePara CamPose3;
	//CamPosePara CamPose4;
	//����ƽ��
	LaserPosePara LaserPose[2];
	//LaserPosePara LaserPose2;
protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PointCloudT::Ptr cloud;

public slots :
     void onOpen();
	//�������
	void SlotCamConnect();
	//�Ͽ����
	void SlotCamDisconnect();
	//��ʼ����ɼ�
	void SlotStartGrab();
	//ֹͣ����ɼ�
	void SlotPauseGrab();
	//����֡ͬ����������ͬ���ɼ�
	void SlotOpenSync();
	//����֡ͬ����ر�����ɼ�
	void SlotCloseSync();
	//˫���ؼ�ʱ�����༭״̬
	void TreeWidgetOpenEditor(QTreeWidgetItem *item, int col);
	//�ؼ��仯ʱ�رձ༭״̬
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
