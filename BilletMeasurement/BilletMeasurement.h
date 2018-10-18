#ifndef BILLETMEASUREMENT_H
#define BILLETMEASUREMENT_H

#include <QtWidgets/QMainWindow>
#include "ui_BilletMeasurement.h"
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
	//��ʼ��vtk����
	void initialVtkWidget();

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
