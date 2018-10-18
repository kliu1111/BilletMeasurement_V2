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

	//�������
	CBaslerGigEInstantCameraArray CamList;
	QImage LeftImg;
	QImage RightImg;
	const QPixmap *Lpixmap;
	const QPixmap *Rpixmap;
	QString str_time;
	QDateTime time;

	//���Ӽ�⵽�����е����
	void Connect();

	//������е�����Ƿ�������
	bool IsConnected();

	//ж�����е��豸��ֹ�������ر�ʱ����
	void DetachDevice();

	//�Ͽ����е�������������²ɼ����������������
	void Disconnect();

	//��������󣬽��вɼ�������
	void StartGrabbing();

	//������е�����Ƿ񶼴��ڲɼ�״̬
	bool IsGrabbing();

	//ֹͣ�ɼ����������²ɼ���ֱ��StartGrabbing��
	void StopGrabbing();

	//ȡ������ĳ�ʼ����
	void GetCamInitPara();

	//�����������
	void SetCamPara(CameraPara CamPara);
	void SetIniCamParam(CameraPara CamPara);

	//�ɼ��߳�
	static void GrabThread(BaslerCamera* pClass);
	void GrabThreadFunction();

	//��������label(ͼ����ʾ)�ؼ��ĵ�ַ
	QLabel* LeftUpLabelAdd;
	QLabel* RightUpLabelAdd;
	QLabel* LeftDownLabelAdd;
	QLabel* RightDownLabelAdd;
	void GetLeftUpLabelAddress(QLabel &LeftUpLabelPointer);
	void GetRightUpLabelAddress(QLabel &RightUpLabelPointer);
	void GetLeftDownLabelAddress(QLabel &LeftDownLabelPointer);
	void GetRightDownLabelAddress(QLabel &RightDownLabelPointer);
	//treewidget��ַ
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

