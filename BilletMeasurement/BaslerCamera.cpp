#include "BaslerCamera.h"


BaslerCamera::BaslerCamera()
{
}

BaslerCamera::~BaslerCamera()
{
	PylonTerminate();
}

//basler相机在开发使用过程中，重新打开相机时会出现该相机已占用的情况，该类可以有效解决。
class CHeartbeatHelper
{
public:
	explicit CHeartbeatHelper(CInstantCamera& camera)
		: m_pHeartbeatTimeout(NULL)
	{
		// m_pHeartbeatTimeout may be NULL
		m_pHeartbeatTimeout = camera.GetTLNodeMap().GetNode("HeartbeatTimeout");
	}

	bool SetValue(int64_t NewValue)
	{
		// Do nothing if no heartbeat feature is available.
		if (!m_pHeartbeatTimeout.IsValid())
			return false;

		// Apply the increment and cut off invalid values if neccessary.
		int64_t correctedValue = NewValue - (NewValue % m_pHeartbeatTimeout->GetInc());

		m_pHeartbeatTimeout->SetValue(correctedValue);
		return true;
	}

	bool SetMax()
	{
		// Do nothing if no heartbeat feature is available.
		if (!m_pHeartbeatTimeout.IsValid())
			return false;

		int64_t maxVal = m_pHeartbeatTimeout->GetMax();
		return SetValue(maxVal);
	}

protected:
	GenApi::CIntegerPtr m_pHeartbeatTimeout; // Pointer to the node, will be NULL if no node exists.
};

vector<int> DecimalToBinary(qlonglong decimal)
{
	int remainder;
	vector<int> binary;
	while (decimal != 0)
	{
		remainder = decimal % 2;
		decimal /= 2;
		binary.push_back(remainder);
	}
	reverse(binary.begin(), binary.end());

	return binary;
}

string DecimalToMac(int64_t decimal)
{
	int remainder;
	string binary;
	while (decimal != 0)
	{
		remainder = decimal % 16;
		decimal /= 16;
		if (remainder == 10)
		{
			binary.append("A");
		}
		else if (remainder == 11)
		{
			binary.append("B");
		}
		else if (remainder == 12)
		{
			binary.append("C");
		}
		else if (remainder == 13)
		{
			binary.append("D");
		}
		else if (remainder == 14)
		{
			binary.append("E");
		}
		else if (remainder == 15)
		{
			binary.append("F");
		}
		else
		{
			binary.append(to_string(remainder));
		}
	}
	reverse(binary.begin(), binary.end());
	binary.insert(0, "00");
	for (int i = 2; i < binary.size(); i = i + 3)
	{
		binary.insert(i, ":");
	}

	return binary;
}

string BinaryToIp(vector<int> binary)
{
	string ip;
	int partsum = 0;
	for (size_t i = 0; i < binary.size(); i++)
	{

		if (binary.at(i))
		{
			partsum = partsum + pow(2, (7 - i % 8));
		}

		if (i == 7 || i == 15 || i == 23 || i == 31)
		{
			ip.append(to_string(partsum));
			partsum = 0;
			if (i != 31)
			{
				ip.append(".");
			}
		}
	}
	return ip;
}

void BaslerCamera::Connect()
{
	//对所有的相机设备进行初始化
	PylonInitialize();

	try
	{
		CTlFactory& tlFactory = CTlFactory::GetInstance();
		DeviceInfoList_t deviceList;
		//没有检测到相机时及时提醒
		if (tlFactory.EnumerateDevices(deviceList) == 0)
		{
			QMessageBox::critical(NULL, QStringLiteral("提示"), QStringLiteral("未检测到相机,请检查硬件连接！！！"),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
			return;
		}

		//打开所有已检测到的相机设备
		size_t nSize = deviceList.size();
		CamList.Initialize(nSize);

		//指定挂载设备
		//string CamName0 = tlFactory.CreateDevice(deviceList[0])->GetDeviceInfo().GetUserDefinedName();
		//string CamName1 = tlFactory.CreateDevice(deviceList[1])->GetDeviceInfo().GetUserDefinedName();
		//string CamName2 = tlFactory.CreateDevice(deviceList[2])->GetDeviceInfo().GetUserDefinedName();
		//string CamName3 = tlFactory.CreateDevice(deviceList[3])->GetDeviceInfo().GetUserDefinedName();
		//CamList[CamName0[4] - 1].Attach(tlFactory.CreateDevice(deviceList[0]));
		//CamList[CamName1[4] - 1].Attach(tlFactory.CreateDevice(deviceList[1]));
		//CamList[CamName2[4] - 1].Attach(tlFactory.CreateDevice(deviceList[2]));
		//CamList[CamName3[4] - 1].Attach(tlFactory.CreateDevice(deviceList[3]));

		string CamName = tlFactory.CreateDevice(deviceList[0])->GetDeviceInfo().GetUserDefinedName();
		if (CamName == "Cam1")
		{
			CamList[0].Attach(tlFactory.CreateDevice(deviceList[0]));
			CamList[1].Attach(tlFactory.CreateDevice(deviceList[1]));
		}
		else
		{
			CamList[1].Attach(tlFactory.CreateDevice(deviceList[0]));
			CamList[0].Attach(tlFactory.CreateDevice(deviceList[1]));

		}

		CamList.Open();
		CamList[0].TriggerMode.SetValue(Basler_GigECamera::TriggerMode_On);
		CamList[1].TriggerMode.SetValue(Basler_GigECamera::TriggerMode_On);
		//CamList[2].TriggerMode.SetValue(Basler_GigECamera::TriggerMode_On);
		//CamList[3].TriggerMode.SetValue(Basler_GigECamera::TriggerMode_On);
		CamList[0].TriggerSource.SetValue(Basler_GigECamera::TriggerSource_Line1);
		CamList[1].TriggerSource.SetValue(Basler_GigECamera::TriggerSource_Line1);
		//CamList[2].TriggerSource.SetValue(Basler_GigECamera::TriggerSource_Line1);
		//CamList[3].TriggerSource.SetValue(Basler_GigECamera::TriggerSource_Line1);

		//该代码有效解决相机被占用的问题
		CHeartbeatHelper cam1_heartbeatHelper(CamList[0]);
		CHeartbeatHelper cam2_heartbeatHelper(CamList[1]);
		cam1_heartbeatHelper.SetValue(500);  // 1000 ms timeout
		cam2_heartbeatHelper.SetValue(500);  // 1000 ms timeout
		//CHeartbeatHelper cam3_heartbeatHelper(CamList[2]);
		//CHeartbeatHelper cam4_heartbeatHelper(CamList[3]);
		//cam3_heartbeatHelper.SetValue(500);  // 1000 ms timeout
		//cam4_heartbeatHelper.SetValue(500);  // 1000 ms timeout

	}
	catch (const GenericException &e)
	{
		cerr << "An exception occurred." << endl << e.GetDescription() << endl;
	}

}

bool BaslerCamera::IsConnected()
{
	return CamList.IsOpen();
}

void BaslerCamera::DetachDevice()
{
	CamList.DetachDevice();
	CamList.DestroyDevice();
}

void BaslerCamera::Disconnect()
{
	try
	{
		if (CamList.GetSize() == 0)
		{
			QMessageBox::about(NULL, QStringLiteral("提示"), QStringLiteral("当前无相机连接，无需断开！！！"));
			return;
		}
		CamList.Close();
		CamList.DestroyDevice();
		PylonTerminate();
	}
	catch (GenICam::GenericException &e)
	{
		// Error handling.
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
	}
}

void BaslerCamera::StartGrabbing()
{
	if (CamList.IsGrabbing())
	{
		QMessageBox::about(NULL, QStringLiteral("提示"), QStringLiteral("相机正处于采集状态！！！"));
		return;
	}
	if (!CamList.IsOpen())
	{
		QMessageBox::about(NULL, QStringLiteral("提示"), QStringLiteral("请连接相机后再进行采集！！！"));
		return;
	}
	CamList.StartGrabbing();
	//开启线程
	std::thread Thread(BaslerCamera::GrabThread, this);
	Thread.detach();
}

void BaslerCamera::GrabThread(BaslerCamera* pClass)
{
	pClass->GrabThreadFunction();
}

void BaslerCamera::GrabThreadFunction()
{
	try
	{
		while (CamList[0].IsGrabbing() && CamList[1].IsGrabbing())
		{
			if (CamList[0].RetrieveResult(5000, LeftUpptrGrabResult, TimeoutHandling_Return) && CamList[1].RetrieveResult(5000, RightUpptrGrabResult, TimeoutHandling_Return))
			{
				if (LeftUpptrGrabResult->GrabSucceeded() && RightUpptrGrabResult->GrabSucceeded())
				{
					LeftImg = QImage((unsigned char *)(LeftUpptrGrabResult->GetBuffer()), LeftUpptrGrabResult->GetWidth(), LeftUpptrGrabResult->GetHeight(), QImage::Format_Indexed8);
					RightImg = QImage((unsigned char *)(RightUpptrGrabResult->GetBuffer()), RightUpptrGrabResult->GetWidth(), RightUpptrGrabResult->GetHeight(), QImage::Format_Indexed8);

					LeftUpLabelAdd->setPixmap(QPixmap::fromImage(LeftImg).scaled(LeftUpLabelAdd->size(), Qt::KeepAspectRatioByExpanding));
					RightUpLabelAdd->setPixmap(QPixmap::fromImage(RightImg).scaled(RightUpLabelAdd->size(), Qt::KeepAspectRatioByExpanding));
					Lpixmap = LeftUpLabelAdd->pixmap();
					Rpixmap = RightUpLabelAdd->pixmap();
					if (Rpixmap && Lpixmap)
					{
						time = QDateTime::currentDateTime();
						//str_time = time.toString("yyyy-MM-dd_hh-mm-ss");
						str_time = time.toString("yyyy-MM-dd-hh-mm-ss-zzz");
						Lpixmap->save(("image/L_image/") + str_time + ".jpg");
						Rpixmap->save(("image/R_image/") + str_time + ".jpg");
					//	//CImagePersistence::Save(ImageFileFormat_Png, FileName, LeftUpptrGrabResult);
					//	//CImagePersistence::Save(ImageFileFormat_Tiff, aa, LeftUpptrGrabResult);
					//	CImagePersistence::Save(ImageFileFormat_Tiff, "image/aaa.jpg", RightUpptrGrabResult);
					//	
					}

					QCoreApplication::processEvents();
				}
			}
		}

	}
	catch (GenICam::GenericException &e)
	{
		// Error handling.
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
	}

}

bool BaslerCamera::IsGrabbing()
{
	return CamList.IsGrabbing();
}

void BaslerCamera::StopGrabbing()
{
	if (!CamList.IsGrabbing())
	{
		QMessageBox::about(NULL, QStringLiteral("提示"), QStringLiteral("当前无相机采集，无需停止采集！！！"));
		return;
	}
	try
	{
		CamList.StopGrabbing();

	}
	catch (GenICam::GenericException &e)
	{
		// Error handling.
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
	}
}

void BaslerCamera::GetCamInitPara()
{
	//-----数组
	//int CamNumber = CamList.GetSize();
	//for (int i = 0; i < CamNumber;i++)
	//{
	//	QString aa = CamList[i].GetDeviceInfo().GetFriendlyName();
	//	CamInfoList[i] = new QTreeWidgetItem(CamInfo, QStringList(QStringLiteral("相机型号")));
	//	CamInfoList[i]->setText(1, QString(CamList[i].GetDeviceInfo().GetUserDefinedName()));//ID
	//	CamInfoList[i]->setText(2, QString::fromStdString(DecimalToMac(CamList[i].GevMACAddress.GetValue())));
	//	CamInfoList[i]->setText(3, QString::fromStdString(BinaryToIp(DecimalToBinary(CamList[i].GevCurrentIPAddress.GetValue()))));
	//	CamInfoList[i]->setText(4, QStringLiteral("已连接"));
	//	CamInfoList[i]->setText(5, QString::number(CamList[i].AcquisitionFrameRateAbs.GetValue()));
	//	CamInfoList[i]->setText(6, QString::number(CamList[i].ExposureTimeAbs.GetValue()));
	//	CamInfoList[i]->setText(7, QString::number(CamList[i].Height.GetValue()));
	//	CamInfoList[i]->setText(8, QString::number(CamList[i].Width.GetValue()));
	//}

	//------Cam1
	if (CamList[0].IsOpen())
	{
		Cam1Info = new QTreeWidgetItem(CamInfo, QStringList(QString(CamList[0].GetDeviceInfo().GetModelName())));
		Cam1Info->setText(1, QString(CamList[0].GetDeviceInfo().GetUserDefinedName()));//ID
		Cam1Info->setText(2, QString::fromStdString(DecimalToMac(CamList[0].GevMACAddress.GetValue())));
		Cam1Info->setText(3, QString::fromStdString(BinaryToIp(DecimalToBinary(CamList[0].GevCurrentIPAddress.GetValue()))));
		Cam1Info->setText(4, QStringLiteral("已连接"));
		Cam1Info->setText(5, QString::number(CamList[0].AcquisitionFrameRateAbs.GetValue()));
		Cam1Info->setText(6, QString::number(CamList[0].ExposureTimeAbs.GetValue()));
		Cam1Info->setText(7, QString::number(CamList[0].Width.GetValue()));
		Cam1Info->setText(8, QString::number(CamList[0].Height.GetValue()));
	} 
	//------Cam2
	if (CamList[1].IsOpen())
	{
		Cam2Info = new QTreeWidgetItem(CamInfo, QStringList(QString(CamList[1].GetDeviceInfo().GetModelName())));
		Cam2Info->setText(1, QString(CamList[1].GetDeviceInfo().GetUserDefinedName()));//ID
		Cam2Info->setText(2, QString::fromStdString(DecimalToMac(CamList[1].GevMACAddress.GetValue())));
		Cam2Info->setText(3, QString::fromStdString(BinaryToIp(DecimalToBinary(CamList[1].GevCurrentIPAddress.GetValue()))));
		Cam2Info->setText(4, QStringLiteral("已连接"));
		Cam2Info->setText(5, QString::number(CamList[1].AcquisitionFrameRateAbs.GetValue()));
		Cam2Info->setText(6, QString::number(CamList[1].ExposureTimeAbs.GetValue()));
		Cam2Info->setText(7, QString::number(CamList[1].Width.GetValue()));
		Cam2Info->setText(8, QString::number(CamList[1].Height.GetValue()));
	}

	////------Cam3
	//if (CamList[2].IsOpen())
	//{
	//	Cam3Info = new QTreeWidgetItem(CamInfo, QStringList(QString(CamList[2].GetDeviceInfo().GetModelName())));
	//	Cam3Info->setText(1, QString(CamList[2].GetDeviceInfo().GetUserDefinedName()));//ID
	//	Cam3Info->setText(2, QString::fromStdString(DecimalToMac(CamList[2].GevMACAddress.GetValue())));
	//	Cam3Info->setText(3, QString::fromStdString(BinaryToIp(DecimalToBinary(CamList[2].GevCurrentIPAddress.GetValue()))));
	//	Cam3Info->setText(4, QStringLiteral("已连接"));
	//	Cam3Info->setText(5, QString::number(CamList[2].AcquisitionFrameRateAbs.GetValue()));
	//	Cam3Info->setText(6, QString::number(CamList[2].ExposureTimeAbs.GetValue()));
	//	Cam3Info->setText(7, QString::number(CamList[2].Height.GetValue()));
	//	Cam3Info->setText(8, QString::number(CamList[2].Width.GetValue()));
	//}

	////------Cam4
	//if (CamList[3].IsOpen())
	//{
		//Cam4Info = new QTreeWidgetItem(CamInfo, QStringList(QString(CamList[3].GetDeviceInfo().GetModelName())));
		//Cam4Info->setText(1, QString(CamList[3].GetDeviceInfo().GetUserDefinedName()));//ID
		//Cam4Info->setText(2, QString::fromStdString(DecimalToMac(CamList[3].GevMACAddress.GetValue())));
		//Cam4Info->setText(3, QString::fromStdString(BinaryToIp(DecimalToBinary(CamList[3].GevCurrentIPAddress.GetValue()))));
		//Cam4Info->setText(4,  QStringLiteral("已连接"));
		//Cam4Info->setText(5, QString::number(CamList[3].AcquisitionFrameRateAbs.GetValue()));
		//Cam4Info->setText(6, QString::number(CamList[3].ExposureTimeAbs.GetValue()));
		//Cam4Info->setText(7, QString::number(CamList[3].Height.GetValue()));
		//Cam4Info->setText(8, QString::number(CamList[3].Width.GetValue()));
	//}


}
void BaslerCamera::SetIniCamParam(CameraPara CamPara)
{
	CamList[0].AcquisitionFrameRateAbs.SetValue(CamPara.LeftImageFrequency);
	CamList[0].ExposureTimeAbs.SetValue(CamPara.LeftExposureTime);
	CamList[0].Width.SetValue(CamPara.LeftImageWidth);
	CamList[0].Height.SetValue(CamPara.LeftImageHeight);

	CamList[1].AcquisitionFrameRateAbs.SetValue(CamPara.RightImageFrequency);
	CamList[1].ExposureTimeAbs.SetValue(CamPara.RightExposureTime);
	CamList[1].Width.SetValue(CamPara.RightImageWidth);
	CamList[1].Height.SetValue(CamPara.RightImageHeight);
}
void BaslerCamera::SetCamPara(CameraPara CamPara)
{
	try
	{
		//CamList[0].GevPersistentIPAddress.SetValue(Para.LeftUpCamIp.toInt());
		//int aa = Para.LeftUpImageFrequency;
		CamList[0].AcquisitionFrameRateAbs.SetValue(CamPara.LeftImageFrequency);
		CamList[0].ExposureTimeAbs.SetValue(CamPara.LeftExposureTime);
		CamList[0].Width.SetValue(CamPara.LeftImageWidth);
		CamList[0].Height.SetValue(CamPara.LeftImageHeight);

		CamList[1].AcquisitionFrameRateAbs.SetValue(CamPara.RightImageFrequency);
		CamList[1].ExposureTimeAbs.SetValue(CamPara.RightExposureTime);
		CamList[1].Width.SetValue(CamPara.RightImageWidth);
		CamList[1].Height.SetValue(CamPara.RightImageHeight);
	}
	catch (const GenericException &e)
	{
		// Error handling
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
	}
}

void BaslerCamera::GetLeftUpLabelAddress(QLabel &LeftUpLabelPointer)
{
	LeftUpLabelAdd = &LeftUpLabelPointer;
}
void BaslerCamera::GetRightUpLabelAddress(QLabel &RightUpLabelPointer)
{
	RightUpLabelAdd = &RightUpLabelPointer;
}
void BaslerCamera::GetLeftDownLabelAddress(QLabel &LeftDownLabelPointer)
{
	LeftDownLabelAdd = &LeftDownLabelPointer;
}
void BaslerCamera::GetRightDownLabelAddress(QLabel &RightDownLabelPointer)
{
	RightDownLabelAdd = &RightDownLabelPointer;
}
void BaslerCamera::GetCamInfoListAddress(QTreeWidget &CamInfoListPointer)
{
	CamInfo = &CamInfoListPointer;
}