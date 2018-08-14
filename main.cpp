#include <iostream>
#include<windows.h>    //头文件  
#include <fstream>
#include <opencv2\imgproc.hpp>  //opencv头文件
#include <opencv2\calib3d.hpp>
#include <opencv2\highgui.hpp>
#include <Kinect.h> //Kinect头文件
#include <string>
#include<direct.h>

using   namespace   std;
using   namespace   cv;
using	std::string;

void    draw(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper);
int main(void)
{
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IColorFrameSource   * myColorSource = nullptr;
	mySensor->get_ColorFrameSource(&myColorSource);

	IColorFrameReader   * myColorReader = nullptr;
	myColorSource->OpenReader(&myColorReader);

	int colorHeight = 0, colorWidth = 0;
	IFrameDescription   * myDescription = nullptr;
	myColorSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&colorHeight);
	myDescription->get_Width(&colorWidth);

	IColorFrame * myColorFrame = nullptr;
	Mat original(colorHeight, colorWidth, CV_8UC4);

	//**********************以上为ColorFrame的读取前准备**************************

	IBodyFrameSource    * myBodySource = nullptr;
	mySensor->get_BodyFrameSource(&myBodySource);

	IBodyFrameReader    * myBodyReader = nullptr;
	myBodySource->OpenReader(&myBodyReader);

	int myBodyCount = 0;
	myBodySource->get_BodyCount(&myBodyCount);

	IBodyFrame  * myBodyFrame = nullptr;

	ICoordinateMapper   * myMapper = nullptr;
	mySensor->get_CoordinateMapper(&myMapper);
	/**************************************************************/
	int flag = 0, k = 0, j = 0, m = 1, p = 1;


	char filename[60] = "save_data/all/", dataname[60], imagename[60], xxxname[60], folder_name[7];
	cout << "格式：p_a_r_" << endl;
	cin >> folder_name;
	//char filename[30] = "save_data/s2/d1/03/",dataname[30],imagename[30];
	

	sprintf_s(filename, "%s%s%s", filename, folder_name,"/");

	string filename_s(filename);

	_mkdir(filename_s.c_str());

	sprintf_s(dataname, "%s%s", filename, "data.txt");

	


	ofstream fout(dataname);
	//int a[] = { 7, 5, 4, 8, 9, 11, 20, 3, 0, 16, 17, 18, 12, 13, 14 };
	int a[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 ,9, 10, 11, 21, 22, 23, 24, 20};
	IplImage qImg;
	IplImage *dst = cvCreateImage(cvSize(400, 300), 8, 1);
	cvZero(dst);
	for (int i = 0; i<dst->height; i++)
		for (int j = 0; j<dst->width; j++)
		{
			cvSet2D(dst, i, j, cvScalar(0));
		}
	//**********************以上为BodyFrame以及Mapper的准备***********************
	while (1)
	{

		while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);
		myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, original.data, ColorImageFormat_Bgra);
		Mat copy = original.clone();        //读取彩色图像并输出到矩阵
		Mat copy2 = copy.clone();
		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK); //读取身体图像
		IBody   **  myBodyArr = new IBody *[myBodyCount];       //为存身体数据的数组做准备
		for (int i = 0; i < myBodyCount; i++)
			myBodyArr[i] = nullptr;

		if (myBodyFrame->GetAndRefreshBodyData(myBodyCount, myBodyArr) == S_OK)     //把身体数据输入数组
			for (int i = 0; i < myBodyCount; i++)
			{
				BOOLEAN     result = false;
				if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) //先判断是否侦测到
				{
					Joint   myJointArr[JointType_Count];
					if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)   //如果侦测到就把关节数据输入到数组并画图
					{
						draw(copy, myJointArr[JointType_Head], myJointArr[JointType_Neck], myMapper);
						draw(copy, myJointArr[JointType_Neck], myJointArr[JointType_SpineShoulder], myMapper);

						draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderLeft], myMapper);
						draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_SpineMid], myMapper);
						draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderRight], myMapper);

						draw(copy, myJointArr[JointType_ShoulderLeft], myJointArr[JointType_ElbowLeft], myMapper);
						draw(copy, myJointArr[JointType_SpineMid], myJointArr[JointType_SpineBase], myMapper);
						draw(copy, myJointArr[JointType_ShoulderRight], myJointArr[JointType_ElbowRight], myMapper);

						draw(copy, myJointArr[JointType_ElbowLeft], myJointArr[JointType_WristLeft], myMapper);
						draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipLeft], myMapper);
						draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipRight], myMapper);
						draw(copy, myJointArr[JointType_ElbowRight], myJointArr[JointType_WristRight], myMapper);

						draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_ThumbLeft], myMapper);
						draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_HandLeft], myMapper);
						draw(copy, myJointArr[JointType_HipLeft], myJointArr[JointType_KneeLeft], myMapper);
						draw(copy, myJointArr[JointType_HipRight], myJointArr[JointType_KneeRight], myMapper);
						draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_ThumbRight], myMapper);
						draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_HandRight], myMapper);

						draw(copy, myJointArr[JointType_HandLeft], myJointArr[JointType_HandTipLeft], myMapper);
						draw(copy, myJointArr[JointType_KneeLeft], myJointArr[JointType_FootLeft], myMapper);
						draw(copy, myJointArr[JointType_KneeRight], myJointArr[JointType_FootRight], myMapper);
						draw(copy, myJointArr[JointType_HandRight], myJointArr[JointType_HandTipRight], myMapper);
						/********************************************************************************/
						if (flag == 0)
							if (waitKey(20) == 's')
							{
								cout << "flag = 1 ,start collecting !" << endl;
								flag = 1;
								k = 0;
							}
						if (flag == 1)
							if (waitKey(20) == 'c')
							{
								cout << "flag = 0 ,stop collecting !" << endl;
								flag = 0;
								cout << "**********************Index flag and .jpg********************" << endl;
								fout << endl << endl
									<< "***************************************Here**********************************************"
									<< endl << endl;
								p = m;
								sprintf_s(xxxname, "%s%d%s%s", filename, p, "xxx", ".jpg");
								//cvSaveImage(xxxname, dst);
								imwrite(imagename, copy2);
							}
						if (flag == 1)
						{
							cout << "YES SIR !";
							k++;
							for (j = 0; j < 17; j++)
							{
								fout << ((float)((int)(myJointArr[a[j]].Position.X * 100000))) / 100000 << ","
									<< ((float)((int)(myJointArr[a[j]].Position.Y * 100000))) / 100000 << ","
									<< ((float)((int)(myJointArr[a[j]].Position.Z * 100000))) / 100000 << ",";

							}
							
							fout << endl;
							qImg = IplImage(copy2); // cv::Mat -> IplImage
							sprintf_s(imagename, "%s%s%d%s", filename, "rgb_", m++, ".jpg");
							//cvSaveImage(imagename, &qImg);
							imwrite(imagename, copy2);
							
						}
					}

				}
			}
		delete[]myBodyArr;
		myBodyFrame->Release();
		myColorFrame->Release();
		imshow("TEST", copy);
		if (waitKey(20) == VK_ESCAPE)
			break;
	}
	fout.close();
	myMapper->Release();

	myDescription->Release();
	myColorReader->Release();
	myColorSource->Release();

	myBodyReader->Release();
	myBodySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}

void    draw(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper)
{
	//用两个关节点来做线段的两端，并且进行状态过滤
	if (r_1.TrackingState == TrackingState_Tracked && r_2.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint t_point;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点
		Point   p_1, p_2;
		myMapper->MapCameraPointToColorSpace(r_1.Position, &t_point);
		p_1.x = t_point.X;
		p_1.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(r_2.Position, &t_point);
		p_2.x = t_point.X;
		p_2.y = t_point.Y;

		line(img, p_1, p_2, Vec3b(0, 255, 0), 5);
		circle(img, p_1, 10, Vec3b(255, 0, 0), -1);
		circle(img, p_2, 10, Vec3b(255, 0, 0), -1);
	}
}