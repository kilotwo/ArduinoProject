#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include "Astar.h"
#include "Serial.h"
#include <time.h>
#include <pthread.h>
#include <algorithm>
#include <iostream>
#include <sstream>

#define CAM 0   //定义摄像头设备号
#define SERIAL_PORT "/dev/ttyUSB0"  //定义串口设备号
#define WIDTH_MIN 0
#define WIDTH_MAX 640

using namespace std;
using namespace cv;
using namespace zbar;

//构建对象
CAStar m_AStar;
Serial m_Serial;
VideoCapture cap;

enum Color{RED = 1, GREEN, BLUE};
enum Mode{GET = 1, PUT};

//定义变量
int sfd = 0;
int ret = 0;
int order_count = 0;
int order[2] = {0}; //存储物料抓取时摆放的位置，及放置时摆放的位置
string zbar_data = "12";  //存储二维码数据
string serial_data = ""; //存储串口数据

//整型转字符串
string int2string(int num)
{
    stringstream ss;
    string str;
    ss << num;
    str = ss.str();
    ss.clear();
    return str;
}

//字符串拼接
string StringConcat(int px, int py)
{
    return int2string(px) + '_' + int2string(py) + "\n";
}

//二维码检测，返回值为布尔值
bool readQRCode(const Mat inputImage)
{
    ImageScanner scanner;// 所转化成的灰度图像,定义一个扫描仪
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	Mat imageGray;
	cvtColor(inputImage,imageGray,CV_RGB2GRAY);
    int width = imageGray.cols;
	int height = imageGray.rows;
	// 在Zbar中进行扫描时候，需要将OpenCV中的Mat类型转换为（uchar *）类型，raw中存放的是图像的地址；对应的图像需要转成Zbar中对应的图像zbar::Image
	uchar *raw = (uchar *)imageGray.data;
	Image imageZbar(width, height, "Y800", raw, width * height);
	scanner.scan(imageZbar); //扫描条码
	Image::SymbolIterator symbol = imageZbar.symbol_begin();
	if(imageZbar.symbol_begin()==imageZbar.symbol_end())
	{
		//cout<<"查询条码失败，请检查图片！"<<endl;
		return false;
	}
	else
    {
        for(;symbol != imageZbar.symbol_end();++symbol)
        {
            cout<<"Type："<<endl<<symbol->get_type_name()<<endl<<endl;
            cout<<"Code："<<endl<<symbol->get_data()<<endl<<endl;
            zbar_data = symbol->get_data();
        }

        //imshow("Source Image",inputImage);
        imageZbar.set_data(NULL,0);
        return true;
    }
}

//颜色检测，返回值为颜色序号
int ColorDetect(int color, const Mat &inputImage)
{

    int point_x;
    int iLowH, iHighH;
    int iLowS = 43, iHighS = 255, iLowV = 46, iHighV = 255;

    switch(color)
    {
    case RED:
        iLowH = 0;
        iHighH = 10; //10
        break;
    case GREEN:
        iLowH = 33; //35
        iHighH = 80; //77
        break;
    case BLUE:
        iLowH = 100; //100
        iHighH = 134;
        break;
    }

    //Mat img = imread("color.jpg",1);
    Mat img, imgHSV;
    inputImage.copyTo(img);

    cvtColor(img, imgHSV, COLOR_BGR2HSV);//转为HSV

    //imwrite("hsv.jpg",imgHSV);

    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //开操作 (去除一些噪点)  如果二值化后图片干扰部分依然很多，增大下面的size
    Mat element = getStructuringElement(MORPH_RECT, Size(40, 40));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    //闭操作 (连接一些连通域)
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    //namedWindow("Thresholded Image",CV_WINDOW_NORMAL);
    //imshow("Thresholded Image", imgThresholded);


    vector<vector <Point> >contours;
    vector<Vec4i>hierarchy;
    findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);//查找轮廓

    int count = 0;
    Point pt[512];//存储连通区域个数
    Moments moment;//矩
    vector<Point>Center;//创建一个向量保存重心坐标

    for (int i=0;i<contours.size();i++)//读取每一个轮廓求取重心
    {
        Mat temp(contours.at(i));
        Scalar color(255, 255, 255);
        moment = moments(temp, false);

        if (moment.m00 != 0)//除数不能为0
        {
            pt[i].x = cvRound(moment.m10 / moment.m00);//计算重心横坐标
            pt[i].y = cvRound(moment.m01 / moment.m00);//计算重心纵坐标
            point_x = pt[i].x;
        }
        Point p = Point(pt[i].x, pt[i].y);//重心坐标
        circle(img, p, 1, color, 5, 8);//原图画出重心坐标
        count++;//重心点数或者是连通区域数
        Center.push_back(p);//将重心坐标保存到Center向量中
    }

    //cout << "重心点个数：" << Center.size() << endl;
    //cout << "轮廓数量：" << contours.size() << endl;

    //显示染色检测结果，并保存图片
    //imshow("result", inputImage);
    string name = "end"+int2string(color)+".jpg";
    imwrite(name, img);
    int result = (point_x > WIDTH_MIN) && (point_x < WIDTH_MAX) ? color : 0;
    point_x = 0;
    return result;
}

//串口检测线程函数
void *SerialReadThread(void *)
{
    while(1)
    {
        serial_data = string(m_Serial.serial_read(sfd));
        //cout << serial_data << endl;
    }
    return 0;
}

//抓取及放置函数
void GrabPut(int mode, const Mat &originImg)
{
    if(order_count > 2)
    {
        string cmd = "";

        for(int i=0;i<zbar_data.length();i++)
        {
            for(int j=0;j<sizeof(order)/sizeof(order[0]);j++)
            {
                if((int)(zbar_data[i] - '0') == order[j])
                {
                    cmd += int2string(j);
                }

            }
        }
        cout << "original order: " << cmd << endl;
        switch(mode)
        {
        case GET:
            cout << "final order: " << cmd << endl;
            break;
        case PUT:
            reverse(cmd.begin(), cmd.end());
            cout << "final order: " << cmd << endl;
            break;

        }
        m_Serial.serial_send(sfd, cmd.c_str(), sizeof(cmd).c_str());
        order_count = 0;
    }
    else
    {
        int number[2] = {0};
        for(int i=0;i<2;i++)
        {
           number[i] = ColorDetect((i+1), originImg);       //颜色检测存入数组

           if(number[i])
           {
                order[order_count] = number[i];
                cout << number[i] << endl;
           }

        }
        order_count++;
        cout << "detect " + int2string(order_count) + " complete!" << endl;
        m_Serial.serial_send(sfd, "ok", sizeof("ok"));
    }
}

int main()
{
    /***********************串口部分***********************/
    sfd = m_Serial.openSerialDevice(SERIAL_PORT);

    if(sfd < 0)
    {
        cout << "open serial port failed!" << endl;
        return 0;
    }
    else
    {
        cout << "open serial port successful!" << endl;
    }

    //设置波特率
    m_Serial.set_speed(sfd, 9600);
    //设置串口标志位
    ret = m_Serial.set_Parity(sfd, 8, 1, 'N');
    /**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  char  效验类型 取值为N,E,O,,S
*/

    if(ret < 0)
    {
        cout << "set parity error, ret:" << ret << endl;
        return 0;
    }

    /***********************摄像头部分***********************/
    //打开摄像头设备
    cap.open(CAM);
    //判断摄像头是否打开
    if(!cap.isOpened())
    {
        cout << "open camera failed!" << endl;
        return -1;
    }
    else
    {
        cout << "open camera successful!" << endl;
    }

    //存储每一帧图片
    Mat frame;
    //开启串口检测线程
    pthread_t id;
    int th_ret = pthread_create(&id, NULL, SerialReadThread, NULL);

    if(th_ret)
    {
        cout << "Create pthread error!\n" << endl;
        return 1;
    }

    //进入主循环
    for(;;)
    {
        cap >> frame;

        if(frame.empty())
            break;

        if(serial_data != "")
        {
            if(serial_data == "qr")
            {
                //接收下位机检测命令，进入二维码识别模式
                 if(!readQRCode(frame))   //调用二维码识别函数，识别返回true，未识别返回false
                 {
                     cout << "No qr code is detected!" << endl;
                     m_Serial.serial_send(sfd, "error", sizeof("error")); //未检测到二维码，发送错误标志，进行重复检测
                 }
                 else
                 {
                     m_Serial.serial_send(sfd, zbar_data.c_str(), sizeof(zbar_data.c_str()));
                 }
            }   //此处有新增
            else if(serial_data == "get")
            {
                //接收到下位机检测命令，进行抓取检测
                GrabPut(GET, frame);
            }
   /*         else if(serial_data == "put")
            {
                //接收到下位机命令，进行放置检测
                GrabPut(PUT, frame);
            }           */
            //清空串口缓冲区
            m_Serial.serial_clear();
            serial_data = "";
        }

        imshow("video", frame);
        char c = (char)waitKey(1); //每帧延时10毫秒

        if(c == 27) //"Esc"键，直接退出
            break;
    }

    //等待线程结束
    pthread_join(id, NULL);
    return 0;
}
