/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<Frame.h>
#include<MapPoint.h>
#include<Settings.h>
#include<ORBextractor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fstream>

#include <netinet/in.h>	//套接字地址的相关结构体和网络字节序列转换的相关函数
#include <arpa/inet.h>	//IP地址转换为网络字节序列的相关函数
#include <sys/types.h>
#include <sys/socket.h>	//套接子建立的相关函数
#include <unistd.h>

#include <zlib.h>
#include <zconf.h>

#define IMAGE_HEIGHT 350
#define IMAGE_WIDTH 600

#define KBUF_SIZE 60000
#define DBUF_SIZE 160000

using namespace std;

class MyPoint
{
public:
    MyPoint(cv::KeyPoint kp)
    {
        angle = kp.angle;
        pt = kp.pt;
    }

    float angle;

    cv::Point2f pt;
};

class FrameHeader
{
public:
    int n_frames;
    double tframe;
    int key_size;
    int des_size;
    int levels[8];
};

class Reply
{
public:
    double tframe;
    double match_rate;
};

void LoadImages(const string& strImagePath, const string& strPathTimes,
    vector<string>& vstrImages, vector<double>& vTimeStamps);

int safe_recv(int socket, char* buffer, int size, int flag = 0);

int recv_frame(int socket, ORB_SLAM3::Frame* frame);

int recv_frame(int socket, FrameHeader* fh, ORB_SLAM3::Frame* frame);

int safe_send(int socket, void* data, int size, int flag = 0);

char rev_buf[KBUF_SIZE + DBUF_SIZE] = { 0 };
unsigned char key_buf[KBUF_SIZE] = { 0 };
unsigned char des_buf[DBUF_SIZE] = { 0 };

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    int socket_server;
    int socket_connect;
    struct sockaddr_in server_address;	//保存服务端的地址相关的参数
    struct sockaddr_in client_address;	//保存接收到的客户端的地址相关的参数
    struct linger socket_exit;	//确定套接子的退出的方式
    struct timeval socket_OT;	//定义套接字的超时的时间

    //初始化我们要使用的相关数据-----------------------
    memset(&server_address, 0x00, sizeof(server_address));
    memset(&client_address, 0x00, sizeof(client_address));

    //开始创建服务端套接字------------------------------
    socket_server = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_server < 0)
    {
        fprintf(stderr, "Create socket error, the error is %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);     //INADDR_ANY表示任意地址
    server_address.sin_port = htons(23940);
    //将套接字和地址结构体绑定
    int ret = bind(socket_server, (struct sockaddr*)&server_address, sizeof(server_address));
    if (ret < 0)
    {
        fprintf(stderr, "Bind error, The error is %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    //设置套接字属性
    socket_exit.l_onoff = 0;
    socket_OT.tv_sec = 60;
    socket_OT.tv_usec = 0;

    setsockopt(socket_server, SOL_SOCKET, SO_LINGER, &socket_exit, sizeof(socket_exit));
    setsockopt(socket_server, SOL_SOCKET, SO_RCVTIMEO, &socket_OT, sizeof(socket_OT));
    setsockopt(socket_server, SOL_SOCKET, SO_SNDTIMEO, &socket_OT, sizeof(socket_OT));

    //监听套接字
    ret = listen(socket_server, 1);
    if (ret < 0)
    {
        fprintf(stderr, "listen error, The error is %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    //等待连接
    socklen_t size = sizeof(client_address);
    socket_connect = accept(socket_server, (struct sockaddr*)&client_address, &size);
    if (socket_connect < 0)
    {
        fprintf(stderr, "accpet error, The error is %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
    printf("accpet success!\n");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    double tframe;

    int live_flag = 1;
    char fh_buf[sizeof(FrameHeader)] = { 0 };

    int state = SLAM.GetTrackingState();
    bool isInitializing = ((state == 0) || (state == 1));
    cout.precision(17);

	vector<int> matchesIndexs;
    while (live_flag)
    {
        safe_recv(socket_connect, fh_buf, sizeof(FrameHeader));
        FrameHeader* fh = (FrameHeader*)fh_buf;
        
        tframe = fh->tframe;
        if (tframe + 1 < 1e-6 && tframe + 1 > -1e-6)     //tframe == -1
        {
            live_flag = 0;
            break;
        }

        ORB_SLAM3::Frame* pFrame = new ORB_SLAM3::Frame();
        int n = recv_frame(socket_connect, fh, pFrame);
		SLAM.TrackMonocular(*pFrame, tframe);
		
        Reply reply;
        reply.tframe = tframe;

        state = SLAM.GetTrackingState();
        if (isInitializing)
        {
            isInitializing = ((state == 0) || (state == 1));
            reply.match_rate = 0;
        }
        else
        {
            matchesIndexs = SLAM.GetMatches();
            int nMatches = matchesIndexs.size();
            reply.match_rate = (double)nMatches / n;
        }

        safe_send(socket_connect, &reply, sizeof(reply));
        delete pFrame;
    }

    int stop;
    cout << "enter any key to exit" << endl;
    cin >> stop;
    // Stop all threads
    SLAM.Shutdown();

    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

int safe_recv(int socket, char* buffer, int size, int flag)
{
    int len = 0;
    while (len < size)
    {
        int ret = recv(socket, buffer + len, size - len, flag);

        if (ret == 0)
        {
            printf("Connect Over!\n");
            exit(EXIT_FAILURE);                                               //进程异常退出
        }
        else if (ret < 0)
        {
            fprintf(stderr, "Send error, The error is %s\n", strerror(errno));//输出错误信息
            exit(EXIT_FAILURE);                                               //程序异常终止
        }

        len += ret;
    }
    return len;
}

int recv_frame(int socket, FrameHeader* fh, ORB_SLAM3::Frame* frame)
{
    safe_recv(socket, rev_buf, fh->key_size + fh->des_size);
    unsigned long key_len = KBUF_SIZE;
    uncompress(key_buf, &key_len, (unsigned char*)rev_buf, fh->key_size);
    unsigned long des_len = DBUF_SIZE;
    uncompress(des_buf, &des_len, (unsigned char*)rev_buf + fh->key_size, fh->des_size);

    int n = key_len / sizeof(MyPoint);

    frame->mvKeys.resize(n);

    MyPoint* mp = (MyPoint*)key_buf;

    int index = 0;
    for (int octave = 7; octave >= 0; --octave)
    {
        int nLevel = fh->levels[octave];
        for (int i = 0; i < nLevel; ++i)
        {
            frame->mvKeys[index].angle = mp[index].angle;
            frame->mvKeys[index].pt = mp[index].pt;
            frame->mvKeys[index].octave = octave;
            index++;
        }
    }

    cv::Mat mat(n, 32, CV_8U, des_buf);
    frame->mDescriptors = mat.clone();     //必须使用clone以复制mat.data
    return n;
}

int safe_send(int socket, void* data, int size, int flag)
{
    int len = 0;
    while (len < size)
    {
        int ret = send(socket, data + len, size - len, flag);

        if (ret == 0)
        {
            printf("Connect Over!\n");
            exit(EXIT_FAILURE);                                               //进程异常退出
        }
        else if (ret < 0)
        {
            fprintf(stderr, "Send error, The error is %s\n", strerror(errno));//输出错误信息
            exit(EXIT_FAILURE);                                               //程序异常终止
        }

        len += ret;
    }
    return len;
}
