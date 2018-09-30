/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#define WIN32_LEAN_AND_MEAN
#define _WIN32_WINNT 0x501

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <mmsystem.h>
#include <SFML/Audio.hpp>
#include <thread>
#include<mutex>
#include <SFML/Audio/SoundSource.hpp>
#pragma comment(lib, "Winmm.lib")

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


#define DEFAULT_BUFLEN 50012
#define DEFAULT_PORT "27015"


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>


using namespace std;

extern void run_detector(int argc, char **argv);

bool createThread = true;
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);

void extract_values(string yolo,vector<string> &vstrLabel, vector<cv::Mat> &vobjPosition);



int main(int argc, char **argv)
{
	if (argc != 4)
	{
		cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
		return 1;
	}

	
	


	//CLIENT-SIDE


	WSADATA wsaData;
	SOCKET ConnectSocket = INVALID_SOCKET;
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;
	char sendbuf[2000] = "img.jpg";
	char recvbuf[DEFAULT_BUFLEN];
	int iResult;
	int recvbuflen = DEFAULT_BUFLEN;
	

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("localhost", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 1;
	}


	// ------------------



	//system("start cd C:/Users/Leonardo/Documents/Git_Repositories/YOLOv2_SLAM_ORB2/darknet/build/darknet/x64 && darknet.exe detector test data/coco.data yolo.cfg yolo.weights -i 0 -thresh 0.2 ");
	//system("darknet.exe detector test data/coco.data yolo.cfg yolo.weights -i 0 -thresh 0.2");
	
	// Retrieve paths to images
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	
	string strFile = string(argv[3]) + "/rgb.txt";
	LoadImages(strFile, vstrImageFilenames, vTimestamps);

	int nImages = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);


	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;






	// Main loop
	cv::Mat im;
	
	for (int ni = 0; ni<nImages; ni++)
	{
		// Read image from file
		im = cv::imread(string(argv[3]) + "/" + vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
		double tframe = vTimestamps[ni];
		vector<string> vstrLabel;
		vector<cv::Mat> vobjPosition;
		if (im.empty())
		{
			cerr << endl << "Failed to load image at: "
				<< string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
			return 1;
		}


		//CLIENT ---------------------------------------------------

		string temp = string(argv[3]) + "\\" + vstrImageFilenames[ni];

		strncpy(sendbuf, temp.c_str(), sizeof(sendbuf));
		sendbuf[sizeof(sendbuf) - 1] = 0;

		iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
		if (iResult == SOCKET_ERROR) {
			printf("send failed with error: %d\n", WSAGetLastError());
			closesocket(ConnectSocket);
			WSACleanup();
			return 1;
		}

		//printf("Bytes Sent: %ld\n", iResult);

		// shutdown the connection since no more data will be sent
		//iResult = shutdown(ConnectSocket, SD_SEND);
		if (iResult == SOCKET_ERROR) {
			printf("shutdown failed with error: %d\n", WSAGetLastError());
			closesocket(ConnectSocket);
			WSACleanup();
			return 1;
		}

		// Receive until the peer closes the connection


		iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
		//if (iResult > 0)
			//printf("Bytes received: %d\n", iResult);
		//if (iResult == 0)
		// printf("Connection closed\n");
		//else
			//printf("recv failed with error: %d\n", WSAGetLastError());


		

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		string yolo = string(recvbuf);

		extract_values(yolo, vstrLabel, vobjPosition);
		//printf("yolo out: %s\n", x.c_str());

		// Pass the image to the SLAM system
		SLAM.TrackMonocular_yolo(im, tframe, vstrLabel, vobjPosition);
		memset(recvbuf, 0, sizeof recvbuf);
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni] = ttrack;
		
		// Wait to load the next frame
		double T = 0;
		if (ni<nImages - 1)
			T = vTimestamps[ni + 1] - tframe;
		else if (ni>0)
			T = tframe - vTimestamps[ni - 1];

		if (ttrack<T)
			usleep((T - ttrack)*1e6);


	

	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	ifstream f;
	f.open(strFile.c_str());

	// skip first three lines
	string s0;
	getline(f, s0);
	getline(f, s0);
	getline(f, s0);

	while (!f.eof())
	{
		string s;
		getline(f, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			string sRGB;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenames.push_back(sRGB);
		}
	}
}



void extract_values(string yolo, vector<string> &vstrLabel, vector<cv::Mat> &vobjPosition) {


	stringstream ss;
	ss.str(yolo);
	string s;
	
	while (getline(ss, s)) {
		string label;
		int p;
		stringstream ss2;
		cv::Mat coord;
		ss2.str(s);
		ss2 >> label;
		vstrLabel.push_back(label);
		coord = cv::Mat::zeros(4, 1, CV_32SC1);
		
		ss2 >> p;
		coord.at<int>(0) = p;		
		ss2 >> p;
		coord.at<int>(1) = p;	
		ss2 >> p;
		coord.at<int>(2) = p;		
		ss2 >> p;
		coord.at<int>(3) = p;
		
		vobjPosition.push_back(coord);
	}
}
