// k4a.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include "pch.h"

#include "AzureKinect.h"

#include <iostream>
#include <sstream>

#include <windows.h>

#include <opencv2\opencv.hpp>

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl\io\ply_io.h>

#pragma warning(suppress : 4996)

class App
{
public: 

	std::vector<AzureKinect::Sensor> kinect;
	//std::vector<AzureKinect::Transformation> transformation;

	~App()
	{
		for (auto& k : kinect) {
			k.stopImu();
			k.stopCamera();
		}
	}

	void initialize()
	{
		int count = AzureKinect::Sensor::deviceCouunt();
		std::cout << "Kinect Count : " << count << std::endl;
		if (count == 0) {
			throw std::runtime_error("Kinectが接続されていません");
		}

		kinect.resize(count);
		//transformation.resize(count);

		auto getDefaultConfig = []()
		{
			// パラメーターは全部設定しないとエラーになる
			k4a_device_configuration_t config;
			config.camera_fps = K4A_FRAMES_PER_SECOND_30;
			config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
			config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
			config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
			config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
			config.synchronized_images_only = true;
			config.depth_delay_off_color_usec = 0;
			config.subordinate_delay_off_master_usec = 0;
			config.disable_streaming_indicator = false;

			return config;
		};

		AzureKinect::Sensor* master = 0;
		for (int i = 0; i < kinect.size(); ++i) {
			std::cout << "Open Kinect " << i << std::endl;
			kinect[i].open(i);
			auto jack = kinect[i].getJackState();
			if (jack.in) {
				std::cout << "sub " << kinect[i].gtSerialNumber() << std::endl;

				auto config = getDefaultConfig();
				config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
				kinect[i].startCamera(&config);
				kinect[i].startImu();
			}
			else {
				master = &kinect[i];
			}

		}

		auto config = getDefaultConfig();
		//config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
		std::cout << "master " << master->gtSerialNumber() << std::endl;
		master->startCamera(&config);
		master->startImu();
	}

	void run()
	{

		//pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

		std::cout << kinect.size() << std::endl;

		while(1) {
			for (int i = 0; i < kinect.size(); ++i) {
				//::Sleep(100);

				AzureKinect::Capture capture;
				auto ret = capture.Open(kinect[i], 0);
				if (!ret) {
					continue;
				}

				// データはカラーのフォーマットによる
				auto colorImage = capture.getColorImage();
				cv::Mat color(colorImage.getHeight(), colorImage.getWidth(), CV_8UC4, colorImage.getBuffer());
				cv::resize(color, color, cv::Size(), 0.3f, 0.3f);
				{
					std::stringstream ss;
					ss << "Color " << i;
					cv::imshow(ss.str(), color);
				}

				// ミリ単位(WFOV 2x2 binned の場合 2880 になる)
				auto depthImage = capture.getDepthImage();
				cv::Mat depth(depthImage.getHeight(), depthImage.getWidth(), CV_16U, depthImage.getBuffer());
				depth.convertTo(depth, CV_8U, 255.0 / 3000);
				{
					std::stringstream ss;
					ss << "Depth " << i;
					cv::imshow(ss.str(), depth);
				}

				// 0-255の範囲(データは16bit)
				auto irImage = capture.getIrImage();
				cv::Mat ir(irImage.getHeight(), irImage.getWidth(), CV_16U, irImage.getBuffer());
				ir.convertTo(ir, CV_8U, 1);
				{
					std::stringstream ss;
					ss << "IR " << i;
					cv::imshow(ss.str(), ir);
				}

				auto imu = kinect[i].getImuSample();

				std::cout << "Kinect " << i << " " << colorImage.getTimestamp() << " " << depthImage.getTimestamp() << " " << irImage.getTimestamp() <<
					" " << imu.acc_timestamp_usec << " " << imu.acc_sample.xyz.x << " " << imu.acc_sample.xyz.y << " " << imu.acc_sample.xyz.z << std::endl;


				// Depth座標系のカラーデータ
				//auto depthColorImage = transformation[i].ColorToDepth(colorImage, depthImage);
				//cv::Mat depthColor(depthColorImage.getHeight(), depthColorImage.getWidth(), CV_8UC4, depthColorImage.getBuffer());
				//{
				//	std::stringstream ss;
				//	ss << "ColorToDepth " << i;
				//	cv::imshow(ss.str(), depthColor);
				//}

				// 点群データ
				//auto pointCloud = transformation[0].DepthToPointCloud(depthImage);

				// 点群
				//{
				//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				//	auto count = depthImage.getWidth() * depthImage.getHeight();
				//	uint8_t* colorBuffer = (uint8_t*)depthColorImage.getBuffer();
				//	int16_t* pointBuffer = (int16_t*)pointCloud.getBuffer();
				//	for (int i = 0; i < count; ++i) {
				//		pcl::PointXYZRGB point;

				//		int colorIndex = i * 4;
				//		point.b = colorBuffer[colorIndex + 0];
				//		point.g = colorBuffer[colorIndex + 1];
				//		point.r = colorBuffer[colorIndex + 2];

				//		int pointIndex = i * 3;
				//		point.x = pointBuffer[pointIndex + 0];
				//		point.y = pointBuffer[pointIndex + 1];
				//		point.z = pointBuffer[pointIndex + 2];
				//		cloud->push_back(point);
				//	}

				//	// 点群を更新する
				//	std::stringstream ss;
				//	ss << "cloud " << i;

				//	auto ret = viewer.updatePointCloud(cloud, ss.str());
				//	if (!ret) {
				//		// 更新がエラーになった場合は、
				//		// 未作成なので新しい点群として追加する
				//		viewer.addPointCloud(cloud, ss.str());
				//	}

				//	viewer.spinOnce();
				//}
			}



			int key = cv::waitKey(1);
			//int key = cv::waitKey(-1);
			if (key == 'q') {
				break;
			}
			//else if (key == 's') {
			//	pcl::PLYWriter writer;
			//	writer.write("cloud.ply", *cloud, true, false);
			//	std::cout << "save ply" << std::endl;
			//}
		}
	}
};

int main()
{
	try
	{
		App app;
		app.initialize();
		app.run();

		std::cout << "Success." << std::endl;
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}

}

// プログラムの実行: Ctrl + F5 または [デバッグ] > [デバッグなしで開始] メニュー
// プログラムのデバッグ: F5 または [デバッグ] > [デバッグの開始] メニュー

// 作業を開始するためのヒント: 
//    1. ソリューション エクスプローラー ウィンドウを使用してファイルを追加/管理します 
//   2. チーム エクスプローラー ウィンドウを使用してソース管理に接続します
//   3. 出力ウィンドウを使用して、ビルド出力とその他のメッセージを表示します
//   4. エラー一覧ウィンドウを使用してエラーを表示します
//   5. [プロジェクト] > [新しい項目の追加] と移動して新しいコード ファイルを作成するか、[プロジェクト] > [既存の項目の追加] と移動して既存のコード ファイルをプロジェクトに追加します
//   6. 後ほどこのプロジェクトを再び開く場合、[ファイル] > [開く] > [プロジェクト] と移動して .sln ファイルを選択します
