#include <stdio.h>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include "ShowManyImages.hpp"
#include "ShowManyImages.cpp"
#include "blobs.hpp"
#include "blobs.cpp"
#include "fgseg.hpp"
#include "fgseg.cpp"
#include "kalman.hpp"
#include "kalman.cpp"
#include "vsrc.hpp"
#include "vsrc.cpp"

#include "lccv.hpp"

using namespace cv;
using namespace std;

#define CAMERA_HEIGHT 720	 // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 1280	 // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.

int main(int argc, char **argv)
{
	//**********************************************************
	// Debug detailed mode -> acctiaves all the 'cout' code lines.
	bool bFullDebugOn = true;
	//**********************************************************

	//**********************************************************
	// PARAMETRIZATION: DATASET
	//
	// We initialize the Video source (0, 1, 2, 3 or 4)
	// 0 -> takes videos from the arguments of main (./main url1)
	// 1-4 -> videos from 4 datasets defined in 'vsrc.hpp'
	//**********************************************************
	int iVideoMode = 0;

	vidsrc::vsrc oSRC(iVideoMode);
	int iNumCategories;
	int iNumSequences;
	string sVideoPath;

	if (iVideoMode > 0)
	{
		iNumCategories = oSRC.GetNumCategs();
		iNumSequences = oSRC.GetNumSeqs();
	}
	else
	{
		iNumCategories = argc - 1;
		iNumSequences = 1;
		sVideoPath = argv[1];
	}


	cv::namedWindow("fgmask", cv::WINDOW_NORMAL);
	cv::namedWindow("Blobs", cv::WINDOW_NORMAL);


	//**********************************************************

	//**********************************************************
	// PARAMETRIZATION: FOREGROUND SEGM. AND BLOB ANALYSIS
	//
	// We initialize the Foreground Segementation object
	// to use MOG2 and morphological operation OPENING.
	// Arg1 = 0 -> MOG2
	// Arg2 = 1 -> OPENING
	//		= 2 -> OPENING + CLOSING
	//		= 3 -> OPENING x 4
	//
	// learningrate: between 0-1, for MOG2
	// structuringElement_CCA: for Connected Component Analysis
	// min_width and min_height for Blob extraction
	//**********************************************************
	fgseg::bgs oBGS(0, 3);

	double learningrate = 0.01; // Test 3.1
	// double learningrate = 0.001;//Test 3.2
	// double learningrate = 0.0001;//Test 3.3
	int structuringElement_CCA = 4; // 4 or 8
	int min_width = 15;				// Test 3.1
	int min_height = 15;			// Test 3.1
	// int min_width = 55;//Test 3.2
	// int min_height = 55;//Test 3.2
	int max_width = 150;  // Test 3.2
	int max_height = 150; // Test 3.2
	// int min_width = 15; //Test 3.3_1
	// int min_height = 29; //Test 3.3_1
	// int min_width = 30; //Test 3.3_2
	// int min_height = 65; //Test 3.3_2
	// int min_width = 25; //Test 3.3_3
	// int min_height = 55; //Test 3.3_3
	// int min_width = 30; //Test 3.3_4
	// int min_height = 30; //Test 3.3_4
	//**********************************************************

	//**********************************************************
	// PARAMETRIZATION: KALMAN INIT
	//**********************************************************
	// First parameter sets the predefenied values for:
	// 1 -> Constant Velocity Model
	// 2 -> Accelaration Model
	//**********************************************************
	int iKalmanMode = 1;
	//**********************************************************

	Mat frame;							  // current Frame
	Mat fgmask;							  // foreground mask
	std::vector<cvBlob> bloblist;		  // list for blobs
	std::vector<cvBlob> bloblistFiltered; // list for blobs
	cv::Rect rectBall;
	cv::Rect rectKBall;
	Point ball;
	Point kalmanBall;
	Scalar K_corr_Color(0, 0, 255);
	Scalar K_pred_Color(0, 255, 0);
	Scalar GT_Color(255, 0, 0);
	int iTextOffsetCorrection_1 = 15; // PutText function has a small offset in y axis depending on the simbol you print (* or _)
	int iTextOffsetCorrection_2 = 7;

	int image_count = 0;

	lccv::PiCamera cam(0);
		
	cam.options->video_width = CAMERA_WIDTH;
	cam.options->video_height = CAMERA_HEIGHT;
	cam.options->framerate = CAMERA_FRAMERATE;
	// cam.options->verbose = true;
	


	// Loop for all categories
	for (int c = 0; c < iNumCategories; c++)
	{
		// Loop for all sequence of each category
		for (int s = 0; s < iNumSequences; s++)
		{
			//**********************************************************
			// KALMAN TUNING
			//**********************************************************
			kalmantracking::kalman oKF(iKalmanMode, bFullDebugOn);

			// Mat R;
			// R = Mat::zeros(Size(2, 2), CV_32F);
			// cv::setIdentity(R);
			// R.at<float>(0) = 100.0f;
			// R.at<float>(3) = 100.0f;
			// if (bFullDebugOn) cout << "R = " << endl << " " << R << endl << endl;

			// oKF.SetUncertaintyMatrix(R);
			//**********************************************************

			int it = 1;
			int iFrameNum = 0;

			cam.startVideo();

			while (true)
			{

				if (!cam.getVideoFrame(frame, 1000))
				{
					std::cerr << "Timeout error" << std::endl;
				}
				else
				{

					// frame = frame(cv::Rect(cv::Point(300, 0), cv::Point(700, 717)));
					iFrameNum += 1;

					// check if we achieved the end of the file
					// if (!frame.data)
					// 	continue;

					//**********************************************************
					// FOREGROUND DETECTION
					// 0 bkg, 255 fg, 127 (gray) shadows ...
					//**********************************************************
					fgmask = oBGS.getFGmask(frame, learningrate);
					fgmask = oBGS.removeShadows_127(fgmask);

					//**********************************************************
					// BLOB EXTRACTION
					//**********************************************************
					// Extract the blobs in fgmask
					extractBlobs(fgmask, bloblist, structuringElement_CCA);
					//		cout << "Num blobs extracted=" << bloblist.size() << endl;
					removeSmallBlobs(bloblist, bloblistFiltered, min_width, min_height);
					// removeBigBlobs(bloblistFiltered, bloblistFiltered, max_width, max_height);
					//		cout << "Num small blobs removed=" << bloblist.size()-bloblistFiltered.size() << endl;

					ball = GetBallCoordinates(bloblistFiltered);

					//**********************************************************
					// KALMAN TRACKING
					//**********************************************************
					//
					// ball is the center of the object, parameter for Kalman
					kalmanBall = oKF.Predict(ball);

					//**********************************************************
					// RESULTS VISUALIZATION
					//**********************************************************
					//
					int iTextSize1 = 1;		// Test 3.1 and 3.3
					int iTextSize2 = 1.5;	// Test 3.1 and 3.3
					int iTextThickness = 1; // Test 3.1 and 3.3
					int iTestsOffset = 20;	// Test 3.1 and 3.3
					// int iTextSize1 = 1.8;//Test 3.2
					// int iTextSize2 = 2.5;//Test 3.2
					// int iTextThickness = 3;//Test 3.2
					// int iTestsOffset = 400;//Test 3.2


					// Point p1 = Point(kalmanBall.x-10, kalmanBall.y-10); //Test 3.3
					// Point p2 = Point(kalmanBall.x + 10, kalmanBall.y + 10); //Test 3.3
					Point p1 = Point(kalmanBall.x - 10, kalmanBall.y - 10); // Test 3.3
					Point p2 = Point(kalmanBall.x + 10, kalmanBall.y + 10); // Test 3.3

					if (oKF.getStatus() == "Predicted")
					{
						rectangle(frame, p1, p2, K_pred_Color, 1, 8, 0);
					}
					else
					{
						rectangle(frame, p1, p2, K_corr_Color, 1, 8, 0);
					}

					for (int i = 0; i < oKF.getGroundTruthList().size(); i++)
					{
						putText(frame, "o", Point(oKF.getGroundTruthList()[i].x, oKF.getGroundTruthList()[i].y + iTextOffsetCorrection_2), FONT_HERSHEY_COMPLEX, iTextSize2, GT_Color, iTextThickness);
					}
					for (int i = 0; i < oKF.getPredictedList().size(); i++)
					{
						putText(frame, "*", Point(oKF.getPredictedList()[i].x, oKF.getPredictedList()[i].y + iTextOffsetCorrection_1), FONT_HERSHEY_COMPLEX, iTextSize2, K_pred_Color, iTextThickness);
					}
					for (int i = 0; i < oKF.getCorrectedList().size(); i++)
					{
						putText(frame, "*", Point(oKF.getCorrectedList()[i].x, oKF.getCorrectedList()[i].y + iTextOffsetCorrection_1), FONT_HERSHEY_COMPLEX, iTextSize2, K_corr_Color, iTextThickness);
					}

					if (bFullDebugOn)
						cout << "Ball.x =  " << ball.x << endl
							 << "Ball.y =  " << ball.y << endl;
					if (bFullDebugOn)
						cout << "KBall.x = " << kalmanBall.x << endl
							 << "KBall.y = " << kalmanBall.y << endl;

					// ShowManyImages("AVSA Lab 3", 6, frame, fgmask, fgmask,
					//	paintBlobImage(frame, bloblistFiltered, false), paintBlobImage(frame, bloblistFiltered, true), paintBlobImage(frame, bloblistFiltered, true));
					// ShowManyImages("AVSA Lab 3", 2, , fgmask);
					// cv::rotate(fgmask, fgmask, cv::ROTATE_90_COUNTERCLOCKWISE);
					// cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);

					// putText(frame,"Ground Truth (ball)", Point(10,15+ iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, Scalar(255, 255, 255), 1);//Test 3.1 and 3.3
					putText(frame, "Ground Truth (ball)", Point(10, 15 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, GT_Color, iTextThickness);   // Test 3.1 and 3.3
					putText(frame, "Kalman Prediction", Point(10, 40 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_pred_Color, iTextThickness); // Test 3.1 and 3.3
					putText(frame, "Kalman Corrected", Point(10, 65 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_corr_Color, iTextThickness);  // Test 3.1 and 3.3

					// putText(frame, "Ground Truth (ball)", Point(10, 50 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, GT_Color, iTextThickness);//Test 3.2
					// putText(frame, "Kalman Prediction", Point(10, 100 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_pred_Color, iTextThickness);//Test 3.2
					// putText(frame, "Kalman Corrected", Point(10, 150 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_corr_Color, iTextThickness);//Test 3.2

					// putText(frame, "Frame " + to_string(iFrameNum), Point(10, 200 + iTestsOffset), FONT_HERSHEY_COMPLEX, iTextSize1, Scalar(255,255,255), iTextThickness);//Test 3.2
					putText(frame, "Frame " + to_string(iFrameNum), Point(10, 90 + iTestsOffset), FONT_HERSHEY_COMPLEX, iTextSize1, Scalar(255, 255, 255), iTextThickness); // Test 3.2 and 3.3

					cv::imshow("fgmask", fgmask);
					cv::imshow("Blobs", paintBlobImage(frame, bloblistFiltered, true));

					if (cv::waitKey(1) == 32)
					{
						std::string filename = "ANDERS_yippi" + std::to_string(image_count++) + ".jpg";
						cv::imwrite(filename, frame);
						std::cout << "Saved frame: " << filename << std::endl;
					}

					// exit if ESC key is pressed
					if (waitKey(30) == 113)
						break;
					it++;
				}
			}

			cam.stopVideo();
			destroyAllWindows();
			waitKey(0);
		}
	}
	return 0;
}
