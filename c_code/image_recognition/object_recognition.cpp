#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <lccv.hpp>
#include <iostream>

#include <chrono>
#include <unistd.h>

#define CAMERA_HEIGHT 720       // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 1280       // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 1000 // heheh    // If fps higher than what the thread can handle, it will just run lower fps.


// From https://docs.opencv.org/4.x/d2/d0a/tutorial_introduction_to_tracker.html
// https://learnopencv.com/object-tracking-using-opencv-cpp-python/

int main(int argc, char** argv) {
    cv::Rect bbox;
    cv::Mat image;

    // cv::Ptr<cv::Tracker> tracker = cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerMOSSE::create());
    // cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();
    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();
    lccv::PiCamera cam;

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;

    cam.startVideo();

    sleep(1);

    cam.getVideoFrame(image,  1000);

    bbox = cv::selectROI("tracker", image);

    std::cout << "bbox width: " << bbox.width << ", bbox heigth: " << bbox.height << std::endl;

    if(bbox.width == 0 || bbox.height == 0) {
        std::cout << "No ROI selected" << std::endl;
        sleep(1);
        cam.stopVideo();
        return 0;
    }

    // initialize the tracker
    tracker->init(image, bbox);

    std::cout << "initialized tracker" << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();

    while(true) {
        if(!cam.getVideoFrame(image, 1000)) {  // Define which mat to load the image into, and the timeout in ms
            std::cout << "Timeout error" << std::endl;
        }
        else {
            // stop the program if no more images
            if(image.rows == 0 || image.cols == 0)
                break;

            double timer = (double)cv::getTickCount();
            
            // update the tracking result
            if (tracker->update(image, bbox)) {
                // draw the tracked object
                cv::rectangle(image, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
            }
            else {
                cv::putText(image, "Tracking failure detected", cv::Point(100,80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
            }

            float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);
            cv::putText(image, "FPS : " + std::to_string(int(fps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);
        
            // show image with the tracked object
            cv::imshow("tracker",image);
        
            //quit on ESC button
            if(cv::waitKey(1)=='q') break;
        }
    }

    cam.stopVideo();
    cv::destroyAllWindows();
    std::cout << "End of file" << std::endl;
    return 0;
}