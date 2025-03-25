#include <iostream>

// For the image recognition and
#include <opencv2/opencv.hpp>
// A interface between the camera and OpenCV
#include <lccv.hpp>

// Sources
// https://github.com/Sleddersquid/fussball_system/blob/main/c_code/main.cpp
// https://github.com/hcglhcgl/BallDetection/blob/master/balls.cpp


// https://docs.opencv.org/3.4/d6/d6e/group__imgproc__draw.html why is this here?

#define CAMERA_HEIGHT 720    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 1280    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.

/**
 * @brief Function to calculate the center of a contour using moments
 * @param contour - The contour to calculate the center of
 * @return The center of the contour as cv::Point
 */
cv::Point calculateCenter(const std::vector<cv::Point> &contour) {
  cv::Moments M = cv::moments(contour);
  if (M.m00 != 0) {
      // See https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html for center of mass (x¯, y¯)
      return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
  }
  return cv::Point(0, 0); // If no center is found, return (0, 0)
}

int main() {

  std::cout << "Press ESC to stop. (Does not work if no window is displayed)" << std::endl;

  cv::Mat image, mask, HSV;
  lccv::PiCamera cam;

  // Was (0, 120, 120) and (10, 255, 255).
  // Lightings conditions such as sunlight might detect hands and face 
  cv::Scalar hsv_lower(0, 120, 120); // 0, 150, 50
  cv::Scalar hsv_upper(10, 255, 200);  // 15, 255, 255

  cam.options->video_width = CAMERA_WIDTH;
  cam.options->video_height = CAMERA_HEIGHT;
  cam.options->framerate = CAMERA_FRAMERATE;
  cam.options->verbose = true;
  // cam.options->list_cameras = true;

  cv::namedWindow("Video", cv::WINDOW_NORMAL);
  cv::namedWindow("Mask", cv::WINDOW_NORMAL);
  
  cam.startVideo();

  cv::Point new_center(0, 0);
  // cv::Point old_center(0, 0);

  while (true) {
    if (!cam.getVideoFrame(image, 1000)) {
      std::cerr << "Timeout error" << std::endl;
    } 
    else {
      // This is from the python file "test_opencv.py" and https://github.com/hcglhcgl/BallDetection/blob/master/balls.cpp
      cv::cvtColor(image, HSV, cv::COLOR_BGR2HSV);

      cv::inRange(HSV, hsv_lower, hsv_upper, mask);

      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);
      cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      if (!contours.empty()) {
        std::vector<cv::Point> largestContour = *std::max_element(contours.begin(), contours.end(), 
                                                [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                                                    return cv::contourArea(a) < cv::contourArea(b);
                                                }); // Lambda function. Finds the largest contour

        new_center = calculateCenter(largestContour);
      }

      if (new_center.x == 0 && new_center.y == 0) {
        continue;
      }

      std::cout << "New center.x: " << new_center.x << " New center.y: " << new_center.y << std::endl;

      cv::Mat mask_color;
      image.copyTo(mask_color, mask);

      cv::circle(image, new_center, 5, cv::Scalar(0, 0, 255), -1); // Draw on mask, at point new_center,  

      // image is the Image from the camera (stream)
      cv::imshow("Video", image);
      // mask is a black and white image from the InRange function
      cv::imshow("Mask", mask);
      // Think of mask_color as the mask, but with the color of the object
      cv::imshow("Mask with color", mask_color);

      // // quit on q button
      if (cv::waitKey(1) == 'q') {
        break;
      }

      // old_center = new_center;
    }
  }

  cam.stopVideo();
  cv::destroyWindow("Video");
  cv::destroyWindow("Mask");
  cv::destroyAllWindows();
  std::cout << "End of opencv" << std::endl;

  std::cout << "End of file!" << std::endl;
  return 0;
}
