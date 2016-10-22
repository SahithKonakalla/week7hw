#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;

int main(int argc, char* argv[]) {
  std::string fileLocation = argv[1];
  Mat img_rgb = imread(fileLocation);

  Mat img_hsv;
  cvtColor(img_rgb, img_hsv, CV_BGR2HSV);
  std::vector<cv::Mat> channels;
  split(img_hsv, channels);

  Mat hueOrig = channels.at(0).clone();
  Mat threshLower, threshUpper;
  Mat result;

  threshold(hueOrig, threshLower, 75, 255, CV_THRESH_BINARY);
  threshold(hueOrig, threshUpper, 150, 255, CV_THRESH_BINARY_INV);

  result = threshLower & threshUpper;

  Mat edges;
  std::vector<std::vector<Point> > contours;

  Canny(result, edges, 0, 25, 3, false);

  cv::findContours(edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  std::vector<Point> approxContour;

  int rightC;

  for(int i = 0; i < contours.size(); i++) {

    approxPolyDP(contours[i], approxContour, cv::arcLength(cv::Mat(contours.at(i)), true) * 0.02, true);

    if (approxContour.size() == 8 && (cv::contourArea(approxContour, false) > 80)) {

      cv::drawContours(img_rgb, contours, i , Scalar(255,0,0), 5);

      rightC = i;

      std::cout << cv::contourArea(approxContour, false) << "\n";

      imshow("Drawn", img_rgb);

      waitKey(0);
    }
  }

  int xmax;
  int xmin;

  for (int j = 0; j < contours[rightC].size(); j++) {
    int xpoint = contours[rightC][j].x;

    if (j == 0){
      xmax = xpoint;
      xmin = xpoint;
    }

    if (xpoint > xmax) {
      xmax = xpoint;
    }

    if (xpoint < xmin) {
      xmin = xpoint;
    }
    // std::cout << "xmin = " << xmin << "\n";
    // std::cout << "xmax = " << xmax << "\n";
    // std::cout << "xpoint = " << xpoint << "\n";
  }

  int pixel = xmax - xmin;
  float distance = (480 * 0.1)/ pixel;
  // std::cout << "pixel = " << pixel << "\n";
  std::cout << "distance = " << distance << "\n";

  float rectCenter = (xmax + xmin)/2;
  float azimuth = (atan((rectCenter - 240)/distance))*180/3.141592;
  std::cout << "azimuth = " << azimuth << "\n";
}
