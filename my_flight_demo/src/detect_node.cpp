#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include "my_flight_demo/common.h"

#define DebugP(x) std::cout << "Line" << __LINE__ << "  " << #x << "=" << x << std::endl

void readCameraParameters(cv::Mat &cameraMatrix,cv::Mat &distCoeffs)
{
    cameraMatrix=(cv::Mat_<float>(3,3)<<3.495223076908601e+02,  0,  3.362714348648687e+02,
                                        0,                  3.494326644909979e+02,   2.156009107951729e+02,
                                        0,0,1);
    distCoeffs=(cv::Mat_<float>(4,1)<<7.3373e-05, -0.0312,-0.001,2.5029e-04);
}

void CodeRotateByZ(float x, float y, float thetaz, float &outx, float &outy)
{
  float x1 = x; //将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
  float y1 = y;
  float rz = thetaz * M_PI / 180;
  outx = cos(rz) * x1 - sin(rz) * y1;
  outy = sin(rz) * x1 + cos(rz) * y1;
}

void CodeRotateByY(float x, float z, float thetay, float &outx, float &outz)
{
  float x1 = x;
  float z1 = z;
  float ry = thetay * M_PI / 180;
  outx = cos(ry) * x1 + sin(ry) * z1;
  outz = cos(ry) * z1 - sin(ry) * x1;
}
void CodeRotateByX(float y, float z, float thetax, float &outy, float &outz)
{
  float y1 = y; //将变量拷贝一次，保证&y == &y这种情况下也能计算正确
  float z1 = z;
  float rx = thetax * M_PI / 180;
  outy = cos(rx) * y1 - sin(rx) * z1;
  outz = cos(rx) * z1 + sin(rx) * y1;
}


// transform body frame to marker frame
std::vector<float> transform(cv::Vec3d &rvec,cv::Vec3d &tvec)
{
  // DebugP(rvec);
  float r1=rvec[0];
  float r2=rvec[1];
  float r3=rvec[2];
  Eigen::Matrix3f RoteM=Rodrigues(r1,r2,r3);
  // DebugP(rvec[1]);
  // DebugP(rvec[2]);
  //  float rm[9];
    // cv::Mat RoteM(3,3,CV_32F,rm);
    // cv::Rodrigues(rvec,RoteM);

    float r11 = RoteM(0,0);
    float r12 = RoteM(0,1);
    float r13 = RoteM(0,2);
    float r21 = RoteM(1,0);
    float r22 = RoteM(1,1); 
    float r23 = RoteM(1,2);
    float r31 = RoteM(2,0);
    float r32 = RoteM(2,1);
    float r33 = RoteM(2,2);
    // DebugP(r11);
    // DebugP(r12);
    // DebugP(r13);
    // DebugP(r21);
    // DebugP(r22);
    // DebugP(r23);
    // DebugP(r31);
    // DebugP(r32);
    // DebugP(r33);

    // DebugP(RoteM);
    // float thetaz = atan(r21, r11);
    // float thetay = atan(-1 * r31, sqrt(r32 * r32 + r33 * r33));
    // float thetax = atan(r32, r33);
    float thetaz = atan2(r21, r11) / M_PI * 180;
    float thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / M_PI * 180;
    float thetax = atan2(r32, r33) / M_PI * 180;
    // DebugP(thetaz);
    // DebugP(thetay);
    // DebugP(thetax);
    float x = tvec[0];
    float y = tvec[1];
    float z = tvec[2];
    // DebugP(x);
    // DebugP(y);
    // DebugP(z);

    CodeRotateByZ(x, y, -1 * thetaz, x, y);
    CodeRotateByY(x, z, -1 * thetay, x, z);
    CodeRotateByX(y, z, -1 * thetax, y, z);

    x=x*-1;
    y=y*-1;
    z=z*-1;
    // DebugP(x);
    // DebugP(y);
    // DebugP(z);
    std::vector<float> position;
    position.push_back(x);
    position.push_back(y);
    position.push_back(z);

    return position;
    
}

// std::string get_tegra_pipeline(int width, int height, int fps) {
//     return "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
//             std::to_string(height) + ", format=(string)I420, framerate=(fraction)" + std::to_string(fps) +
//             "/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
// }

ros::Publisher detectPub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_detect_node");
  ros::NodeHandle nh;
  detectPub = nh.advertise<my_flight_demo::Visual_msg>("visual", 20);
  // std::string pipeline = get_tegra_pipeline(640, 480, 30);
  cv::VideoCapture cap;
  // std::string pipeline = "appsrc ! floatvideoconvert ! omxh265enc ! matroskamux ! filesink    
  //   location=test.mkv ";
  // cv::VideoWriter("appsrc ! video/x-raw,width=320,height=240,framerate=15/1 ! videoconvert ! video/x-raw,format=I420 ! shmsink socket-path=/tmp/testsocket", cv::CAP_GSTREAMER, 30, (640, 480))
  cap.open(1);
  //cap.set(6,cv::VideoWriter::fourcc('M','J','P','G'));
  // cap.set(5,100);
  cap.set(3,640);
  cap.set(4,480);
  cv::Mat cameraMatrix, distCoeffs;
  // camera parameters are read from somewhere
  readCameraParameters(cameraMatrix, distCoeffs);
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  ros::Rate loop_rate(10);
  while (cap.grab()) {
      cv::Mat image, imageCopy;
      cap.retrieve(image);
      image.copyTo(imageCopy);
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(image, dictionary, corners, ids);
      // if at least one marker detected
      if (ids.size() > 0) {
          cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
          std::vector<cv::Vec3d> rvecs, tvecs;
          cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
          // draw axis for each marker
          for(int i=0; i<ids.size(); i++){
              cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.025);
              std::vector<float> position=transform(rvecs[i],tvecs[i]);  
              std::cout<<"x:"<<position[0]<<" y:"<<position[1]<<" z:"<<position[2]<<std::endl;
              my_flight_demo::Visual_msg msg;
              geometry_msgs::Vector3 tvec;
              geometry_msgs::Vector3 rvec;
              tvec.x=position[0];
              tvec.y=position[1];
              tvec.z=position[2];
              rvec.x=rvecs[i][0];
              rvec.y=rvecs[i][1];
              rvec.z=rvecs[i][2];
              msg.id=ids[i];
              msg.tvec=tvec;
              msg.rvec=rvec;
              detectPub.publish(msg);
              ros::spinOnce();
              loop_rate.sleep();
          }
      }
      else{
        my_flight_demo::Visual_msg msg;
        geometry_msgs::Vector3 tvec;
        geometry_msgs::Vector3 rvec;
        tvec.x=0;
        tvec.y=0;
        tvec.z=0;
        rvec.x=0;
        rvec.y=0;
        rvec.z=0;
        msg.id=100;
        msg.tvec=tvec;
        msg.rvec=rvec;
        detectPub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
      }
      cv::imshow("out", imageCopy);
      char key = (char) cv::waitKey(1);
      if (key == 27)
          break;
  }
  return 0;
}