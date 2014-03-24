#include <dmtx.h>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <datamatrix_finder/Datamatrix.h>

using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher datamatrix_pub;
float datamatrixSize = 0.0f;
    
struct DataMatrixInfo {
  std::string message;
  cv::Mat transformation;
};

DataMatrixInfo decode(cv::Mat image, const CameraInfoConstPtr &cam_info, int timeout) {
  DataMatrixInfo dataMatrixInfo;
  dataMatrixInfo.message = "";
  dataMatrixInfo.transformation = cv::Mat::eye(4, 4, CV_32FC1);
  DmtxImage *img = NULL;
  DmtxDecode *dec = NULL;
  DmtxRegion *reg = NULL;
  DmtxMessage *msg = NULL;
  DmtxTime t;
	
  img = dmtxImageCreate(image.data, image.cols, image.rows, DmtxPack24bppBGR);
	
  dec = dmtxDecodeCreate(img, 1);
  assert(dec != NULL);
	
  t = dmtxTimeAdd(dmtxTimeNow(), timeout);
  reg = dmtxRegionFindNext(dec, &t);
	
  if (reg != NULL) {
    DmtxVector2 p00, p10, p11, p01;
    int height = dmtxDecodeGetProp(dec, DmtxPropHeight);
		
    p00.X = p00.Y = p01.X = p10.Y = 0.0;
    p01.Y = p10.X = p11.X = p11.Y = 1.0;
		
    msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
		
    dmtxMatrix3VMultiplyBy(&p00, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p10, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p11, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p01, reg->fit2raw);
		
    cv::Point2f corner1(p00.X, image.rows - p00.Y), corner2(p10.X, image.rows - p10.Y);
    cv::Point2f corner3(p11.X, image.rows - p11.Y), corner4(p01.X, image.rows - p01.Y);
    std::vector<cv::Point2f> datamatrixPixelVector;
    datamatrixPixelVector.push_back(corner1);
    datamatrixPixelVector.push_back(corner2);
    datamatrixPixelVector.push_back(corner4);
    datamatrixPixelVector.push_back(corner3);
    cv::Point3f originPoint(0.0f, 0.0f, 0.0f), xPoint(datamatrixSize, 0.0f, 0.0f);
    cv::Point3f yPoint(0.0f, datamatrixSize, 0.0f), xyPoint(datamatrixSize, datamatrixSize, 0.0f);
    std::vector<cv::Point3f> datamatrixPointVector;
    datamatrixPointVector.push_back(originPoint);
    datamatrixPointVector.push_back(xPoint);
    datamatrixPointVector.push_back(yPoint);
    datamatrixPointVector.push_back(xyPoint);
    
    cv::Mat rotationVector, translationVector;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
    for(int r = 0; r < cameraMatrix.rows; r++) {
      for(int c = 0; c < cameraMatrix.cols; c++) {
	cameraMatrix.at<float>(r, c) = (float)cam_info->K[r * cameraMatrix.rows + c];
      }
    }
    cv::solvePnP(datamatrixPointVector, datamatrixPixelVector, cameraMatrix, cam_info->D, rotationVector, translationVector);
    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_32FC1);
    
    cv::Rodrigues(rotationVector, rotationMatrix);

    cv::line(image, corner1, corner2, cv::Scalar(0, 255, 0), 3);
    cv::line(image, corner2, corner3, cv::Scalar(255, 0, 0), 3);
    cv::line(image, corner3, corner4, cv::Scalar(0, 0, 255), 3);
    cv::line(image, corner4, corner1, cv::Scalar(255, 255, 0), 3);

    for(int r = 0; r < rotationMatrix.rows; r++) {
      for(int c = 0; c < rotationMatrix.cols; c++) {
	dataMatrixInfo.transformation.at<float>(r, c) = (float)rotationMatrix.at<double>(r, c);
      }
    }
    dataMatrixInfo.transformation.at<float>(0, 3) = (float)translationVector.at<double>(0, 0);
    dataMatrixInfo.transformation.at<float>(1, 3) = (float)translationVector.at<double>(1, 0);
    dataMatrixInfo.transformation.at<float>(2, 3) = (float)translationVector.at<double>(2, 0);

    if (msg != NULL) {
      dataMatrixInfo.message.assign((const char*) msg->output, msg->outputIdx);
      dmtxMessageDestroy(&msg);      
    }
		
    dmtxRegionDestroy(&reg);
  }
	
  dmtxDecodeDestroy(&dec);
  dmtxImageDestroy(&img);

  cv::imshow("Image window", image);
  cv::waitKey(1);
	
  return dataMatrixInfo;
}

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info) {
  CvImagePtr cvRgbImage = toCvCopy(image, "bgr8");
  DataMatrixInfo dataMatrixInfo = decode(cvRgbImage->image, cam_info, 100);
	
  if(!dataMatrixInfo.message.empty()) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(dataMatrixInfo.transformation.at<float>(0, 3),
				    dataMatrixInfo.transformation.at<float>(1, 3),
				    dataMatrixInfo.transformation.at<float>(2, 3)));
    Eigen::Quaternionf quaternion((float*)dataMatrixInfo.transformation(cv::Rect(0, 0, 3, 3)).data);
    quaternion.normalize();
    transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "datamatrix_frame"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tower_cam3d_rgb_optical_frame", "datamatrix_frame"));
    datamatrix_finder::Datamatrix msg;
    msg.message = dataMatrixInfo.message;
    for(int i = 0; i < msg.translation.size(); i++) {
      msg.translation[i] = dataMatrixInfo.transformation.at<float>(i, 3);
    }
    for(int r = 0; r < dataMatrixInfo.transformation.rows - 1; r++) {
      for(int c = 0; c < dataMatrixInfo.transformation.cols - 1; c++) {
	msg.rotation[r * (dataMatrixInfo.transformation.rows - 1) + c] = dataMatrixInfo.transformation.at<float>(r, c);
      }
    }
    datamatrix_pub.publish(msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "datamatrix_finder_node_sim");

  if(argc < 2) {
    std::cerr << "Usage: " << std::endl;
    std::cerr << "rosrun datamatrix_finder datamatrix_finder_node datamatrixSize" << std::endl;
    return 1;
  }

  datamatrixSize = std::atof(argv[1]);
  std::cout << "Datamatrix size: " << datamatrixSize << std::endl;

  ros::NodeHandle nh;

  datamatrix_pub = nh.advertise<datamatrix_finder::Datamatrix>("datamatrix", 1000);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 1);
  //message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/tower_cam3d/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/rgb/camera_info", 1);
  //message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/tower_cam3d/rgb/camera_info", 1);
  typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
