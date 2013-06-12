#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <opencv2/video/background_segm.hpp>
#include <vector>
namespace enc = sensor_msgs::image_encodings;
using namespace sensor_msgs;
using namespace message_filters;


cv::Mat pre, prede;
float alpha = 0.98;
int morph_size = 2;
float fx=0;
float fy=0;
float cx=0;
float cy=0;

// RVIZ colours
cv::Mat image2depthVis(const cv::Mat& d, const cv::Mat& rgb, float range_min=0, float range_max=5){
    cv::Mat out = cv::Mat::zeros(d.size(),CV_8UC3);
    for(int row =0; row< d.rows;++row){
        for (int col=0; col < d.cols; ++col){
            float depth = d.at<float>(row, col);
            if (depth > 0.01){
                float diff_intensity = range_max - range_min;
                if( diff_intensity == 0 ){
                    diff_intensity = 1e20;
                }
                float value = 1.0 - (depth - range_min)/diff_intensity;

                value = std::min(value, 1.0f);
                value = std::max(value, 0.0f);

                float h = value * 5.0f + 1.0f;
                int i = floor(h);
                float f = h - i;
                if ( !(i&1) ) f = 1 - f; // if i is even
                float n = 1 - f;
                cv::Vec3f color;

                if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
                else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
                else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
                else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
                else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;
                color *= 255;

                out.at<cv::Vec3b>(row, col) = cv::Vec3b(color[2], color[1], color[0]);
            } else {
                if (!rgb.empty()){
                    if (rgb.channels()==3){
                        out.at<cv::Vec3b>(row, col) = rgb.at<cv::Vec3b>(row, col);
                    } else {
                        uchar intensity = rgb.at<uchar>(row, col);
                        out.at<cv::Vec3b>(row, col) = cv::Vec3b(intensity, intensity, intensity);
                    }
                } else {
                    out.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,0);
                }
            }
        }
    }
    return out;
}


int clip(int limit, int val){
        return MIN(limit-1, MAX(0, val ));
}

void setCamInfo(sensor_msgs::CameraInfoConstPtr info){
    fx = info->K[0];
    fy = info->K[4];
    cx = info->K[2];
    cy = info->K[5];
}

void callback(const ImageConstPtr& img_i, const ImageConstPtr& img_z){

    if (fx == 0){
        return;
    }
    cv_bridge::CvImagePtr cv_ptr_i,cv_ptr_z;
    try {
        cv_ptr_i = cv_bridge::toCvCopy(img_i, "");
        cv_ptr_z = cv_bridge::toCvCopy(img_z, "");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }




    if (!pre.empty()){
        cv::Mat d;

        cv::absdiff(cv_ptr_i->image, pre, d);
        cv_ptr_i->image.copyTo(pre);
        cv::Mat background = d>10;



        cv::Mat element = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
        cv::Mat element2 = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size( 6*morph_size + 1, 6*morph_size+1 ), cv::Point( morph_size, morph_size ) );

        /// Apply the specified morphology operation
        cv::morphologyEx( background, background, CV_MOP_OPEN, element );
        cv::morphologyEx( background, background, CV_MOP_DILATE, element2 );

        if (prede.empty()){
            background.copyTo(prede);
            return;
        }

        cv::Mat newarea;

        cv::bitwise_or(background, prede, newarea);
        background.copyTo(prede);

        cv::Mat dist;
        cv::distanceTransform(newarea>0, dist, CV_DIST_L2,5);

        double mn, mx;
        cv::Point mxloc;
        cv::minMaxLoc(dist, &mn, &mx, 0, &mxloc);

        if (mx>2
            && mxloc.x-2*mx > 0
            && mxloc.y-2*mx > 0
            && mxloc.x + 2*mx < cv_ptr_i->image.cols
            && mxloc.y + 2*mx < cv_ptr_i->image.rows){
            //cv::circle(cv_ptr_i->image, mxloc, mx*2, CV_RGB(255,0,0));
            cv::Rect roi = cv::Rect(mxloc-cv::Point(1.5*mx,1.5*mx), cv::Size(mx*3, mx*3));
            cv::rectangle(cv_ptr_i->image, roi, CV_RGB(255,0,0));

            double z3d;
            cv::minMaxLoc(cv_ptr_z->image(roi), &z3d,0);


            ROS_INFO_STREAM("CX,FX" << cx << " " << fx);
//            float x3d = (mxloc.x+cx*fx + cx)/z3d;
//            float y3d = (mxloc.y+cy*fx + cy)/z3d;
            float x3d = (mxloc.x * 1/fx + 1/cx) / (1/z3d);
            float y3d = (mxloc.y * 1/fy + 1/cy) / (1/z3d);
        }






        cv::imshow("combo",image2depthVis(cv_ptr_z->image, cv_ptr_i->image));
        cv::imshow("mono", cv_ptr_i->image);
        cv::waitKey(10);
    } else {

cv_ptr_i->image.copyTo(pre);
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> sub_img_i(nh, "/camera/rgb/image_rect", 1);
  message_filters::Subscriber<Image> sub_img_z(nh, "/camera/depth_registered/image_rect", 1);
  ros::Subscriber sub_cam_info = nh.subscribe("/camera/rgb/camera_info", 1, setCamInfo);
//  message_filters::Subscriber<Image> sub_img_i(nh, "/camera/rgb/image_color", 1);
//  message_filters::Subscriber<Image> sub_img_z(nh, "/camera/depth/image", 1);
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), sub_img_i, sub_img_z);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
