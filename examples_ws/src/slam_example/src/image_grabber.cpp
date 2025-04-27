#include "slam_example/image_grabber.hpp"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
    return seconds;
  }
};

ImageGrabber::ImageGrabber()
    : mbClahe(false), first_pose(true) {}

ImageGrabber::ImageGrabber(
    std::shared_ptr<ORB_SLAM3::System> pSLAM,
    bool bClahe,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    std::shared_ptr<rclcpp::Node> ros_node,
    const std::string camera_frame_name
) : mpSLAM(pSLAM)
 , mbClahe(bClahe)
 , first_pose(true)
 , odom_pub_(rospub)
 , rosNode_(ros_node)
 , tf_frame(camera_frame_name)
 , mClahe(cv::createCLAHE(3.0, cv::Size(8,8)))
{
    odom_msg_.header.frame_id    = "odom";
    odom_msg_.child_frame_id     = tf_frame;
    RCLCPP_INFO_STREAM(
        rosNode_->get_logger(),
        "ImageGrabber initialized: odom_frame=" << odom_msg_.header.frame_id
    );
}

void ImageGrabber::grabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    if (!img0Buf.empty()) img0Buf.pop();
    img0Buf.push(msg);
}

void ImageGrabber::grabImu(const ImuMsg::SharedPtr msg)
{
    // GrabImu implementation...

    mBufMutexImu.lock();

    if (!mImuBuf.empty())
        mImuBuf.pop();
    mImuBuf.push(msg);

    mBufMutexImu.unlock();
}

cv::Mat ImageGrabber::getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ImageGrabber"), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
    return cv_ptr->image.clone();
}

void ImageGrabber::processImages()
{
    RCLCPP_INFO(rosNode_->get_logger(), "processImages thread started");
    
    while (rclcpp::ok()) {
        if (img0Buf.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        cv::Mat im;
        double tIm = 0.0;
        {
            std::lock_guard<std::mutex> lk(mBufMutex);
            auto img = img0Buf.front();
            im = getImage(img);
            tIm = img->header.stamp.sec + img->header.stamp.nanosec * 1e-9;
            img0Buf.pop();
        }
        if (im.empty()) {
            RCLCPP_WARN(rosNode_->get_logger(), "Empty image received");
            continue;
        }
        if (mbClahe) mClahe->apply(im, im);

        

        // Monocular tracking
        auto vImuMeas = imuToSlam(tIm);
        Sophus::SE3f curr_pose = mpSLAM->TrackMonocular(im, tIm, vImuMeas);
        auto state = mpSLAM->GetTrackingState();

        LogResult(curr_pose);

        if (state == TS::OK) {
            publishSE3fToOdom(curr_pose);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ImageGrabber::FpsLog(double tIm) {
    static double last_img_time = 0.0;
    static double img_accum_dt = 0.0;
    static int    img_count = 0;
    static double last_imu_time = 0.0;
    static double imu_accum_dt = 0.0;
    static int    imu_count = 0;
    static double last_report_time = 0.0;
    // Камера: accumulate dt and count
    if (last_img_time > 0.0) {
        double dt_img = tIm - last_img_time;
        img_accum_dt += dt_img;
        img_count++;
    }
    last_img_time = tIm;

    // Report every 30s
    if (last_report_time <= 0.0) {
        last_report_time = tIm;
    } else if (tIm - last_report_time >= 30.0) {
        if (img_count > 0) {
            double avg_fps = img_count / img_accum_dt;
            RCLCPP_INFO(rosNode_->get_logger(), "Avg Camera FPS (30s): %.2f", avg_fps);
        }
        if (imu_count > 0) {
            double avg_imu_freq = imu_count / imu_accum_dt;
            RCLCPP_INFO(rosNode_->get_logger(), "Avg IMU Freq (30s): %.2f Hz", avg_imu_freq);
        }
        // reset
        img_accum_dt = imu_accum_dt = 0.0;
        img_count = imu_count = 0;
        last_report_time = tIm;
    }
}

void ImageGrabber::publishSE3fToOdom(const Sophus::SE3f &se3)
{
    odom_msg_.header.stamp = rosNode_->now();
    auto t = se3.translation();
    odom_msg_.pose.pose.position.x = t.x();
    odom_msg_.pose.pose.position.y = t.y();
    odom_msg_.pose.pose.position.z = t.z();
    Eigen::Quaternionf q(se3.rotationMatrix());
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom_msg_);
}


void ImageGrabber::LogResult(const Sophus::SE3f &curr_pose) {
    auto state = mpSLAM->GetTrackingState();
    std::string state_str;
        switch (state) {
            case TS::SYSTEM_NOT_READY: state_str = "NOT_READY"; break;
            case TS::NO_IMAGES_YET:    state_str = "NO_IMAGES"; break;
            case TS::NOT_INITIALIZED:  state_str = "INITIALIZING"; break;
            case TS::OK:               state_str = "OK"; break;
            case TS::RECENTLY_LOST:    state_str = "RELOCALIZING"; break;
            case TS::LOST:             state_str = "LOST"; break;
            default:                   state_str = "UNKNOWN"; break;
        }
        auto t = curr_pose.translation();
        RCLCPP_INFO(rosNode_->get_logger(),
            "SLAM State: %s | Pos [m] x=%.3f y=%.3f z=%.3f",
            state_str.c_str(), t.x(), t.y(), t.z());
}

vector<ORB_SLAM3::IMU::Point> ImageGrabber::imuToSlam(double tImg) {
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    mBufMutexImu.lock();
    if (!mImuBuf.empty())
    {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!mImuBuf.empty() && Utility::StampToSec(mImuBuf.front()->header.stamp) <= tImg)
        {
            double t = Utility::StampToSec(mImuBuf.front()->header.stamp);

            cv::Point3f acc(mImuBuf.front()->linear_acceleration.x, 
                mImuBuf.front()->linear_acceleration.y, 
                mImuBuf.front()->linear_acceleration.z);

            cv::Point3f gyr(mImuBuf.front()->angular_velocity.x, 
                mImuBuf.front()->angular_velocity.y, 
                mImuBuf.front()->angular_velocity.z);

            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            mImuBuf.pop();
        }
    }
    mBufMutexImu.unlock();

    return vImuMeas;
}