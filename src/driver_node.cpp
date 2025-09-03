#include "HCNetSDK.h"
#include "PlayM4.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <functional>

#define UNUSED(expr) (void)(expr)

class HikvisionCamera {
public:
  HikvisionCamera(const std::string &ip, const std::string &username,
                  const std::string &password,
                  rclcpp::Node::SharedPtr node_ptr,
                  const std::string &topic_name,
                  std::function<void(rclcpp::Time)> timestamp_callback)
      : node_(node_ptr),
        userID_(-1),
        playHandle_(-1),
        port_(-1),
        timestamp_callback_(timestamp_callback) {

    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

    if (!PlayM4_GetPort(&port_)) {
      RCLCPP_ERROR(node_->get_logger(), "PlayM4_GetPort failed for IP %s", ip.c_str());
      return;
    }

    PlayM4_SetDisplayBuf(port_, 1);

    NET_DVR_DEVICEINFO_V30 deviceInfo;
    userID_ = NET_DVR_Login_V30(
        const_cast<char *>(ip.c_str()), 8000,
        const_cast<char *>(username.c_str()),
        const_cast<char *>(password.c_str()), &deviceInfo);

    if (userID_ < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Login failed for IP %s: %d", ip.c_str(), NET_DVR_GetLastError());
      PlayM4_FreePort(port_);
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Logged in to camera %s", ip.c_str());

    // Sync time with PC clock
    time_t raw_time;
    time(&raw_time);
    struct tm *timeinfo = localtime(&raw_time);

    NET_DVR_TIME dvrTime = {};
    dvrTime.dwYear   = timeinfo->tm_year + 1900;
    dvrTime.dwMonth  = timeinfo->tm_mon + 1;
    dvrTime.dwDay    = timeinfo->tm_mday;
    dvrTime.dwHour   = timeinfo->tm_hour;
    dvrTime.dwMinute = timeinfo->tm_min;
    dvrTime.dwSecond = timeinfo->tm_sec;

    BOOL timeSynced = NET_DVR_SetDVRConfig(
        userID_,
        NET_DVR_SET_TIMECFG,
        0, // Global config
        &dvrTime,
        sizeof(dvrTime)
    );

    if (!timeSynced) {
      DWORD err = NET_DVR_GetLastError();
      RCLCPP_WARN(node_->get_logger(), "Failed to sync time with camera %s: error %d",
                  ip.c_str(), err);
    } else {
      RCLCPP_INFO(node_->get_logger(), "Time synced with camera %s", ip.c_str());
    }

    NET_DVR_SetExceptionCallBack_V30(0, nullptr, ExceptionCallback, this);

    NET_DVR_CLIENTINFO clientInfo = {};
    clientInfo.lChannel = 1;
    clientInfo.lLinkMode = 0;
    clientInfo.hPlayWnd = 0;

    playHandle_ = NET_DVR_RealPlay_V30(userID_, &clientInfo, FrameCallback, this, TRUE);

    if (playHandle_ < 0) {
      RCLCPP_ERROR(node_->get_logger(), "RealPlay failed for IP %s: %d", ip.c_str(), NET_DVR_GetLastError());
      NET_DVR_Logout(userID_);
      PlayM4_FreePort(port_);
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Streaming started for camera %s", ip.c_str());
  }

  ~HikvisionCamera() {
    cleanup();
  }

private:
  void cleanup() {
    if (playHandle_ >= 0) {
      NET_DVR_StopRealPlay(playHandle_);
      playHandle_ = -1;
    }

    if (userID_ >= 0) {
      NET_DVR_Logout(userID_);
      userID_ = -1;
    }

    if (port_ >= 0) {
      PlayM4_Stop(port_);
      PlayM4_CloseStream(port_);
      PlayM4_FreePort(port_);
      port_ = -1;
    }
  }

  static void CALLBACK ExceptionCallback(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser) {
    auto *camera = static_cast<HikvisionCamera *>(pUser);
    RCLCPP_WARN(camera->node_->get_logger(), "SDK Exception: Type=%u, Handle=%d", dwType, lHandle);
  }

  static void CALLBACK FrameCallback(LONG, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser) {
    auto *camera = static_cast<HikvisionCamera *>(pUser);
    if (!camera || !pBuffer || dwBufSize <= 0) return;

    switch (dwDataType) {
      case NET_DVR_SYSHEAD:
        if (!PlayM4_SetStreamOpenMode(camera->port_, STREAME_REALTIME)) return;
        if (!PlayM4_OpenStream(camera->port_, pBuffer, dwBufSize, 2 * 1024 * 1024)) return;
        if (!PlayM4_SetDecCallBackMend(camera->port_, DecodeCallback, camera)) return;
        if (!PlayM4_Play(camera->port_, 0)) return;
        break;

      case NET_DVR_STREAMDATA:
        PlayM4_InputData(camera->port_, pBuffer, dwBufSize);
        break;

      default:
        break;
    }
  }

  static void CALLBACK DecodeCallback(int, char *pBuf, int, FRAME_INFO *pFrameInfo, void *nUser, int) {
    auto *camera = reinterpret_cast<HikvisionCamera *>(nUser);
    if (!camera || !pBuf || !pFrameInfo) return;

    if (pFrameInfo->nType != T_YV12) return;

    int width = pFrameInfo->nWidth;
    int height = pFrameInfo->nHeight;

    try {
      cv::Mat yv12(height * 3 / 2, width, CV_8UC1, pBuf);
      cv::Mat bgr;
      cv::cvtColor(yv12, bgr, cv::COLOR_YUV2BGR_YV12);

      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();

      if (pFrameInfo->nStamp != 0) {
        uint64_t ts_ms = static_cast<uint64_t>(pFrameInfo->nStamp);
        msg->header.stamp.sec = ts_ms / 1000;
        msg->header.stamp.nanosec = (ts_ms % 1000) * 1e6;
      } else {
        msg->header.stamp = camera->node_->now();
      }

      msg->header.frame_id = "camera_link";
      camera->image_pub_->publish(*msg);

      if (camera->timestamp_callback_) {
        camera->timestamp_callback_(msg->header.stamp);
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(camera->node_->get_logger(), "Decode exception: %s", e.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  LONG userID_;
  LONG playHandle_;
  LONG port_;
  std::function<void(rclcpp::Time)> timestamp_callback_;
};

// ================= NODE MANAGER =====================

class HikvisionNodeManager : public rclcpp::Node {
public:
  HikvisionNodeManager() : Node("hikvision_node_manager") {
    ip1_ = this->declare_parameter<std::string>("ip1", "192.168.1.65");
    ip2_ = this->declare_parameter<std::string>("ip2", "192.168.1.66");
    username_ = this->declare_parameter<std::string>("username", "admin");
    password_ = this->declare_parameter<std::string>("password", "12345");
  }

  void init() {
    if (!NET_DVR_Init()) {
      RCLCPP_ERROR(this->get_logger(), "NET_DVR_Init failed");
      rclcpp::shutdown();
      return;
    }

    auto self = this->shared_from_this();

    camera1_ = std::make_unique<HikvisionCamera>(
        ip1_, username_, password_, self, "camera1/image_raw",
        [this](rclcpp::Time ts) { this->log_time_difference("cam1", ts); });

    camera2_ = std::make_unique<HikvisionCamera>(
        ip2_, username_, password_, self, "camera2/image_raw",
        [this](rclcpp::Time ts) { this->log_time_difference("cam2", ts); });
  }

  ~HikvisionNodeManager() {
    NET_DVR_Cleanup();
  }

private:
  void log_time_difference(const std::string &source, rclcpp::Time ts) {
    if (source == "cam1") last_time_cam1_ = ts;
    if (source == "cam2") last_time_cam2_ = ts;

    if (last_time_cam1_.nanoseconds() > 0 && last_time_cam2_.nanoseconds() > 0) {
      double delta_ms = std::abs((last_time_cam1_ - last_time_cam2_).nanoseconds()) / 1e6;
      // RCLCPP_INFO(this->get_logger(), "Timestamp delta between cameras: %.2f ms", delta_ms);
    }
  }

  std::unique_ptr<HikvisionCamera> camera1_;
  std::unique_ptr<HikvisionCamera> camera2_;
  std::string ip1_, ip2_, username_, password_;
  rclcpp::Time last_time_cam1_;
  rclcpp::Time last_time_cam2_;
};

// ================= MAIN =====================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<HikvisionNodeManager>();
    node->init();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("hikvision_node_manager"), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
