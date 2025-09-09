#include "HCNetSDK.h"
#include "PlayM4.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <functional>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>

#define UNUSED(expr) (void)(expr)

struct FrameData {
    std::vector<uint8_t> buffer;
    int width;
    int height;
    uint64_t timestamp;
    std::string camera_id;
};

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
        timestamp_callback_(timestamp_callback),
        camera_id_(ip),
        processing_thread_running_(true) {

    // Cola más grande para el publisher
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, 100);

    if (!PlayM4_GetPort(&port_)) {
      RCLCPP_ERROR(node_->get_logger(), "PlayM4_GetPort failed for IP %s", ip.c_str());
      return;
    }

    // Buffer más grande para evitar pérdida de frames
    PlayM4_SetDisplayBuf(port_, 10);  // Aumentado de 1 a 10

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
        0,
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

    // Iniciar hilo de procesamiento antes del streaming
    processing_thread_ = std::thread(&HikvisionCamera::processFrames, this);

    playHandle_ = NET_DVR_RealPlay_V30(userID_, &clientInfo, FrameCallback, this, TRUE);

    if (playHandle_ < 0) {
      RCLCPP_ERROR(node_->get_logger(), "RealPlay failed for IP %s: %d", ip.c_str(), NET_DVR_GetLastError());
      processing_thread_running_ = false;
      cv_.notify_all();
      if (processing_thread_.joinable()) {
        processing_thread_.join();
      }
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
    processing_thread_running_ = false;
    cv_.notify_all();
    
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }

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

  // Hilo separado para procesar frames
  void processFrames() {
    while (processing_thread_running_) {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this] { return !frame_queue_.empty() || !processing_thread_running_; });
      
      if (!processing_thread_running_) break;
      
      FrameData frame = frame_queue_.front();
      frame_queue_.pop();
      lock.unlock();

      try {
        // Procesamiento fuera del callback crítico
        cv::Mat yv12(frame.height * 3 / 2, frame.width, CV_8UC1, frame.buffer.data());
        cv::Mat bgr;
        cv::cvtColor(yv12, bgr, cv::COLOR_YUV2BGR_YV12);

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();

        if (frame.timestamp != 0) {
          msg->header.stamp.sec = frame.timestamp / 1000;
          msg->header.stamp.nanosec = (frame.timestamp % 1000) * 1e6;
        } else {
          msg->header.stamp = node_->now();
        }

        msg->header.frame_id = "camera_link";
        
        // Publicación asíncrona
        image_pub_->publish(*msg);

        if (timestamp_callback_) {
          timestamp_callback_(msg->header.stamp);
        }

        // Estadísticas de frames
        frame_count_++;
        if (frame_count_ % 100 == 0) {
          std::lock_guard<std::mutex> stats_lock(queue_mutex_);
          RCLCPP_DEBUG(node_->get_logger(), 
                      "Camera %s: Processed %zu frames, Queue size: %zu", 
                      camera_id_.c_str(), frame_count_.load(), frame_queue_.size());
        }

      } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Frame processing exception for %s: %s", 
                    camera_id_.c_str(), e.what());
      }
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
        if (!PlayM4_OpenStream(camera->port_, pBuffer, dwBufSize, 4 * 1024 * 1024)) return;  // Buffer más grande
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

    // Procesamiento mínimo en el callback crítico
    FrameData frame;
    frame.width = pFrameInfo->nWidth;
    frame.height = pFrameInfo->nHeight;
    frame.timestamp = static_cast<uint64_t>(pFrameInfo->nStamp);
    frame.camera_id = camera->camera_id_;
    
    int data_size = frame.height * frame.width * 3 / 2;  // YV12 format
    frame.buffer.resize(data_size);
    memcpy(frame.buffer.data(), pBuf, data_size);

    // Añadir a cola de procesamiento de forma no bloqueante
    std::lock_guard<std::mutex> lock(camera->queue_mutex_);
    
    // Limitar tamaño de cola para evitar uso excesivo de memoria
    if (camera->frame_queue_.size() > 50) {
      RCLCPP_WARN_THROTTLE(camera->node_->get_logger(), 
                           *camera->node_->get_clock(), 5000,
                           "Frame queue full for camera %s, dropping frames", 
                           camera->camera_id_.c_str());
      camera->frame_queue_.pop();  // Eliminar frame más antiguo
      camera->dropped_frames_++;
    }
    
    camera->frame_queue_.push(std::move(frame));
    camera->cv_.notify_one();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  LONG userID_;
  LONG playHandle_;
  LONG port_;
  std::function<void(rclcpp::Time)> timestamp_callback_;
  std::string camera_id_;
  
  // Threading y queue management
  std::thread processing_thread_;
  std::atomic<bool> processing_thread_running_;
  std::queue<FrameData> frame_queue_;
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  
  // Estadísticas
  std::atomic<size_t> frame_count_{0};
  std::atomic<size_t> dropped_frames_{0};
};

// ================= NODE MANAGER =====================

class HikvisionNodeManager : public rclcpp::Node {
public:
  HikvisionNodeManager() : Node("hikvision_node_manager") {
    ip1_ = this->declare_parameter<std::string>("ip1", "192.168.1.65");
    ip2_ = this->declare_parameter<std::string>("ip2", "192.168.1.66");
    username_ = this->declare_parameter<std::string>("username", "admin");
    password_ = this->declare_parameter<std::string>("password", "12345");
    
    // Configurar QoS para mejor rendimiento con rosbag
    auto qos = rclcpp::QoS(100)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
        .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  }

  void init() {
    if (!NET_DVR_Init()) {
      RCLCPP_ERROR(this->get_logger(), "NET_DVR_Init failed");
      rclcpp::shutdown();
      return;
    }

    // Configurar threads del SDK para mejor rendimiento
    NET_DVR_SetLogToFile(3, "/tmp/hikvision_sdk.log", false);

    auto self = this->shared_from_this();

    camera1_ = std::make_unique<HikvisionCamera>(
        ip1_, username_, password_, self, "camera_left/image_raw",
        [this](rclcpp::Time ts) { this->log_time_difference("cam1", ts); });

    camera2_ = std::make_unique<HikvisionCamera>(
        ip2_, username_, password_, self, "camera_right/image_raw",
        [this](rclcpp::Time ts) { this->log_time_difference("cam2", ts); });

    // Timer para estadísticas periódicas
    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        [this]() {
          RCLCPP_INFO(this->get_logger(), "Cameras running normally");
        });
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
      // Log solo si la diferencia es significativa
      if (delta_ms > 100) {  // > 100ms
        RCLCPP_DEBUG(this->get_logger(), "Time delta between cameras: %.2f ms", delta_ms);
      }
    }
  }

  std::unique_ptr<HikvisionCamera> camera1_;
  std::unique_ptr<HikvisionCamera> camera2_;
  std::string ip1_, ip2_, username_, password_;
  rclcpp::Time last_time_cam1_;
  rclcpp::Time last_time_cam2_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

// ================= MAIN =====================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  // Configurar executor para mejor rendimiento
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  
  try {
    auto node = std::make_shared<HikvisionNodeManager>();
    node->init();
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("hikvision_node_manager"), "Fatal error: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}