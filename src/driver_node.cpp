#include "HCNetSDK.h"
#include "PlayM4.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <future>
#include <deque>

struct FrameData {
    std::vector<uint8_t> buffer;
    int width;
    int height;
    rclcpp::Time capture_time;  // Timestamp de captura para mejor sincronización
};

struct CameraStats {
    std::atomic<uint64_t> frames_received{0};
    std::atomic<uint64_t> frames_dropped_queue_full{0};
    std::atomic<uint64_t> frames_published{0};
};

class HikvisionCamera {
public:
  HikvisionCamera(const std::string &ip, const std::string &username,
                  const std::string &password,
                  rclcpp::Node::SharedPtr node_ptr,
                  const std::string &topic_name,
                  int jpeg_quality = 85,
                  bool use_compressed = true,
                  bool is_left_camera = true)
      : node_(node_ptr),
        userID_(-1),
        playHandle_(-1),
        port_(-1),
        processing_thread_running_(true),
        jpeg_quality_(jpeg_quality),
        use_compressed_(use_compressed),
        is_left_camera_(is_left_camera) {

    // QoS optimizado para rosbag y cero pérdida de paquetes
    // - RELIABLE: garantiza entrega (compatible con RViz2)
    // - History depth 50: buffer grande para absorber picos de tráfico
    // - Deadline 100ms: detecta publicadores lentos
    // - Liveliness 3s: detecta desconexiones
    auto qos = rclcpp::QoS(50)  // Buffer grande para rosbag
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
        .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
        .deadline(std::chrono::milliseconds(100))  // Alertar si publicación > 100ms
        .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
        .liveliness_lease_duration(std::chrono::seconds(3));

    if (use_compressed_) {
      compressed_pub_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
          topic_name + "/compressed", qos);
    } else {
      image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, qos);
    }

    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        topic_name + "/camera_info", qos);

    // Timer para reportar solo problemas críticos (pérdida por cola llena)
    stats_timer_ = node_->create_wall_timer(
        std::chrono::seconds(10),
        [this, topic_name]() {
          uint64_t dropped_queue = stats_.frames_dropped_queue_full.load();
          
          // Solo reportar si hay frames descartados por cola llena (problema real)
          if (dropped_queue > 0) {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] Queue overflow: %lu frames dropped",
                        topic_name.c_str(), dropped_queue);
          }
        });

    // Optimización: configurar OpenCV threads una sola vez
    cv::setNumThreads(2);  // 2 threads para operaciones paralelas (cvtColor, etc)

    if (!PlayM4_GetPort(&port_)) {
      RCLCPP_ERROR(node_->get_logger(), "PlayM4_GetPort failed for %s", ip.c_str());
      return;
    }

    PlayM4_SetDisplayBuf(port_, 10);

    NET_DVR_DEVICEINFO_V30 deviceInfo;
    userID_ = NET_DVR_Login_V30(
        const_cast<char *>(ip.c_str()), 8000,
        const_cast<char *>(username.c_str()),
        const_cast<char *>(password.c_str()), &deviceInfo);

    if (userID_ < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Login failed for %s: error %d", 
                   ip.c_str(), NET_DVR_GetLastError());
      PlayM4_FreePort(port_);
      return;
    }

    syncCameraTime();

    NET_DVR_SetExceptionCallBack_V30(0, nullptr, ExceptionCallback, this);

    NET_DVR_CLIENTINFO clientInfo = {};
    clientInfo.lChannel = 1;
    clientInfo.lLinkMode = 0;
    clientInfo.hPlayWnd = 0;

    // Iniciar thread de procesamiento principal
    processing_thread_ = std::thread(&HikvisionCamera::processFrames, this);
    
    // Iniciar thread pool para procesamiento paralelo (2 workers por cámara)
    for (int i = 0; i < 2; ++i) {
      worker_threads_.emplace_back(&HikvisionCamera::workerThread, this);
    }

    playHandle_ = NET_DVR_RealPlay_V30(userID_, &clientInfo, FrameCallback, this, TRUE);

    if (playHandle_ < 0) {
      RCLCPP_ERROR(node_->get_logger(), "RealPlay failed for %s: error %d", 
                   ip.c_str(), NET_DVR_GetLastError());
      stopProcessing();
      NET_DVR_Logout(userID_);
      PlayM4_FreePort(port_);
      return;
    }
  }

  ~HikvisionCamera() {
    cleanup();
  }

private:
  void syncCameraTime() {
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

    if (!NET_DVR_SetDVRConfig(userID_, NET_DVR_SET_TIMECFG, 0, &dvrTime, sizeof(dvrTime))) {
      // Silenciar warning - la sincronización de tiempo no es crítica
    }
  }

  void stopProcessing() {
    processing_thread_running_ = false;
    
    // Notificar a todos los threads
    cv_.notify_all();
    work_cv_.notify_all();
    
    // Esperar thread principal
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    
    // Esperar workers del thread pool
    for (auto& worker : worker_threads_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  void cleanup() {
    stopProcessing();

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

  void processFrames() {
    // Thread principal: distribuye frames a workers para procesamiento paralelo
    while (processing_thread_running_) {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this] { return !frame_queue_.empty() || !processing_thread_running_; });
      
      if (!processing_thread_running_) break;
      
      FrameData frame = frame_queue_.front();
      frame_queue_.pop();
      lock.unlock();

      // Publicar todos los frames a FPS nativo de la cámara (~50 FPS)
      // Sin control de FPS artificial para máxima fluidez

      // Enviar frame a la cola de trabajo para procesamiento asíncrono
      {
        std::lock_guard<std::mutex> work_lock(work_mutex_);
        work_queue_.push_back(std::move(frame));
      }
      work_cv_.notify_one();
    }
  }

  void workerThread() {
    // Worker thread: procesa frames de forma asíncrona y paralela
    while (processing_thread_running_) {
      FrameData frame;
      
      {
        std::unique_lock<std::mutex> lock(work_mutex_);
        work_cv_.wait(lock, [this] { 
          return !work_queue_.empty() || !processing_thread_running_; 
        });
        
        if (!processing_thread_running_ && work_queue_.empty()) break;
        if (work_queue_.empty()) continue;
        
        frame = std::move(work_queue_.front());
        work_queue_.pop_front();
      }

      // Procesamiento intensivo (cvtColor + JPEG) en paralelo
      publishFrame(frame);
      stats_.frames_published++;
    }
  }

  void publishFrame(const FrameData &frame) {
    try {
      // Conversión YV12 a BGR optimizada con OpenCV paralelo
      cv::Mat yv12(frame.height * 3 / 2, frame.width, CV_8UC1, 
                   const_cast<uint8_t*>(frame.buffer.data()));
      cv::Mat bgr;
      
      // OpenCV usa automáticamente los threads configurados en el constructor
      cv::cvtColor(yv12, bgr, cv::COLOR_YUV2BGR_YV12);

      // Usar timestamp de captura para mejor sincronización temporal
      rclcpp::Time stamp = frame.capture_time;

      if (use_compressed_) {
        publishCompressedImage(bgr, stamp);
      } else {
        publishRawImage(bgr, stamp);
      }

      publishCameraInfo(frame.width, frame.height, stamp);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Frame processing error: %s", e.what());
    }
  }

  void publishCompressedImage(const cv::Mat &bgr, const rclcpp::Time &stamp) {
    auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "camera_link";
    msg->format = "jpeg";

    // Buffer thread-local reutilizable para evitar allocations
    thread_local std::vector<uint8_t> jpeg_buffer;
    jpeg_buffer.clear();  // Limpiar pero mantener capacidad
    
    // Optimizar JPEG encoding: usar optimización rápida
    std::vector<int> params = {
      cv::IMWRITE_JPEG_QUALITY, jpeg_quality_,
      cv::IMWRITE_JPEG_OPTIMIZE, 0,  // Desactivar optimización extra (más rápido)
      cv::IMWRITE_JPEG_PROGRESSIVE, 0  // JPEG baseline (más rápido)
    };
    cv::imencode(".jpg", bgr, jpeg_buffer, params);
    
    msg->data = std::move(jpeg_buffer);  // Mover en lugar de copiar

    compressed_pub_->publish(*msg);
  }

  void publishRawImage(const cv::Mat &bgr, const rclcpp::Time &stamp) {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = "camera_link";
    image_pub_->publish(*msg);
  }

  void publishCameraInfo(int width, int height, const rclcpp::Time &stamp) {
    auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    msg->header.stamp = stamp;
    msg->header.frame_id = "camera_link";
    msg->width = width;
    msg->height = height;
    msg->distortion_model = "plumb_bob";
    
    // Parámetros de calibración reales de las cámaras
    if (is_left_camera_) {
      // Cámara izquierda - Camera matrix
      msg->k = {
        1571.8274485379468, 0.0, 1249.6694603195083,
        0.0, 1575.9598273120064, 750.6778820969588,
        0.0, 0.0, 1.0
      };
      
      // Cámara izquierda - Distortion coefficients (k1, k2, p1, p2, k3)
      msg->d = {
        -0.35262583654991225, 
        0.1800948838208316, 
        -0.000770386070305207, 
        -0.00014062951365452944, 
        -0.06748934647152573
      };
    } else {
      // Cámara derecha - Camera matrix
      msg->k = {
        1561.9905505514428, 0.0, 1252.1852035921543,
        0.0, 1567.541622051507, 758.2859120251709,
        0.0, 0.0, 1.0
      };
      
      // Cámara derecha - Distortion coefficients (k1, k2, p1, p2, k3)
      msg->d = {
        -0.3397013201284959, 
        0.16325369157507694, 
        -0.0008762542630270343, 
        -0.0005334133198146973, 
        -0.06085011032557315
      };
    }
    
    // Rectification matrix (identity para cámaras no rectificadas)
    msg->r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    
    // Projection matrix (K para imágenes no rectificadas)
    if (is_left_camera_) {
      msg->p = {
        1571.8274485379468, 0.0, 1249.6694603195083, 0.0,
        0.0, 1575.9598273120064, 750.6778820969588, 0.0,
        0.0, 0.0, 1.0, 0.0
      };
    } else {
      msg->p = {
        1561.9905505514428, 0.0, 1252.1852035921543, 0.0,
        0.0, 1567.541622051507, 758.2859120251709, 0.0,
        0.0, 0.0, 1.0, 0.0
      };
    }
    
    camera_info_pub_->publish(*msg);
  }

  static void CALLBACK ExceptionCallback(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser) {
    auto *camera = static_cast<HikvisionCamera *>(pUser);
    RCLCPP_ERROR(camera->node_->get_logger(), "Camera exception: %u", dwType);
  }

  static void CALLBACK FrameCallback(LONG, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser) {
    auto *camera = static_cast<HikvisionCamera *>(pUser);
    if (!camera || !pBuffer || dwBufSize == 0) return;

    switch (dwDataType) {
      case NET_DVR_SYSHEAD:
        if (!PlayM4_SetStreamOpenMode(camera->port_, STREAME_REALTIME)) return;
        if (!PlayM4_OpenStream(camera->port_, pBuffer, dwBufSize, 4 * 1024 * 1024)) return;
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
    if (!camera || !pBuf || !pFrameInfo || pFrameInfo->nType != T_YV12) return;

    camera->stats_.frames_received++;

    FrameData frame;
    frame.width = pFrameInfo->nWidth;
    frame.height = pFrameInfo->nHeight;
    frame.capture_time = camera->node_->now();  // Capturar timestamp inmediatamente
    
    // Pre-alocar buffer con tamaño exacto para evitar re-allocations
    int data_size = frame.height * frame.width * 3 / 2;
    frame.buffer.reserve(data_size);
    frame.buffer.resize(data_size);
    memcpy(frame.buffer.data(), pBuf, data_size);

    std::lock_guard<std::mutex> lock(camera->queue_mutex_);
    
    // Aumentar cola de 5 a 30 frames para manejar picos de tráfico
    // Esto previene pérdida de frames durante procesamiento lento temporal
    if (camera->frame_queue_.size() >= 30) {
      camera->frame_queue_.pop();  // Descartar el más antiguo
      camera->stats_.frames_dropped_queue_full++;
    }
    
    camera->frame_queue_.push(std::move(frame));
    camera->cv_.notify_one();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  LONG userID_;
  LONG playHandle_;
  LONG port_;
  
  int jpeg_quality_;
  bool use_compressed_;
  bool is_left_camera_;
  
  CameraStats stats_;
  
  std::thread processing_thread_;
  std::vector<std::thread> worker_threads_;  // Thread pool para procesamiento paralelo
  std::atomic<bool> processing_thread_running_;
  
  std::queue<FrameData> frame_queue_;  // Cola de frames desde cámara
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  
  std::deque<FrameData> work_queue_;  // Cola de trabajo para workers
  std::mutex work_mutex_;
  std::condition_variable work_cv_;
};

class HikvisionNodeManager : public rclcpp::Node {
public:
  HikvisionNodeManager() : Node("hikvision_node_manager") {
    ip1_ = this->declare_parameter<std::string>("ip1", "192.168.1.65");
    ip2_ = this->declare_parameter<std::string>("ip2", "192.168.1.66");
    username_ = this->declare_parameter<std::string>("username", "admin");
    password_ = this->declare_parameter<std::string>("password", "12345");
    jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 85);
    use_compressed_ = this->declare_parameter<bool>("use_compressed", true);
  }

  void init() {
    if (!NET_DVR_Init()) {
      RCLCPP_ERROR(this->get_logger(), "NET_DVR_Init failed");
      rclcpp::shutdown();
      return;
    }

    // Deshabilitar logs del SDK en consola (nivel 0 = solo errores críticos en archivo)
    NET_DVR_SetLogToFile(0, "/tmp/hikvision_sdk.log", false);

    auto self = this->shared_from_this();

    camera1_ = std::make_unique<HikvisionCamera>(
        ip1_, username_, password_, self, "camera_left/image_raw",
        jpeg_quality_, use_compressed_, true);

    camera2_ = std::make_unique<HikvisionCamera>(
        ip2_, username_, password_, self, "camera_right/image_raw",
        jpeg_quality_, use_compressed_, false);
    
    RCLCPP_INFO(this->get_logger(), "Hikvision cameras initialized at native FPS (Quality: %d)", 
                jpeg_quality_);
  }

  ~HikvisionNodeManager() {
    NET_DVR_Cleanup();
  }

private:
  std::unique_ptr<HikvisionCamera> camera1_;
  std::unique_ptr<HikvisionCamera> camera2_;
  std::string ip1_, ip2_, username_, password_;
  int jpeg_quality_;
  bool use_compressed_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  
  try {
    auto node = std::make_shared<HikvisionNodeManager>();
    node->init();
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("hikvision_node_manager"), 
                 "Fatal error: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}