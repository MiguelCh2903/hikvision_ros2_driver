#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <atomic>

namespace fs = std::filesystem;

class StereoCaptureNode : public rclcpp::Node {
public:
  StereoCaptureNode()
    : Node("stereo_capture_node"), 
      is_capturing_(false),
      capture_count_(0) {

    // Parámetros del nodo
    this->declare_parameter<std::string>("camera1_topic", "camera1/image_raw");
    this->declare_parameter<std::string>("camera2_topic", "camera2/image_raw");
    this->declare_parameter<std::string>("camera1_name", "cam1");
    this->declare_parameter<std::string>("camera2_name", "cam2");
    this->declare_parameter<double>("capture_interval", 1.0);
    this->declare_parameter<int>("max_captures", 30);

    camera1_topic_ = this->get_parameter("camera1_topic").as_string();
    camera2_topic_ = this->get_parameter("camera2_topic").as_string();
    camera1_name_ = this->get_parameter("camera1_name").as_string();
    camera2_name_ = this->get_parameter("camera2_name").as_string();
    capture_interval_ = this->get_parameter("capture_interval").as_double();
    max_captures_ = this->get_parameter("max_captures").as_int();

    // Suscripciones a las dos cámaras
    cam1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera1_topic_, 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex1_);
          last_frame_cam1_ = msg;
        });

    cam2_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera2_topic_, 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex2_);
          last_frame_cam2_ = msg;
        });

    // Configurar carpeta de salida
    setup_output_directory();

    // Servicio para iniciar captura stereo
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "start_stereo_capture",
      [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        if (is_capturing_.load()) {
          response->success = false;
          response->message = "Ya hay una captura stereo en progreso.";
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Iniciando captura stereo para calibracion...");
        start_capture();
        response->success = true;
        response->message = "Captura stereo iniciada. Intervalo: " + 
                           std::to_string(capture_interval_) + "s, Max capturas: " + 
                           std::to_string(max_captures_);
      }
    );

    // Servicio para detener captura
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_stereo_capture",
      [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        if (!is_capturing_.load()) {
          response->success = false;
          response->message = "No hay ninguna captura stereo en progreso.";
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Deteniendo captura stereo...");
        stop_capture();
        response->success = true;
        response->message = "Captura stereo detenida. Total de pares de imagenes capturadas: " + 
                           std::to_string(capture_count_);
      }
    );

    RCLCPP_INFO(this->get_logger(), "Nodo stereo inicializado.");
    RCLCPP_INFO(this->get_logger(), "Camara 1 - Topico: %s, Nombre: %s", 
                camera1_topic_.c_str(), camera1_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Camara 2 - Topico: %s, Nombre: %s", 
                camera2_topic_.c_str(), camera2_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Directorio de salida: %s", output_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "Servicios disponibles:");
    RCLCPP_INFO(this->get_logger(), "  - /start_stereo_capture");
    RCLCPP_INFO(this->get_logger(), "  - /stop_stereo_capture");
  }

  ~StereoCaptureNode() {
    stop_capture();
  }

private:
  void setup_output_directory() {
    // Crear directorio base
    fs::path base_dir = fs::path(std::getenv("HOME")) / "hikvision_captures";
    
    // Crear subdirectorio para calibración stereo
    std::string stereo_folder_name = camera1_name_ + "_" + camera2_name_;
    output_dir_ = base_dir / "stereo_calib" / stereo_folder_name;
    
    if (!fs::exists(output_dir_)) {
      if (!fs::create_directories(output_dir_)) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo crear la carpeta de salida: %s", 
                     output_dir_.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Carpeta de salida creada: %s", output_dir_.c_str());
      }
    }
  }

  void start_capture() {
    is_capturing_ = true;
    capture_count_ = 0;
    
    // Crear timer para captura periódica
    capture_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(capture_interval_),
      [this]() {
        if (!is_capturing_.load()) {
          capture_timer_->cancel();
          return;
        }

        if (capture_count_ >= max_captures_) {
          RCLCPP_INFO(this->get_logger(), 
                     "Numero maximo de capturas alcanzado (%d). Deteniendo...", 
                     max_captures_);
          stop_capture();
          return;
        }

        capture_stereo_pair();
      }
    );
    
    RCLCPP_INFO(this->get_logger(), "Captura stereo iniciada. Guardando en: %s", output_dir_.c_str());
  }

  void stop_capture() {
    is_capturing_ = false;
    if (capture_timer_) {
      capture_timer_->cancel();
      capture_timer_.reset();
    }
    RCLCPP_INFO(this->get_logger(), "Captura stereo detenida. Total de pares: %d", capture_count_);
  }

  bool capture_stereo_pair() {
    sensor_msgs::msg::Image::ConstSharedPtr img1, img2;

    // Obtener las últimas imágenes de ambas cámaras de forma sincronizada
    {
      std::lock_guard<std::mutex> lock1(mutex1_);
      std::lock_guard<std::mutex> lock2(mutex2_);
      img1 = last_frame_cam1_;
      img2 = last_frame_cam2_;
    }

    if (!img1 || !img2) {
      RCLCPP_WARN(this->get_logger(), "No se han recibido imagenes de ambas camaras aun.");
      return false;
    }

    try {
      cv::Mat mat1 = cv_bridge::toCvCopy(img1, "bgr8")->image;
      cv::Mat mat2 = cv_bridge::toCvCopy(img2, "bgr8")->image;

      // Generar timestamp
      auto now = std::chrono::system_clock::now();
      std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::tm tm = *std::localtime(&t);

      char timestamp[32];
      std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);

      // Nombres de archivos con contador y timestamp
      std::string filename1 = (output_dir_ / 
        (camera1_name_ + "_stereo_" + std::string(timestamp) + 
         "_" + std::to_string(capture_count_ + 1) + ".png")).string();
      
      std::string filename2 = (output_dir_ / 
        (camera2_name_ + "_stereo_" + std::string(timestamp) + 
         "_" + std::to_string(capture_count_ + 1) + ".png")).string();

      bool success1 = cv::imwrite(filename1, mat1);
      bool success2 = cv::imwrite(filename2, mat2);

      if (success1 && success2) {
        capture_count_++;
        RCLCPP_INFO(this->get_logger(), 
                   "Par stereo %d/%d guardado:", 
                   capture_count_, max_captures_);
        RCLCPP_INFO(this->get_logger(), "  - %s: %s", camera1_name_.c_str(), filename1.c_str());
        RCLCPP_INFO(this->get_logger(), "  - %s: %s", camera2_name_.c_str(), filename2.c_str());
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error al guardar el par stereo:");
        if (!success1) {
          RCLCPP_ERROR(this->get_logger(), "  - Error en %s: %s", camera1_name_.c_str(), filename1.c_str());
        }
        if (!success2) {
          RCLCPP_ERROR(this->get_logger(), "  - Error en %s: %s", camera2_name_.c_str(), filename2.c_str());
        }
        return false;
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Excepcion al procesar las imagenes stereo: %s", e.what());
      return false;
    }
  }

  // Parámetros
  std::string camera1_topic_;
  std::string camera2_topic_;
  std::string camera1_name_;
  std::string camera2_name_;
  double capture_interval_;
  int max_captures_;

  // Suscripciones
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam2_sub_;

  // Últimas imágenes recibidas
  sensor_msgs::msg::Image::ConstSharedPtr last_frame_cam1_;
  sensor_msgs::msg::Image::ConstSharedPtr last_frame_cam2_;

  // Mutexes para proteger imágenes
  std::mutex mutex1_;
  std::mutex mutex2_;

  // Control de captura
  std::atomic<bool> is_capturing_;
  int capture_count_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  // Directorio de salida
  fs::path output_dir_;

  // Servicios
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}