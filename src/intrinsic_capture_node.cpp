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

class IntrinsicCaptureNode : public rclcpp::Node {
public:
  IntrinsicCaptureNode()
    : Node("intrinsic_capture_node"), 
      is_capturing_(false),
      capture_count_(0) {

    // Parámetros del nodo
    this->declare_parameter<std::string>("camera_topic", "camera/image_raw");
    this->declare_parameter<std::string>("camera_name", "camera");
    this->declare_parameter<double>("capture_interval", 1.0);
    this->declare_parameter<int>("max_captures", 30); // Número máximo de capturas por defecto

    camera_topic_ = this->get_parameter("camera_topic").as_string();
    camera_name_ = this->get_parameter("camera_name").as_string();
    capture_interval_ = this->get_parameter("capture_interval").as_double();
    max_captures_ = this->get_parameter("max_captures").as_int();

    // Suscripción a la cámara
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          last_frame_ = msg;
        });

    // Configurar carpeta de salida
    setup_output_directory();

    // Servicio para iniciar captura
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "start_intrinsic_capture",
      [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        if (is_capturing_.load()) {
          response->success = false;
          response->message = "Ya hay una captura en progreso.";
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Iniciando captura intrinseca para calibracion...");
        start_capture();
        response->success = true;
        response->message = "Captura intrinseca iniciada. Intervalo: " + 
                           std::to_string(capture_interval_) + "s, Max capturas: " + 
                           std::to_string(max_captures_);
      }
    );

    // Servicio para detener captura
    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_intrinsic_capture",
      [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        if (!is_capturing_.load()) {
          response->success = false;
          response->message = "No hay ninguna captura en progreso.";
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Deteniendo captura intrinseca...");
        stop_capture();
        response->success = true;
        response->message = "Captura intrinseca detenida. Total de imagenes capturadas: " + 
                           std::to_string(capture_count_);
      }
    );

    RCLCPP_INFO(this->get_logger(), "Nodo inicializado.");
    RCLCPP_INFO(this->get_logger(), "Topico de camara: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Nombre de camara: %s", camera_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Directorio de salida: %s", output_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "Servicios disponibles:");
    RCLCPP_INFO(this->get_logger(), "  - /start_intrinsic_capture");
    RCLCPP_INFO(this->get_logger(), "  - /stop_intrinsic_capture");
  }

  ~IntrinsicCaptureNode() {
    stop_capture();
  }

private:
  void setup_output_directory() {
    // Crear directorio base
    fs::path base_dir = fs::path(std::getenv("HOME")) / "hikvision_captures";
    
    // Crear subdirectorio para calibración intrínseca
    output_dir_ = base_dir / "intrinsic_calib" / camera_name_;
    
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

        capture_frame();
      }
    );
    
    RCLCPP_INFO(this->get_logger(), "Captura iniciada. Guardando en: %s", output_dir_.c_str());
  }

  void stop_capture() {
    is_capturing_ = false;
    if (capture_timer_) {
      capture_timer_->cancel();
      capture_timer_.reset();
    }
    RCLCPP_INFO(this->get_logger(), "Captura detenida. Total de imagenes: %d", capture_count_);
  }

  bool capture_frame() {
    sensor_msgs::msg::Image::ConstSharedPtr img;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      img = last_frame_;
    }

    if (!img) {
      RCLCPP_WARN(this->get_logger(), "No se ha recibido ninguna imagen aun.");
      return false;
    }

    try {
      cv::Mat mat = cv_bridge::toCvCopy(img, "bgr8")->image;

      // Generar timestamp
      auto now = std::chrono::system_clock::now();
      std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::tm tm = *std::localtime(&t);

      char timestamp[32];
      std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);

      // Nombre del archivo con contador
      std::string filename = (output_dir_ / 
        (camera_name_ + "_intrinsic_" + std::string(timestamp) + 
         "_" + std::to_string(capture_count_ + 1) + ".png")).string();

      bool success = cv::imwrite(filename, mat);

      if (success) {
        capture_count_++;
        RCLCPP_INFO(this->get_logger(), 
                   "Imagen %d/%d guardada: %s", 
                   capture_count_, max_captures_, filename.c_str());
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error al guardar la imagen: %s", filename.c_str());
        return false;
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Excepcion al procesar la imagen: %s", e.what());
      return false;
    }
  }

  // Parámetros
  std::string camera_topic_;
  std::string camera_name_;
  double capture_interval_;
  int max_captures_;

  // Suscripción
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

  // Última imagen recibida
  sensor_msgs::msg::Image::ConstSharedPtr last_frame_;

  // Mutex para proteger imagen
  std::mutex mutex_;

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
  auto node = std::make_shared<IntrinsicCaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}