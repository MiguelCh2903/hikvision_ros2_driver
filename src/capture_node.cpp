#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;

class SynchronizedCaptureNode : public rclcpp::Node {
public:
  SynchronizedCaptureNode()
    : Node("synchronized_capture_node") {

    // Suscripciones a las dos cámaras
    cam1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera1/image_raw", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex1_);
          last_frame_cam1_ = msg;
        });

    cam2_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera2/image_raw", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex2_);
          last_frame_cam2_ = msg;
        });

    // Carpeta de salida
    output_dir_ = fs::path(std::getenv("HOME")) / "hikvision_captures";
    if (!fs::exists(output_dir_)) {
      if (!fs::create_directories(output_dir_)) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo crear la carpeta de salida: %s", output_dir_.c_str());
      }
    }

    // Servicio para capturar imágenes
    capture_service_ = this->create_service<std_srvs::srv::Trigger>(
      "capture_images",
      [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Servicio /capture_images invocado. Capturando imágenes...");
        if (capture_and_save()) {
          response->success = true;
          response->message = "Imágenes capturadas correctamente.";
        } else {
          response->success = false;
          response->message = "Error al capturar las imágenes.";
        }
      }
    );

    RCLCPP_INFO(this->get_logger(), "Nodo inicializado. Llama al servicio /capture_images para capturar.");
  }

private:
  bool capture_and_save() {
    sensor_msgs::msg::Image::ConstSharedPtr img1, img2;

    {
      std::lock_guard<std::mutex> lock1(mutex1_);
      std::lock_guard<std::mutex> lock2(mutex2_);
      img1 = last_frame_cam1_;
      img2 = last_frame_cam2_;
    }

    if (!img1 || !img2) {
      RCLCPP_WARN(this->get_logger(), "No se han recibido imágenes aún.");
      return false;
    }

    try {
      cv::Mat mat1 = cv_bridge::toCvCopy(img1, "bgr8")->image;
      cv::Mat mat2 = cv_bridge::toCvCopy(img2, "bgr8")->image;

      auto now = std::chrono::system_clock::now();
      std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::tm tm = *std::localtime(&t);

      char timestamp[32];
      std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);

      std::string filename1 = (output_dir_ / ("cam1_" + std::string(timestamp) + ".png")).string();
      std::string filename2 = (output_dir_ / ("cam2_" + std::string(timestamp) + ".png")).string();

      bool success1 = cv::imwrite(filename1, mat1);
      bool success2 = cv::imwrite(filename2, mat2);

      if (success1) {
        RCLCPP_INFO(this->get_logger(), "Imagen guardada: %s", filename1.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error al guardar la imagen: %s", filename1.c_str());
      }

      if (success2) {
        RCLCPP_INFO(this->get_logger(), "Imagen guardada: %s", filename2.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error al guardar la imagen: %s", filename2.c_str());
      }

      return success1 && success2;

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Excepción al guardar las imágenes: %s", e.what());
      return false;
    }
  }

  // Suscripciones
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam2_sub_;

  // Últimas imágenes recibidas
  sensor_msgs::msg::Image::ConstSharedPtr last_frame_cam1_;
  sensor_msgs::msg::Image::ConstSharedPtr last_frame_cam2_;

  // Mutexes para proteger imágenes
  std::mutex mutex1_;
  std::mutex mutex2_;

  // Carpeta de salida
  fs::path output_dir_;

  // Servicio de captura
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SynchronizedCaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
