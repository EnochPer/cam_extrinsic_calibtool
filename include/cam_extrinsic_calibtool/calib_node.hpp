#ifndef CAM_EXTRINSIC_CALIBTOOL__CALIB_NODE_HPP_
#define CAM_EXTRINSIC_CALIBTOOL__CALIB_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

namespace cam_extrinsic_calibtool {

struct CalibData {
  std::string image_filename;
  std::string camera_id;
  std::vector<double> pose;  // x, y, z, roll, pitch, yaw
};

struct CalibConfig {
  std::string image_dir;
  std::map<std::string, std::string> camera_intrinsics;
  std::vector<CalibData> data;
  std::string output_file;
  int step = 1;
  double scale_factor = 1.0;

  struct {
    int cols = 0;
    int rows = 0;
    double square_length = 0.0;
  } board;
};

struct CameraInfo {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

class CameraCalibratorNode : public rclcpp::Node {
 public:
  explicit CameraCalibratorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~CameraCalibratorNode();

  // Main entry point
  void run();

 private:
  void load_config(const std::string& config_path);
  void load_camera_info();
  void generate_object_points();

  // Process data from file
  void process_data();

  // Core calculation
  bool detect_corners(const cv::Mat& image, std::vector<cv::Point2f>& corners);
  bool solve_pnp(const std::vector<cv::Point2f>& corners,
                 const CameraInfo& cam_info, Eigen::Affine3d& T_cam_board);

  // Result handling
  void save_result(const Eigen::Affine3d& T_robot_cam,
                   const std::string& filename);
  void save_visualization(const cv::Mat& image, const std::string& filename,
                          const CameraInfo& cam_info);

  CalibConfig config_;
  std::map<std::string, CameraInfo> cameras_;
  std::vector<cv::Point3f> object_points_;  // Defined in board frame
};

}  // namespace cam_extrinsic_calibtool

#endif
