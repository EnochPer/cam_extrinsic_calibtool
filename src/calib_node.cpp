#include "cam_extrinsic_calibtool/calib_node.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cam_extrinsic_calibtool {

CameraCalibratorNode::CameraCalibratorNode(const rclcpp::NodeOptions& options)
    : Node("camera_calibrator_node", options) {}

CameraCalibratorNode::~CameraCalibratorNode() {}

void CameraCalibratorNode::run() {
  const std::string config_path =
      "/root/3dtof_extrinsic_calib/src/cam_extrinsic_calibtool/config/"
      "extrinsic_config.yaml";
  try {
    load_config(config_path);
    load_camera_info();
    generate_object_points();
    process_data();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
  }
}

void CameraCalibratorNode::load_config(const std::string& config_path) {
  RCLCPP_INFO(this->get_logger(), "Loading config from: %s",
              config_path.c_str());
  YAML::Node config = YAML::LoadFile(config_path);

  config_.image_dir = config["image_dir"].as<std::string>();
  config_.output_file = config["output_file"].as<std::string>();
  if (config["step"]) config_.step = config["step"].as<int>();
  if (config["scale_factor"])
    config_.scale_factor = config["scale_factor"].as<double>();

  // Load camera intrinsics map
  auto intrinsics_node = config["camera_intrinsics"];
  for (YAML::const_iterator it = intrinsics_node.begin();
       it != intrinsics_node.end(); ++it) {
    config_.camera_intrinsics[it->first.as<std::string>()] =
        it->second.as<std::string>();
  }

  // Load data list
  auto data_node = config["data"];
  for (const auto& item : data_node) {
    CalibData data;
    data.image_filename = item["image"].as<std::string>();
    data.camera_id = item["camera"].as<std::string>();
    data.pose = item["pose"].as<std::vector<double>>();
    config_.data.push_back(data);
  }

  auto board_node = config["board_parameters"];
  config_.board.cols = board_node["cols"].as<int>();
  config_.board.rows = board_node["rows"].as<int>();
  config_.board.square_length = board_node["square_length"].as<double>();
}

void CameraCalibratorNode::load_camera_info() {
  for (const auto& [cam_id, info_path] : config_.camera_intrinsics) {
    RCLCPP_INFO(this->get_logger(), "Loading camera info for %s from: %s",
                cam_id.c_str(), info_path.c_str());
    YAML::Node info = YAML::LoadFile(info_path);

    CameraInfo cam_info;
    auto K = info["camera_matrix"]["data"].as<std::vector<double>>();
    cam_info.camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i)
      cam_info.camera_matrix.at<double>(i / 3, i % 3) = K[i];

    auto D = info["distortion_coefficients"]["data"].as<std::vector<double>>();
    cam_info.dist_coeffs = cv::Mat(D.size(), 1, CV_64F);
    for (size_t i = 0; i < D.size(); ++i)
      cam_info.dist_coeffs.at<double>(i) = D[i];

    cameras_[cam_id] = cam_info;
  }
}

void CameraCalibratorNode::generate_object_points() {
  object_points_.clear();
  // OpenCV default: Z=0, top-left is (0,0,0)
  for (int i = 0; i < config_.board.rows; ++i) {
    for (int j = 0; j < config_.board.cols; ++j) {
      object_points_.emplace_back(j * config_.board.square_length,
                                  i * config_.board.square_length, 0.0f);
    }
  }
}

void CameraCalibratorNode::process_data() {
  RCLCPP_INFO(this->get_logger(), "Processing data...");

  int count = 0;
  for (const auto& data : config_.data) {
    if (count++ % config_.step != 0) continue;

    std::string filename = data.image_filename;
    std::string cam_id = data.camera_id;

    if (cameras_.find(cam_id) == cameras_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Unknown camera ID: %s", cam_id.c_str());
      continue;
    }
    const auto& cam_info = cameras_[cam_id];

    double x = data.pose[0];
    double y = data.pose[1];
    double z = data.pose[2];
    double roll = data.pose[3] * M_PI / 180.0;
    double pitch = data.pose[4] * M_PI / 180.0;
    double yaw = data.pose[5] * M_PI / 180.0;

    // Construct T_robot_board for this frame
    Eigen::Translation3d translation(x, y, z);
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond rotation = yawAngle * pitchAngle * rollAngle;
    Eigen::Affine3d T_robot_board = translation * rotation;

    // Load image
    std::string image_path = config_.image_dir + "/" + filename;
    // Load as 8-bit grayscale (JPG)
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to load image: %s",
                  image_path.c_str());
      continue;
    }

    std::vector<cv::Point2f> corners;
    if (detect_corners(image, corners)) {
      Eigen::Affine3d T_cam_board;
      cv::Mat rvec, tvec;
      if (solve_pnp(corners, cam_info, T_cam_board, rvec, tvec)) {
        save_visualization(image, filename, cam_info, rvec, tvec);
        RCLCPP_INFO(
            this->get_logger(),
            "T_cam_board for %s: translation=(%.2f, %.2f, %.2f), "
            "rotation=(%.2f, %.2f, %.2f)",
            filename.c_str(), T_cam_board.translation().x(),
            T_cam_board.translation().y(), T_cam_board.translation().z(),
            T_cam_board.rotation().eulerAngles(2, 1, 0).x() * 180.0 / M_PI,
            T_cam_board.rotation().eulerAngles(2, 1, 0).y() * 180.0 / M_PI,
            T_cam_board.rotation().eulerAngles(2, 1, 0).z() * 180.0 / M_PI);
        // T_robot_cam = T_robot_board * T_cam_board^-1
        Eigen::Affine3d T_robot_cam = T_robot_board * T_cam_board.inverse();
        save_result(T_robot_cam, filename);
        RCLCPP_INFO(this->get_logger(), "Frame %s: Valid pose found and saved.",
                    filename.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Frame %s: PnP failed.",
                    filename.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Frame %s: Corners not found.",
                  filename.c_str());
    }
  }
}

bool CameraCalibratorNode::detect_corners(const cv::Mat& image,
                                          std::vector<cv::Point2f>& corners) {
  // Image is already loaded as grayscale
  const cv::Mat& gray = image;
  cv::Mat detection_image;

  if (config_.scale_factor > 1.0) {
    cv::resize(gray, detection_image, cv::Size(), config_.scale_factor,
               config_.scale_factor, cv::INTER_LINEAR);
  } else {
    detection_image = gray;
  }

  bool found = cv::findChessboardCorners(
      detection_image, cv::Size(config_.board.cols, config_.board.rows),
      corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
          cv::CALIB_CB_FAST_CHECK);

  if (found) {
    // Perform subpixel refinement on the detection image (which might be
    // scaled)
    cv::cornerSubPix(
        detection_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                         0.1));

    if (config_.scale_factor > 1.0) {
      for (auto& pt : corners) {
        pt.x /= static_cast<float>(config_.scale_factor);
        pt.y /= static_cast<float>(config_.scale_factor);
        // std::cout << "Refined corner: (" << pt.x << ", " << pt.y << ")\n";
      }
    }
  }
  return found;
}

bool CameraCalibratorNode::solve_pnp(const std::vector<cv::Point2f>& corners,
                                     const CameraInfo& cam_info,
                                     Eigen::Affine3d& T_cam_board,
                                     cv::Mat& rvec, cv::Mat& tvec) {
  bool success = cv::solvePnP(object_points_, corners, cam_info.camera_matrix,
                              cam_info.dist_coeffs, rvec, tvec);
  if (!success) return false;

  cv::Mat R;
  cv::Rodrigues(rvec, R);

  Eigen::Matrix3d R_eigen;
  Eigen::Vector3d t_eigen;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_eigen(i, j) = R.at<double>(i, j);
    }
    t_eigen(i) = tvec.at<double>(i);
  }

  T_cam_board = Eigen::Translation3d(t_eigen) * R_eigen;
  return true;
}

void CameraCalibratorNode::save_result(const Eigen::Affine3d& T_robot_cam,
                                       const std::string& filename) {
  Eigen::Vector3d t = T_robot_cam.translation();
  Eigen::Matrix3d R = T_robot_cam.rotation();
  // ZYX order: Z(yaw) * Y(pitch) * X(roll)
  // Manual calculation to ensure pitch is in [-90, 90] range
  double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  bool singular = sy < 1e-6;
  double roll, pitch, yaw;

  if (!singular) {
    roll = std::atan2(R(2, 1), R(2, 2));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    roll = std::atan2(-R(1, 2), R(1, 1));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = 0;
  }

  double roll_deg = roll * 180.0 / M_PI;
  double pitch_deg = pitch * 180.0 / M_PI;
  double yaw_deg = yaw * 180.0 / M_PI;

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "T_robot_cam";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "translation";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << t.x();
  out << YAML::Key << "y" << YAML::Value << t.y();
  out << YAML::Key << "z" << YAML::Value << t.z();
  out << YAML::EndMap;
  out << YAML::Key << "rotation_matrix";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << 3;
  out << YAML::Key << "cols" << YAML::Value << 3;
  out << YAML::Key << "data";
  out << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out << R(i, j);
    }
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  out << YAML::Key << "rotation_euler_rpy_deg";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "roll" << YAML::Value << roll_deg;
  out << YAML::Key << "pitch" << YAML::Value << pitch_deg;
  out << YAML::Key << "yaw" << YAML::Value << yaw_deg;
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;

  namespace fs = std::filesystem;
  fs::path output_path = fs::path(config_.output_file).parent_path();
  if (!fs::exists(output_path)) {
    fs::create_directories(output_path);
  }

  // Replace extension with .yaml
  std::string yaml_filename =
      fs::path(filename).stem().string() + "_extrinsic.yaml";
  fs::path save_path = output_path / yaml_filename;

  std::ofstream fout(save_path.string());
  fout << out.c_str();
  RCLCPP_INFO(this->get_logger(), "Calibration saved to %s", save_path.c_str());
}

void CameraCalibratorNode::save_visualization(const cv::Mat& image,
                                              const std::string& filename,
                                              const CameraInfo& cam_info,
                                              const cv::Mat& rvec,
                                              const cv::Mat& tvec) {
  namespace fs = std::filesystem;
  fs::path output_dir = fs::path(config_.image_dir) / "result";
  if (!fs::exists(output_dir)) {
    fs::create_directories(output_dir);
  }

  cv::Mat undistorted_img;
  cv::undistort(image, undistorted_img, cam_info.camera_matrix,
                cam_info.dist_coeffs);

  cv::Mat scaled_img;
  if (config_.scale_factor > 1.0) {
    cv::resize(undistorted_img, scaled_img, cv::Size(), config_.scale_factor,
               config_.scale_factor, cv::INTER_LINEAR);
  } else {
    scaled_img = undistorted_img;
  }

  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(
      scaled_img, cv::Size(config_.board.cols, config_.board.rows), corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
          cv::CALIB_CB_FAST_CHECK);

  if (found) {
    cv::cornerSubPix(
        scaled_img, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                         0.1));
    cv::drawChessboardCorners(scaled_img,
                              cv::Size(config_.board.cols, config_.board.rows),
                              corners, found);

    // Draw coordinate axes
    cv::Mat K_new = cam_info.camera_matrix.clone();
    if (config_.scale_factor > 1.0) {
      K_new.at<double>(0, 0) *= config_.scale_factor;  // fx
      K_new.at<double>(1, 1) *= config_.scale_factor;  // fy
      K_new.at<double>(0, 2) *= config_.scale_factor;  // cx
      K_new.at<double>(1, 2) *= config_.scale_factor;  // cy
    }

    // Since we are drawing on an undistorted image, distortion coefficients are
    // zero
    cv::Mat dist_coeffs_zero = cv::Mat::zeros(5, 1, CV_64F);

    try {
      float axis_length = config_.board.square_length * 3;
      cv::drawFrameAxes(scaled_img, K_new, dist_coeffs_zero, rvec, tvec,
                        axis_length);

      // Add axis labels
      std::vector<cv::Point3f> axis_points;
      axis_points.push_back(cv::Point3f(axis_length, 0, 0));  // X
      axis_points.push_back(cv::Point3f(0, axis_length, 0));  // Y
      axis_points.push_back(cv::Point3f(0, 0, axis_length));  // Z

      std::vector<cv::Point2f> image_points;
      cv::projectPoints(axis_points, rvec, tvec, K_new, dist_coeffs_zero,
                        image_points);

      // Draw labels
      cv::putText(scaled_img, "X", image_points[0], cv::FONT_HERSHEY_SIMPLEX,
                  1.0, cv::Scalar(0, 0, 255), 2);
      cv::putText(scaled_img, "Y", image_points[1], cv::FONT_HERSHEY_SIMPLEX,
                  1.0, cv::Scalar(0, 255, 0), 2);
      cv::putText(scaled_img, "Z", image_points[2], cv::FONT_HERSHEY_SIMPLEX,
                  1.0, cv::Scalar(255, 0, 0), 2);
    } catch (const cv::Exception& e) {
      // Fallback for older OpenCV versions if drawFrameAxes is not available or
      // fails But drawFrameAxes was introduced in 3.4.1, which is quite old.
      RCLCPP_WARN(this->get_logger(), "Failed to draw axes: %s", e.what());
    }
  }

  std::string save_path = (output_dir / filename).string();
  cv::imwrite(save_path, scaled_img);
  RCLCPP_INFO(this->get_logger(), "Saved visualization to: %s",
              save_path.c_str());
}

}  // namespace cam_extrinsic_calibtool

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cam_extrinsic_calibtool::CameraCalibratorNode>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
