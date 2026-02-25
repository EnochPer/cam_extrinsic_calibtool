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
      "/home/zzh/cam_extrinsic_calibtool/src/cam_extrinsic_calibtool/config/"
      "extrinsic_config.yaml";
  try {
    load_config(config_path);
    load_camera_info(config_.camera_info_file);
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
  config_.pose_file = config["pose_file"].as<std::string>();
  config_.camera_info_file = config["camera_info_file"].as<std::string>();
  config_.output_file = config["output_file"].as<std::string>();
  if (config["step"]) config_.step = config["step"].as<int>();
  if (config["scale_factor"])
    config_.scale_factor = config["scale_factor"].as<double>();

  auto board_node = config["board_parameters"];
  config_.board.cols = board_node["cols"].as<int>();
  config_.board.rows = board_node["rows"].as<int>();
  config_.board.square_length = board_node["square_length"].as<double>();
}

void CameraCalibratorNode::load_camera_info(const std::string& info_path) {
  RCLCPP_INFO(this->get_logger(), "Loading camera info from: %s",
              info_path.c_str());
  YAML::Node info = YAML::LoadFile(info_path);

  auto K = info["camera_matrix"]["data"].as<std::vector<double>>();
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) camera_matrix_.at<double>(i / 3, i % 3) = K[i];

  auto D = info["distortion_coefficients"]["data"].as<std::vector<double>>();
  dist_coeffs_ = cv::Mat(D.size(), 1, CV_64F);
  for (size_t i = 0; i < D.size(); ++i) dist_coeffs_.at<double>(i) = D[i];
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
  std::ifstream pose_fs(config_.pose_file);
  if (!pose_fs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open pose file: %s",
                 config_.pose_file.c_str());
    return;
  }

  std::vector<Eigen::Affine3d> valid_poses;
  std::string line;
  int count = 0;

  RCLCPP_INFO(this->get_logger(), "Processing data...");

  while (std::getline(pose_fs, line)) {
    if (line.empty()) continue;
    if (count++ % config_.step != 0) continue;

    std::stringstream ss(line);
    std::string filename;
    double x, y, z, roll, pitch, yaw;

    // Format: filename x y z roll pitch yaw
    if (!(ss >> filename >> x >> y >> z >> roll >> pitch >> yaw)) {
      RCLCPP_WARN(this->get_logger(), "Skipping invalid line: %s",
                  line.c_str());
      continue;
    }
    // x /= 1000;
    // y /= 1000;
    // z /= 1000;
    // Convert units: deg -> rad
    roll *= M_PI / 180.0;
    pitch *= M_PI / 180.0;
    yaw *= M_PI / 180.0;

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
      save_visualization(image, filename);
      Eigen::Affine3d T_cam_board;
      if (solve_pnp(corners, T_cam_board)) {
        // T_robot_cam = T_robot_board * T_cam_board^-1
        Eigen::Affine3d T_robot_cam = T_robot_board * T_cam_board.inverse();
        valid_poses.push_back(T_robot_cam);
        RCLCPP_INFO(this->get_logger(), "Frame %s: Valid pose found.",
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

  if (valid_poses.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No valid poses found!");
    return;
  }

  Eigen::Affine3d final_pose = compute_average_pose(valid_poses);
  save_result(final_pose);
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
      }
    }
  }
  return found;
}

bool CameraCalibratorNode::solve_pnp(const std::vector<cv::Point2f>& corners,
                                     Eigen::Affine3d& T_cam_board) {
  cv::Mat rvec, tvec;
  bool success = cv::solvePnP(object_points_, corners, camera_matrix_,
                              dist_coeffs_, rvec, tvec);
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

Eigen::Affine3d CameraCalibratorNode::compute_average_pose(
    const std::vector<Eigen::Affine3d>& poses) {
  Eigen::Vector3d avg_translation = Eigen::Vector3d::Zero();
  // Simple averaging for translation
  for (const auto& pose : poses) {
    avg_translation += pose.translation();
  }
  avg_translation /= poses.size();

  // Averaging quaternions is tricky, but for small variations,
  // normalizing the sum is a reasonable approximation or using Slerp
  // iteratively. Here we use a simple sum and normalize approach which works
  // for clustered rotations.
  Eigen::Quaterniond avg_quat(0, 0, 0, 0);
  // Use the first quaternion as reference to avoid sign flipping issues
  Eigen::Quaterniond ref_quat(poses[0].rotation());

  for (const auto& pose : poses) {
    Eigen::Quaterniond q(pose.rotation());
    if (q.dot(ref_quat) < 0.0) {
      q.coeffs() = -q.coeffs();
    }
    avg_quat.coeffs() += q.coeffs();
  }
  avg_quat.normalize();

  return Eigen::Translation3d(avg_translation) * avg_quat;
}

void CameraCalibratorNode::save_result(const Eigen::Affine3d& T_robot_cam) {
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

  std::ofstream fout(config_.output_file);
  fout << out.c_str();
  RCLCPP_INFO(this->get_logger(), "Calibration saved to %s",
              config_.output_file.c_str());

  // Print to console
  std::cout << "Final Result (T_robot_cam):\n"
            << "Translation (x, y, z): " << t.transpose() << "\n"
            << "Euler Angles (deg) [Roll, Pitch, Yaw]: " << roll_deg << ", "
            << pitch_deg << ", " << yaw_deg << std::endl;
}

void CameraCalibratorNode::save_visualization(const cv::Mat& image,
                                              const std::string& filename) {
  namespace fs = std::filesystem;
  fs::path output_dir = fs::path(config_.image_dir) / "result";
  if (!fs::exists(output_dir)) {
    fs::create_directories(output_dir);
  }

  cv::Mat undistorted_img;
  cv::undistort(image, undistorted_img, camera_matrix_, dist_coeffs_);

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
