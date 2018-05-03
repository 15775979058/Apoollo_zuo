/******************************************************************************
 * Zuo added on 2018-05-03
 * Added camera subnode for frontLeft/frontRight
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBORAD_CAMERA_PROCESS_FRONTLEFT_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBORAD_CAMERA_PROCESS_FRONTLEFT_SUBNODE_H_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "yaml-cpp/yaml.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/base/singleton.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/converter/geometry_camera_converter.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"
#include "modules/perception/obstacle/camera/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/camera/filter/object_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_converter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"
#include "modules/perception/obstacle/camera/tracker/cascaded_camera_tracker.h"
#include "modules/perception/obstacle/camera/transformer/flat_camera_transformer.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/traffic_light/util/color_space.h"

namespace apollo {
namespace perception {

class CameraProcessFrontLeftSubnode : public Subnode {
 public:
  CameraProcessFrontLeftSubnode() = default;
  ~CameraProcessFrontLeftSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  bool InitInternal() override;

  bool InitCalibration();

  bool InitModules();

  void ImgCallback(const sensor_msgs::Image& message);

  void ChassisCallback(const apollo::canbus::Chassis& message);

  bool MessageToMat(const sensor_msgs::Image& msg, cv::Mat* img);

  void VisualObjToSensorObj(
      const std::vector<std::shared_ptr<VisualObject>>& objects,
      SharedDataPtr<SensorObjects>* sensor_objects);

  void PublishDataAndEvent(const double& timestamp,
                           const SharedDataPtr<SensorObjects>& sensor_objects,
                           const SharedDataPtr<CameraItem>& camera_item);

  void PublishPerceptionPb(const SharedDataPtr<SensorObjects>& sensor_objects);

  // General
  std::string device_id_ = "camera";
  SeqId seq_num_ = 0;
  double timestamp_ns_ = 0.0;

  // Publish Peception Pb
  std::mutex camera_mutex_;
  bool publish_ = false;
  apollo::canbus::Chassis chassis_;

  // Shared Data
  CameraObjectData* cam_obj_data_;
  CameraSharedData* cam_shared_data_;

  // Calibration
  int32_t image_height_ = 1080;
  int32_t image_width_ = 1920;
  Eigen::Matrix4d camera_to_car_;
  Eigen::Matrix<double, 3, 4> intrinsics_;

  // Modules
  std::unique_ptr<BaseCameraDetector> detector_;
  std::unique_ptr<BaseCameraConverter> converter_;
  std::unique_ptr<BaseCameraTracker> tracker_;
  std::unique_ptr<BaseCameraTransformer> transformer_;
  std::unique_ptr<BaseCameraFilter> filter_;
};

REGISTER_SUBNODE(CameraProcessFrontLeftSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_CAMERA_PROCESS_FRONTLEFT_SUBNODE_H_
