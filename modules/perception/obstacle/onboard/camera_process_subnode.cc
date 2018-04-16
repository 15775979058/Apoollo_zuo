/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/obstacle/onboard/camera_process_subnode.h"

#include <unordered_map>

#include "modules/perception/traffic_light/util/color_space.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;

//-- Zuo added on 2018-04-10
//-- 在Subnode::Init()里被调用
bool CameraProcessSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);

  if (fields.count("device_id")) device_id_ = fields["device_id"];
  if (fields.count("publish") && stoi(fields["publish"])) publish_ = true;

  //-- Zuo added on 2018-04-12
  AINFO << "publish's count = " << fields.count("publish");
  AINFO << "fields[publish] = " << fields["publish"];
  //-- Zuo added on 2018-04-12

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  cam_shared_data_ = static_cast<CameraSharedData *>(
      shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  //-- 这里为什么要设计成Callback，内部使用std::bind //adapter.h
  //-- 会在哪里被调用呢？
  //-- 这里初步判定就是subscribe的回调函数
  AdapterManager::AddImageShortCallback(&CameraProcessSubnode::ImgCallback, this);

  if (publish_) {
    AdapterManager::AddChassisCallback(&CameraProcessSubnode::ChassisCallback, this);
  }

  return true;
}

bool CameraProcessSubnode::InitCalibration() {
  auto ccm = Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration();

  // calibrator->get_image_height_width(&image_height_, &image_width_);
  camera_to_car_ = calibrator->get_camera_extrinsics();
  intrinsics_ = calibrator->get_camera_intrinsic();
  undistortion_handler_ = calibrator->get_camera_undistort_handler();
  return true;
}

bool CameraProcessSubnode::InitModules() {
  RegisterFactoryYoloCameraDetector();
  RegisterFactoryGeometryCameraConverter();
  RegisterFactoryCascadedCameraTracker();
  RegisterFactoryFlatCameraTransformer();
  RegisterFactoryObjectCameraFilter();

  detector_.reset(
      BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector"));
  detector_->Init();

  converter_.reset(BaseCameraConverterRegisterer::GetInstanceByName(
      "GeometryCameraConverter"));
  converter_->Init();

  tracker_.reset(
      BaseCameraTrackerRegisterer::GetInstanceByName("CascadedCameraTracker"));
  tracker_->Init();

  transformer_.reset(BaseCameraTransformerRegisterer::GetInstanceByName(
      "FlatCameraTransformer"));
  transformer_->Init();
  transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(
      BaseCameraFilterRegisterer::GetInstanceByName("ObjectCameraFilter"));
  filter_->Init();

  return true;
}

void CameraProcessSubnode::ImgCallback(const sensor_msgs::Image &message) {
  AdapterManager::Observe();
  sensor_msgs::Image msg = AdapterManager::GetImageShort()->GetLatestObserved();

  double timestamp = msg.header.stamp.toSec();
  AINFO << "CameraProcessSubnode ImgCallback: "
        << " frame: " << ++seq_num_ << " timestamp: ";
  AINFO << std::fixed << std::setprecision(64) << timestamp;
  timestamp_ns_ = timestamp * 1e9;

  cv::Mat img;
  if (!FLAGS_image_file_debug) {
    MessageToMat(msg, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }

  std::vector<VisualObjectPtr> objects;
  cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);

  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);
  converter_->Convert(&objects);
  tracker_->Associate(img, timestamp, &objects);
  transformer_->Transform(&objects);
  filter_->Filter(timestamp, &objects);

  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  out_objs->timestamp = timestamp;
  VisualObjToSensorObj(objects, &out_objs);

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img.clone();
  mask.copyTo(out_objs->camera_frame_supplement->lane_map);
  PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);

  if (publish_) PublishPerceptionPb(out_objs);
}

void CameraProcessSubnode::ChassisCallback(
  const apollo::canbus::Chassis& message) {
  std::lock_guard<std::mutex> lock(camera_mutex_);
  chassis_.CopyFrom(message);
}

bool CameraProcessSubnode::MessageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  cv::Mat cv_img;
  if (msg.encoding.compare("yuyv") == 0) {
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    cv_img = cv::Mat(msg.height, msg.width, CV_8UC3);
    traffic_light::Yuyv2rgb(yuv, cv_img.data, msg.height * msg.width);
    cv::cvtColor(cv_img, cv_img, CV_RGB2BGR);
  } else {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_img = cv_ptr->image;
  }

  if (cv_img.rows != image_height_ || cv_img.cols != image_width_) {
    cv::resize(cv_img, cv_img, cv::Size(image_width_, image_height_));
  }
  *img = cv_img.clone();
  return true;
}

void CameraProcessSubnode::VisualObjToSensorObj(
    const std::vector<VisualObjectPtr> &objects,
    SharedDataPtr<SensorObjects> *sensor_objects) {
  (*sensor_objects)->sensor_type = SensorType::CAMERA;
  (*sensor_objects)->sensor_id = device_id_;
  (*sensor_objects)->seq_num = seq_num_;
  (*sensor_objects)->sensor2world_pose = camera_to_car_;
  ((*sensor_objects)->camera_frame_supplement).reset(new CameraFrameSupplement);

  for (size_t i = 0; i < objects.size(); ++i) {
    VisualObjectPtr vobj = objects[i];
    ObjectPtr obj(new Object());

    obj->id = vobj->id;
    obj->score = vobj->score;
    obj->direction = vobj->direction.cast<double>();
    obj->theta = vobj->theta;
    obj->center = vobj->center.cast<double>();
    obj->length = vobj->length;
    obj->width = vobj->width;
    obj->height = vobj->height;
    obj->type = vobj->type;
    obj->track_id = vobj->track_id;
    obj->tracking_time = vobj->track_age;
    obj->latest_tracked_time = vobj->last_track_timestamp;
    obj->velocity = vobj->velocity.cast<double>();
    obj->anchor_point = obj->center.cast<double>();
    (obj->camera_supplement).reset(new CameraSupplement());
    obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
    obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
    obj->camera_supplement->alpha = vobj->alpha;
    obj->camera_supplement->pts8 = vobj->pts8;

    ((*sensor_objects)->objects).emplace_back(obj);
  }
}

void CameraProcessSubnode::PublishDataAndEvent(
    const double &timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
//   std::string key = "";
//   SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key);
  CommonSharedDataKey key(timestamp, device_id_);
  cam_obj_data_->Add(key, sensor_objects);
  cam_shared_data_->Add(key, camera_item);

  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta &event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

void CameraProcessSubnode::PublishPerceptionPb(
    const SharedDataPtr<SensorObjects>& sensor_objects) {
  AINFO << "Camera publish perception pb data";
  std::lock_guard<std::mutex> lock(camera_mutex_);

  PerceptionObstacles obstacles;

  // Header
  common::adapter::AdapterManager::FillPerceptionObstaclesHeader(
    "perception_obstacle", &obstacles);
  common::Header *header = obstacles.mutable_header();
  header->set_lidar_timestamp(0);
  header->set_camera_timestamp(timestamp_ns_);
  header->set_radar_timestamp(0);
  obstacles.set_error_code(sensor_objects->error_code);

  // Serialize each Object
  for (const auto &obj : sensor_objects->objects) {
    PerceptionObstacle *obstacle = obstacles.add_perception_obstacle();
    obj->Serialize(obstacle);
  }

  // Relative speed of objects + latest ego car speed in X
  for (auto obstacle : obstacles.perception_obstacle()) {
    obstacle.mutable_velocity()->set_x(obstacle.velocity().x() +
                                       chassis_.speed_mps());
  }

  //-- Zuo added on 2018-04-11
  //-- 1. 关于函数PublishPerceptionObstacles()的定义，移步'adapter_manager.h'
  //-- 这里又是用了宏定义+'##'拼接符来实现的。
  //-- 2. 宏'REGISTER_ADAPTER(PerceptionObstacles)'的调用也是在'adapter_manager.h'中,
  //-- 这样子是不是太粗犷了？如果将宏展开放在各自的头文件内(参考subnode Register)，对编译的速度会有多大的提升？
  //--  答：这里不会出现多次声明，因为宏展开是放在类里面，而不是在头文件的普通位置
  common::adapter::AdapterManager::PublishPerceptionObstacles(obstacles);
  ADEBUG << "Camera Obstacles: " << obstacles.ShortDebugString();
}

}  // namespace perception
}  // namespace apollo
