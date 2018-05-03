/******************************************************************************
 * Zuo added on 2018-05-03
 * Added camera subnode for frontLeft/frontRight
 *****************************************************************************/

#include "modules/perception/obstacle/onboard/camera_process_frontRight_subnode.h"

#include "modules/common/time/time_util.h"
#include "modules/perception/cuda_util/util.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;

bool CameraProcessFrontRightSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);

  if (fields.count("device_id")) device_id_ = fields["device_id"];
  if (fields.count("publish") && stoi(fields["publish"])) publish_ = true;

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  cam_shared_data_ = static_cast<CameraSharedData *>(
      shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  AdapterManager::AddImageFrontCallback(&CameraProcessFrontRightSubnode::ImgCallback,
                                        this);
  if (publish_) {
    AdapterManager::AddChassisCallback(&CameraProcessFrontRightSubnode::ChassisCallback,
                                       this);
  }

  return true;
}

bool CameraProcessFrontRightSubnode::InitCalibration() {
  auto ccm = Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration();

  calibrator->get_image_height_width(&image_height_, &image_width_);
  camera_to_car_ = calibrator->get_camera_extrinsics();
  intrinsics_ = calibrator->get_camera_intrinsic();
  return true;
}

bool CameraProcessFrontRightSubnode::InitModules() {
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

void CameraProcessFrontRightSubnode::ImgCallback(const sensor_msgs::Image &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "CameraProcessFrontRightSubnode ImgCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "CameraProcessFrontRightSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ADEBUG << "CameraProcessFrontRightSubnode Process: "
         << " frame: " << ++seq_num_;
  PERF_FUNCTION("CameraProcessFrontRightSubnode");
  PERF_BLOCK_START();

  cv::Mat img;
  if (!FLAGS_image_file_debug) {
    MessageToMat(message, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }
  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat mask;
  PERF_BLOCK_END("CameraProcessFrontRightSubnode_Image_Preprocess");

  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);
  PERF_BLOCK_END("CameraProcessFrontRightSubnode_detector_");

  converter_->Convert(&objects);
  PERF_BLOCK_END("CameraProcessFrontRightSubnode_converter_");

  transformer_->Transform(&objects);
  PERF_BLOCK_END("CameraProcessFrontRightSubnode_transformer_");

  tracker_->Associate(img, timestamp, &objects);
  PERF_BLOCK_END("CameraProcessFrontRightSubnode_tracker_");

  filter_->Filter(timestamp, &objects);
  PERF_BLOCK_END("CameraProcessFrontRightSubnode_filter_");

  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  out_objs->timestamp = timestamp;
  VisualObjToSensorObj(objects, &out_objs);

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img.clone();
  mask.copyTo(out_objs->camera_frame_supplement->lane_map);
  PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
  PERF_BLOCK_END("CameraProcessFrontRightSubnode publish in DAG");

  if (publish_) PublishPerceptionPb(out_objs);

  //-- Zuo added on 2018-05-03
  //-- Added camera subnode for frontLeft/frontRight
  for(int i=0; i<objects.size(); i++){
    cv::Point2f upper_left(objects.at(i)->upper_left.x(), objects.at(i)->upper_left.y());
    cv::Point2f lower_right(objects.at(i)->lower_right.x(), objects.at(i)->lower_right.y());
    cv::rectangle(img, upper_left, lower_right, cv::Scalar(0, 255, 0));
  }
  cv::resize(img, img, cv::Size(640, 480));
  cv::imshow("frontRight", img);
  cv::waitKey(1);

}

void CameraProcessFrontRightSubnode::ChassisCallback(
    const apollo::canbus::Chassis &message) {
  std::lock_guard<std::mutex> lock(camera_mutex_);
  chassis_.CopyFrom(message);
}

bool CameraProcessFrontRightSubnode::MessageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  *img = cv::Mat(msg.height, msg.width, CV_8UC3);
  int pixel_num = msg.width * msg.height;
  if (msg.encoding.compare("yuyv") == 0) {
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    yuyv2bgr(yuv, img->data, pixel_num);
  } else {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    *img = cv_ptr->image;
  }

  return true;
}

void CameraProcessFrontRightSubnode::VisualObjToSensorObj(
    const std::vector<std::shared_ptr<VisualObject>> &objects,
    SharedDataPtr<SensorObjects> *sensor_objects) {
  (*sensor_objects)->sensor_type = SensorType::CAMERA;
  (*sensor_objects)->sensor_id = device_id_;
  (*sensor_objects)->seq_num = seq_num_;
  (*sensor_objects)->sensor2world_pose = camera_to_car_;
  ((*sensor_objects)->camera_frame_supplement).reset(new CameraFrameSupplement);

  if (!CameraFrameSupplement::state_vars.initialized_) {
    CameraFrameSupplement::state_vars.process_noise *= 10;
    CameraFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
        0.0f, 0.33f, 0.0f;
    CameraFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
        1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in CameraFrameSupplement is \n"
           << CameraFrameSupplement::state_vars.trans_matrix << std::endl;
    CameraFrameSupplement::state_vars.initialized_ = true;
  }

  for (size_t i = 0; i < objects.size(); ++i) {
    std::shared_ptr<VisualObject> vobj = objects[i];
    std::shared_ptr<Object> obj(new Object());

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
    obj->anchor_point = obj->center;
    obj->state_uncertainty = vobj->state_uncertainty;

    (obj->camera_supplement).reset(new CameraSupplement());
    obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
    obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
    obj->camera_supplement->alpha = vobj->alpha;
    obj->camera_supplement->pts8 = vobj->pts8;

    ((*sensor_objects)->objects).emplace_back(obj);
  }
}

void CameraProcessFrontRightSubnode::PublishDataAndEvent(
    const double &timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
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

void CameraProcessFrontRightSubnode::PublishPerceptionPb(
    const SharedDataPtr<SensorObjects> &sensor_objects) {
  ADEBUG << "Camera publish perception pb data";
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

  common::adapter::AdapterManager::PublishPerceptionObstacles(obstacles);
  ADEBUG << "Camera Obstacles: " << obstacles.ShortDebugString();
}

}  // namespace perception
}  // namespace apollo
