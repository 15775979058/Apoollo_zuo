/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/perception.h"

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/onboard/camera_process_subnode.h"
#include "modules/perception/obstacle/onboard/fusion_subnode.h"
#include "modules/perception/obstacle/onboard/async_fusion_subnode.h"
#include "modules/perception/obstacle/onboard/lane_post_processing_subnode.h"
#include "modules/perception/obstacle/onboard/lane_shared_data.h"
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/obstacle/onboard/lidar_process_subnode.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/fusion_shared_data.h"
#include "modules/perception/obstacle/onboard/radar_process_subnode.h"
#include "modules/perception/obstacle/onboard/visualization_subnode.h"
#include "modules/perception/traffic_light/onboard/tl_preprocessor_subnode.h"
#include "modules/perception/traffic_light/onboard/tl_proc_subnode.h"

//-- Zuo add on 2018-04-03 --//
#include "modules/perception/obstacle/onboard/zuo_test_subnode.h"
//-- Zuo add on 2018-04-03 --//

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;

std::string Perception::Name() const { return "perception"; }

Status Perception::Init() {
  /**
   * Zuo commented on 2018-04-02
   * FLAGS_perception_adapter_config_filename是gflag的配置参数的方式。在Apollo里面一般在×××_flags.h里面进行申明
   * 这里参照/home/zuo/apollo_ws/apollo-0326/modules/perception/common/perception_gflags.cc :L19
   */
  AdapterManager::Init(FLAGS_perception_adapter_config_filename);

  //-- Zuo add a test info output on 2018-04-04 --//
  AINFO << "===== test info on perception.cc =====";
  //-- Zuo add a test info output on 2018-04-04 --//

  RegistAllOnboardClass();
  /// init config manager
  ConfigManager* config_manager = ConfigManager::instance();
  if (!config_manager->Init()) {
    AERROR << "failed to Init ConfigManager";
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init ConfigManager.");
  }
  AINFO << "Init config manager successfully, work_root: "
        << config_manager->WorkRoot();

  const std::string dag_config_path = apollo::common::util::GetAbsolutePath(
      FLAGS_work_root, FLAGS_dag_config_path);

  if (!dag_streaming_.Init(dag_config_path)) {
    AERROR << "failed to Init DAGStreaming. dag_config_path:"
           << dag_config_path;
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init DAGStreaming.");
  }
  callback_thread_num_ = 5;

  return Status::OK();
}

/**
 * Zuo commented on 2018-04-02
 * 例如RegisterFactoryLidarProcessSubnode
 * 在/perception/obstacle/onboard/lidar_process_subnode.h :L89
 * 即每个模块在其对应的头文件内定义好了，函数使用的##拼接的方法，在register.h，不过又被封装了几层
 * 但是没有找到RegisterFactoryLidarObjectData这类型的在哪里
 */
void Perception::RegistAllOnboardClass() {
  /// regist sharedata
  RegisterFactoryLidarObjectData();
  RegisterFactoryRadarObjectData();
  RegisterFactoryCameraObjectData();
  RegisterFactoryCameraSharedData();
  RegisterFactoryLaneSharedData();
  RegisterFactoryFusionSharedData();
  traffic_light::RegisterFactoryTLPreprocessingData();

  /// regist subnode
  RegisterFactoryLidarProcessSubnode();
  RegisterFactoryRadarProcessSubnode();
  RegisterFactoryCameraProcessSubnode();
  //-- Zuo add on 2018-04-03 --//
  RegisterFactoryZuoTestSubnode();
  AINFO << " Zuo Register TestSubnode successfull ";
  //-- Zuo add on 2018-04-03 --//
  RegisterFactoryLanePostProcessingSubnode();
  RegisterFactoryAsyncFusionSubnode();
  RegisterFactoryFusionSubnode();
  RegisterFactoryMotionService();
  lowcostvisualizer::RegisterFactoryVisualizationSubnode();
  traffic_light::RegisterFactoryTLPreprocessorSubnode();
  traffic_light::RegisterFactoryTLProcSubnode();
}

Status Perception::Start() {
  dag_streaming_.Start();
  return Status::OK();
}

void Perception::Stop() {
  dag_streaming_.Stop();
  dag_streaming_.Join();
}

}  // namespace perception
}  // namespace apollo
