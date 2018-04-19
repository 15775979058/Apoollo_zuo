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

#include "modules/perception/obstacle/onboard/zuo_test_2_subnode.h"

#include <unordered_map>

#include "modules/perception/traffic_light/util/color_space.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;

bool ZuoTest2Subnode::InitInternal() {

  AINFO << "===== Zuo test_2_subnode info on InitInternal =====";

  return true;
}

/**
 * 发送一张图到zuo_test_subnode显示，测试msg的publish/subscribe
 * Zuo added on 2018-04-16
 */
cv::Mat img = cv::imread("/apollo/test.jpg", CV_LOAD_IMAGE_COLOR);

apollo::common::Status ZuoTest2Subnode::ProcEvents() {

  // std::mutex camera_mutex_;
  // std::lock_guard<std::mutex> lock(camera_mutex_);

  cv_bridge::CvImage cvi;
  cvi.header.stamp = ros::Time::now(); 
  cvi.header.frame_id = "image";
  cvi.encoding = "bgr8";
  cvi.image = img;

  sensor_msgs::Image msg;  

  if( !img.empty() ) {

    cv::imshow("publish img", img);
    cv::waitKey(0);

    cvi.toImageMsg(msg);

    // apollo::common::adapter::AdapterManager::FillZuoTestHeader("ZuoTest2Subnode",&msg);
    // common::adapter::AdapterManager::PublishZuoTest(msg);

    ZuoTestMsg Zuomsg;
    Zuomsg.set_info("hello world");
    common::adapter::AdapterManager::PublishZuoTestMsg(Zuomsg);
  }

  return apollo::common::Status::OK();
}

}  // namespace perception
}  // namespace apollo
