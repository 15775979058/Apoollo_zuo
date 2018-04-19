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

#include "modules/perception/obstacle/onboard/zuo_test_subnode.h"

// #include <unordered_map>

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;


bool ZuoTestSubnode::InitInternal() {

  AINFO << "===== ZuoTestSubnode::InitInternal =====";
  AINFO << "===== Zuo callback queue size before Add = "
        << AdapterManager::PopZuoTestMsgCallback();

  // AdapterManager::AddImageShortCallback(&ZuoTestSubnode::ZuoImgTestCallback, this);
  AdapterManager::AddZuoTestMsgCallback(&ZuoTestSubnode::ZuoImgTestCallback, this);

  AINFO << "===== ZuoTest callback queue size after Add = "
        << AdapterManager::PopZuoTestMsgCallback();

  return true;
}

void ZuoTestSubnode::ZuoImgTestCallback(const apollo::perception::ZuoTestMsg &message) {

  AINFO << " ZuoImgTestCallback";

  // common::adapter::AdapterManager::PublishZuoTest(message);

  // cv::Mat img;
  // MessageToMat(message, &img);

  // if( !img.empty() ) {
  //   AINFO << " ZuoImgTestCallback's img size = " << img.size();
  //   cv::imshow("recieve img", img);
  //   cv::waitKey(1);
  // }
}

bool ZuoTestSubnode::MessageToMat(const sensor_msgs::Image &msg, cv::Mat *img) {
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

}  // namespace perception
}  // namespace apollo
