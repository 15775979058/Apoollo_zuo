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

#include "modules/monitor/hardware/can/kvasercan/kvasercan_checker.h"

#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "kvaser_can/include/canlib.h"

namespace apollo {
namespace monitor {
namespace hw {

const char KvaserCanChecker::KVASER_CAN_NAME[] = "KVASER_CAN";

KvaserCanChecker::KvaserCanChecker() {
  name_ = apollo::common::util::StrCat(KVASER_CAN_NAME, "-", can_id_);
}

HardwareStatus::Status KvaserCanChecker::kvasercan_result_to_hw_status(
    canStatus stat) {
  // @todo: device not present detection in kvaser_can_test.
  return stat == canOK ? HardwareStatus::OK : HardwareStatus::ERR;
}

std::string KvaserCanChecker::kvasercan_result_to_message(canStatus stat) {
  char str_buf[200];
   if (stat == canOK){
     return "OK";
   }
   else {
     str_buf[0] = '\0';
     canGetErrorText(stat, str_buf, sizeof(str_buf));
     return std::string((const char *)(str_buf));
   }

}

void KvaserCanChecker::run_check(std::vector<HwCheckResult> *results) {
  AINFO << "To check KVASER-CAN-" << can_id_;

  KvaserCanDetails *details = new KvaserCanDetails();
  canStatus result = details->kvasercan_do_test(can_id_);

  HwCheckResult rslt("CAN", kvasercan_result_to_hw_status(result), details,
                     std::move(kvasercan_result_to_message(result)));

  results->emplace_back(std::move(rslt));
}

}  // namespace hw
}  // namespace monitor
}  // namespace apollo
