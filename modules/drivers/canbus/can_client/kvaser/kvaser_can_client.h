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

/**
 * @file
 * @brief Defines the kvaserCanClient class which inherits CanClient.
 */

#ifndef MODULES_DRIVERS_CANBUS_CAN_CLIENT_CLIENT_KVASER_CAN_CLIENT_H_
#define MODULES_DRIVERS_CANBUS_CAN_CLIENT_CLIENT_KVASER_CAN_CLIENT_H_

#include <string>
#include <vector>
#include "kvaser_can/include/canlib.h"
#include "kvaser_can/include/canstat.h"
#include "kvaser_can/include/obsolete.h"
#include "gflags/gflags.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"

/**
 * @namespace apollo::drivers::canbus::can
 * @brief apollo::drivers::canbus::can
 */
namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

/**
 * @class kvaserCanClient
 * @brief The class which defines a kvaser CAN client which inherits CanClient.
 */
class kvaserCanClient : public CanClient {
 public:
  /**
   * @brief Initialize the kvaser CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter &parameter) override;

  /**
   * @brief Destructor
   */
  virtual ~kvaserCanClient();

  /**
   * @brief Start the kvaser CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Start() override;

  /**
   * @brief Stop the kvaser CAN client.
   */
  void Stop() override;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Send(const std::vector<CanFrame> &frames,
                                 int32_t *const frame_num) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Receive(std::vector<CanFrame> *const frames,
                                    int32_t *const frame_num) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const canStatus status) ;
  std::string GetErrorString(const int32_t status) override;

 private:
  CanHandle dev_handler_;
  canStatus stat;
  CANCardParameter::CANChannelId port_;
  Kvaser_MSG send_frames_[MAX_CAN_SEND_FRAME_LEN];
  Kvaser_MSG recv_frames_[MAX_CAN_RECV_FRAME_LEN];
};

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_CANBUS_CAN_CLIENT_CLIENT_kvaser_CAN_CLIENT_H_
