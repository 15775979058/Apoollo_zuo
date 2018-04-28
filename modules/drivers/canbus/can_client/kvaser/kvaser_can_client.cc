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
 * @file kvaser_can_client.cc
 * @brief the encapsulate call the api of kvaser can card according to can_client.h
 *interface
 **/

#include "modules/drivers/canbus/can_client/kvaser/kvaser_can_client.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool kvaserCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }
  port_ = parameter.channel_id();
  /*This function must be called before any other functions is used. It will initialize the driver.

You may call canInitializeLibrary() more than once. The actual initialization will take place only once.

Any errors encountered during library initialization will be "silent" and an appropriate canERR_xxx error code
 will be returned later on when canOpenChannel() (or any other API call that requires initialization) is called.
*/
  canInitializeLibrary();
  return true;
}

kvaserCanClient::~kvaserCanClient() {
  if (dev_handler_) {
    Stop();
  }
}

ErrorCode kvaserCanClient::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }

/*Opens a CAN channel (circuit) and returns a handle which is used in subsequent calls to CANLIB.

Channel numbering is dependent on the installed hardware. The first channel always has number 0.

For example,

If you have a single LAPcan, the channels are numbered 0 and 1.
If you have a USBcan Professional, the channels are numbered 0-1 according to the labels on the cables.
The virtual channels come after all physical channels.
If you are using multiple threads, note that the returned handle is usable only in the context of the
thread that created it. That is, you must call canOpenChannel() in each of the threads in your application
that uses the CAN bus. You can open the same channel from multiple threads, but you must call canOpenChannel() once per thread.

If you are using the same channel via multiple handles, note that the default behaviour is that the different
handles will "hear" each other just as if each handle referred to a channel of its own. If you open, say, channel 0
from thread A and thread B and then send a message from thread A, it will be "received" by thread B.
This behaviour can be changed using canIOCTL_SET_LOCAL_TXECHO.

#define 	canWANT_EXCLUSIVE   0x0008

#define 	canWANT_EXTENDED   0x0010

#define 	canWANT_VIRTUAL   0x0020

#define 	canOPEN_EXCLUSIVE   0x0008

#define 	canOPEN_REQUIRE_EXTENDED   0x0010

#define 	canOPEN_ACCEPT_VIRTUAL   0x0020

#define 	canOPEN_ACCEPT_LARGE_DLC   0x0200

#define 	canOPEN_CAN_FD   0x0400

#define 	canOPEN_CAN_FD_NONISO   0x0800

#define 	canOPEN_LIN   0x1000

*/
//  uint32_t mode = 0;
  // mode |= NTCAN_MODE_NO_RTR;
  if (port_ > MAX_CAN_PORT || port_ < 0) {
    AERROR << "can port number [" << port_ << "] is out of the range [0,"
           << MAX_CAN_PORT << "]";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  //int32_t ret = canOpen(port_, mode, NTCAN_MAX_TX_QUEUESIZE,
//                        NTCAN_MAX_RX_QUEUESIZE, 5, 5, &dev_handler_);


  dev_handler_ = canOpenChannel(port_, canOPEN_EXCLUSIVE);


  stat = canSetBusParams(dev_handler_, canBITRATE_500K, 0, 0, 0, 0, 0);
  if (stat != canOK) {
    AERROR << "set baudrate error code: " << stat << ", " << GetErrorString(stat);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  stat = canBusOn(dev_handler_);
  if (stat != canOK) {
    AERROR << "can Bus On error code: " << stat << ", " << GetErrorString(stat);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  is_started_ = true;
  return ErrorCode::OK;


}

void kvaserCanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    stat = canClose(dev_handler_);
    if (stat != canOK) {
      AERROR << "close error code:" << stat << ", " << GetErrorString(stat);
    } else {
      AINFO << "close kvaser can ok. port:" << port_;
    }
  }
}

// Synchronous transmission of CAN messages
ErrorCode kvaserCanClient::Send(const std::vector<CanFrame> &frames,
                             int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_started_) {
    AERROR << "kvaser can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
  //  send_frames_[i].id = frames[i].id;
  //  send_frames_[i].len = frames[i].len;
  //char data[8];
  //std::memcpy(data,frames[i].data, 8);
  // Synchronous transmission of CAN messages
  //stat = canWrite(dev_handler_, frames[i].id,reinterpret_cast<void*>(data), frames[i].len, canMSG_STD);
  stat = canWrite(dev_handler_, frames[i].id,const_cast<uint8_t*>(frames[i].data), frames[i].len, canMSG_STD);
    if ( stat != canOK ) {
      AERROR << "send message failed, error code: " << stat << ", "
             << GetErrorString(stat);
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
    stat = canWriteSync(dev_handler_,10);
    if ( stat != canOK ) {
      AERROR << "send message wait Sync failed, error code: " << stat << ", "
             << GetErrorString(stat);
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
  }


  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode kvaserCanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "kvaser can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (*frame_num > MAX_KVASER_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_KVASER_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  stat = canReadWait(dev_handler_, &recv_frames_[0].id, recv_frames_[0].data,
                                 &recv_frames_[0].len, &recv_frames_[0].flag, &recv_frames_[0].time, recv_frames_[0].time_out);
  // rx timeout not log
//  if (ret == NTCAN_RX_TIMEOUT) {
//    return ErrorCode::OK;
//  }

    if ( stat != canOK ) {
       if (stat == canERR_NOMSG ){
          AERROR << "receive no message,error code: " << stat << ", "
                 << GetErrorString(stat);
          return ErrorCode::OK;
        }
       else{
       AERROR << "receive message failed, error code: " << stat << ", "
              << GetErrorString(stat);
       return ErrorCode::CAN_CLIENT_ERROR_BASE;
      }
  }
  else {
    if (recv_frames_[0].len != 8){
      AERROR << "receive frame message more than one ,the total frame len is:" << recv_frames_[0].len;
    }

  }

  for (int32_t i = 0; i < *frame_num && i < MAX_KVASER_CAN_RECV_FRAME_LEN; ++i) {
    CanFrame cf;
    cf.id = recv_frames_[i].id;
    cf.len = recv_frames_[i].len;
    std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].len);
    frames->push_back(cf);
  }

  return ErrorCode::OK;
}

/************************************************************************/
/************************************************************************/
/* Function: GetErrorString()                                            */
/* Return ASCII representation of NTCAN return code                     */
/************************************************************************/
/************************************************************************/
const int32_t ERROR_BUF_SIZE = 200;
std::string kvaserCanClient::GetErrorString(const canStatus stat) {

  char str_buf[ERROR_BUF_SIZE];
  str_buf[0] = '\0';
  canGetErrorText(stat, str_buf, sizeof(str_buf));
  return std::string((const char *)(str_buf));

}

std::string kvaserCanClient::GetErrorString(const int32_t status) {

  int8_t str_buf[ERROR_BUF_SIZE];


  return std::string((const char *)(str_buf));
}



}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
