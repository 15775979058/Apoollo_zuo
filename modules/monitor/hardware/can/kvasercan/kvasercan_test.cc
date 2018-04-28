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

#include "modules/monitor/hardware/can/kvasercan/kvasercan_test.h"

#include <ostream>

#include "modules/common/log.h"

namespace apollo {
namespace monitor {
namespace hw {

canStatus KvaserCanDetails::kvasercan_do_test(int id) {
  //CanHandle dev_handler_;
  canStatus stat;

  invalidate();
  canInitializeLibrary();
  int canlibversion=0;
  int chanCount =0;
  char name[256];

  canlibversion = canGetVersion();

  AINFO << "Kavaser Canlib version " << (canlibversion >> 8)<< "." << (canlibversion & 0xff);

  stat = canGetNumberOfChannels(&chanCount);
  if ( stat != canOK ) {

     AERROR << "Could not get the can number of channels";
     return stat;
  }
  else{
    AINFO << "Found " << chanCount << " channels.";
    for (int i = 0;i < chanCount;i++){
      memset(name, 0, sizeof(name));
      stat = canGetChannelData(i, canCHANNELDATA_DEVDESCR_ASCII,
                               &name, sizeof(name));
      if (stat != canOK) {
        AERROR << "Could not get the Card name of channel:" << i;
        return stat;
      }
      else{
        AINFO << "Channel :" << i << " the Card name is:" << name;
        return canOK;
      }
    }
  }










//  return canOK;
/*
  int port_ = 0;
  dev_handler_ = canOpenChannel(port_, canOPEN_EXCLUSIVE);
  if (dev_handler_ < 0){
      AERROR << "Could not open can channel :" << port_;
      result =  canERR_NOCHANNELS ;
      return  canERR_NOCHANNELS ;
  }

  stat = canSetBusParams(dev_handler_, canBITRATE_500K, 0, 0, 0, 0, 0);
  if (stat != canOK) {
    AERROR << "set baudrate error code: " << stat << ", " ;
    goto err;

  }
  else{
    AINFO << "set baudrate SUCCESS";
    stat = canBusOn(dev_handler_);
    if (stat != canOK) {
      AERROR << "can Bus On error code: " << stat << ", ";
      goto err;
    }
    else{
      AINFO << "success open can bus.";
      return stat;
    }

  }


err:
  canClose(dev_handler_);
  result = stat;
  return stat;
  */
}

void KvaserCanDetails::print_summary(std::ostream &os) {

  }


void KvaserCanDetails::print_test_result(std::ostream &os) {

}

}  // namespace hw
}  // namespace monitor
}  // namespace apollo
