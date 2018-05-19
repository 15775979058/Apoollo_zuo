#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/common/pose_util.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"
#include "modules/perception/obstacle/radar/modest/conti_radar_id_expansion.h"
#include "modules/perception/obstacle/radar/modest/modest_radar_detector.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/perception/obstacle/onboard/radar_process_common_subnode.h"


namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using Eigen::Affine3d;
using Eigen::Matrix4d;
using pcl_util::Point;
using pcl_util::PointD;
using std::string;
using std::unordered_map;

#define REGISTER_RADAR(name)                                                 \
class Radar##name##ProcessSubnode : public RadarProcessCommonSubnode         \
{                                                                            \
public:                                                                      \
  Radar##name##ProcessSubnode(){ SetSubnodeName(#name); };                   \
};                                                                           \
REGISTER_SUBNODE(Radar##name##ProcessSubnode);                               \


REGISTER_RADAR(Front);
REGISTER_RADAR(Left);

}  // namespace perception
}  // namespace apollo
