/*
 * 2016 Bernd Pfrommer
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cam_imu_sync/cam_imu_sync_nodelet.h"

namespace cam_imu_sync {
void CamImuSyncNodelet::onInit() {
  camImuSynchronizer_.reset(new CamImuSynchronizer(getPrivateNodeHandle()));
}

PLUGINLIB_EXPORT_CLASS(cam_imu_sync::CamImuSyncNodelet, nodelet::Nodelet);

}  // namespace cam_imu_sync
