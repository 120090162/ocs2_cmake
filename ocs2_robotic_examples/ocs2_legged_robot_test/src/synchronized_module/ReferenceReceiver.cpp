/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/*
Author: Farbod Farshidian, Yuntian Zhao
Last Date Modified: 05/13/2025 by YZ

Description:
Modified from https://github.com/leggedrobotics/ocs2/blob/main/ocs2_ros_interfaces/src/synchronized_module/RosReferenceManager.cpp
This is the reference updater that will update the reference trajectory to mpc
*/

#include "ocs2_legged_robot_test/synchronized_module/ReferenceReceiver.h"

// #include "ocs2_ros_interfaces/common/RosMsgConversions.h"

// #include <ros/transport_hints.h>

// // MPC messages
// #include <ocs2_msgs/mode_schedule.h>
// #include <ocs2_msgs/mpc_target_trajectories.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ReferenceReceiver::ReferenceReceiver(std::string topicPrefix, std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
  std::shared_ptr<TopicReceiverWrapper<TargetTrajectories>> targetTrajectoriesReceiverPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)), topicPrefix_(std::move(topicPrefix)),
      targetTrajectoriesReceiverPtr_(std::move(targetTrajectoriesReceiverPtr)) { }

// ReferenceReceiver::ReferenceReceiver(std::string topicPrefix, std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
//   std::shared_ptr<TopicReceiverWrapper<TargetTrajectories>> targetTrajectoriesReceiverPtr,
//   std::shared_ptr<TopicReceiverWrapper<ModeSchedule>> modeScheduleReceiverPtr)
//     : ReferenceManagerDecorator(std::move(referenceManagerPtr)), topicPrefix_(std::move(topicPrefix)),
//       targetTrajectoriesReceiverPtr_(std::move(targetTrajectoriesReceiverPtr)),
//       modeScheduleReceiverPtr_(std::move(modeScheduleReceiverPtr)) { }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ReferenceReceiver::subscribe() {
  // // ModeSchedule
  // auto modeScheduleCallback = [this](const ModeSchedule& msg) {
  //   referenceManagerPtr_->setModeSchedule(std::move(msg));
  // };
  // modeScheduleReceiverPtr_->setCallback(modeScheduleCallback);

  // TargetTrajectories
  auto targetTrajectoriesCallback = [this](const TargetTrajectories& msg) {
    referenceManagerPtr_->setTargetTrajectories(std::move(msg));
  };
  targetTrajectoriesReceiverPtr_->setCallback(targetTrajectoriesCallback);
}

}  // namespace ocs2
