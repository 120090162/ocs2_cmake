/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
modify the original sqp mpc node to test the new ocs2_cmake
*/


#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include "ocs2_legged_robot_test/DummyReceiver.h"
#include "ocs2_legged_robot_test/synchronized_module/ReferenceReceiver.h"
#include "ocs2_legged_robot_test/gait/GaitReceiver.h"

#include <chrono>
#include <thread>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  std::string taskFile = "/home/zyt/ocs2_cmake/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info";
  std::string urdfFile = "/home/zyt/ocs2_cmake/ocs2_robotic_examples/ocs2_legged_robot_test/urdf/anymal.urdf";
  std::string gaitFile = "/home/zyt/ocs2_cmake/ocs2_robotic_examples/ocs2_legged_robot/config/command/gait.info";
  std::string referenceFile = "/home/zyt/ocs2_cmake/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info";

  // Robot interface
  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

  // Gait receiver
  auto dummyModeScheduleReceiverPtr = 
         std::make_shared<DummyModeScheduleReceiver>();
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(dummyModeScheduleReceiverPtr, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ReferenceReceiver
  auto dummyTargetTrajectoriesReceiverPtr = 
         std::make_shared<DummyTargetTrajectoriesReceiver>();

  auto referenceReceiverPtr = std::make_shared<ReferenceReceiver>(robotName, interface.getReferenceManagerPtr(),
         dummyTargetTrajectoriesReceiverPtr);

  const auto initModeSchedule = ocs2::legged_robot::loadModeSchedule(referenceFile, "initialModeSchedule", false);
  auto defaultModeSequenceTemplate = ocs2::legged_robot::loadModeSequenceTemplate(gaitFile, "trot", false);
  double phaseTransitionStanceTime = 0.2;
  auto gaitSchedulePtr = std::make_shared<ocs2::legged_robot::GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, phaseTransitionStanceTime);

  // MPC
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(referenceReceiverPtr);
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

  auto trotModeSchedule = gaitSchedulePtr->getModeSchedule(0, 1.5);
  std::cout << "trotModeSchedule is " << trotModeSchedule << "\n";
  mpc.getSolverPtr()->getReferenceManager().setModeSchedule(trotModeSchedule);

  std::cout << "successfully created interface and mpc(ocp)\n";

  SystemObservation currentObservation;
  auto stateDim = interface.getCentroidalModelInfo().stateDim;
  auto inputDim = interface.getCentroidalModelInfo().inputDim;
  vector_t state(stateDim), input(inputDim);
  currentObservation.state = state;
  currentObservation.input = input;

  vector_t init_state(stateDim), target_state(stateDim);

  init_state << interface.getInitialState();
  target_state << interface.getInitialState();

  target_state(6) = 0.5;

  mpc.getSolverPtr()->getReferenceManager().setTargetTrajectories(TargetTrajectories(
       {0.0, 1.5}, {init_state, target_state}, {vector_t::Zero(inputDim), vector_t::Zero(inputDim)}
  ));

  MPC_MRT_Interface mrt(mpc);
  mrt.initRollout(&interface.getRollout());
  mrt.setCurrentObservation(currentObservation);
  std::cout << "Waiting for the initial policy ...\n";
  double frequency = interface.mpcSettings().mrtDesiredFrequency_;
  std::chrono::duration<double> period(1.0 / frequency);
  auto next = std::chrono::steady_clock::now() + period;
  while (!mrt.initialPolicyReceived())
  {
       mrt.advanceMpc();
       std::this_thread::sleep_until(next);
       next += period;
  }
  std::cout << "Initial policy has been received.\n";
  mrt.updatePolicy();
  auto primarySol = mrt.getPolicy();
  std::cout << "primarySol.stateTrajectory_ is: \n";
  for (auto i = 0; i != primarySol.stateTrajectory_.size() - 1; ++i)
       std::cout << primarySol.stateTrajectory_[i] << " ";
  std::cout << "\n";
  return 0;
}
