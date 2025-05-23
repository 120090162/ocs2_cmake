/*
Author: Yuntian Zhao
Last Date Modified: 05/13/2025 by YZ

Description:
This is the topic receive wrapper of the internode topic interface that to be used in ocs2_cmake
*/

#pragma once

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_comm_wrapper/TopicReceiverWrapper.h>

namespace ocs2 {

class DummyTargetTrajectoriesReceiver : public TopicReceiverWrapper<TargetTrajectories> {
public:
    DummyTargetTrajectoriesReceiver() = default;
    ~DummyTargetTrajectoriesReceiver() override = default;

    void init() override {
        // Leave blank for now
    }

    void getMsg(TargetTrajectories& msg) override {
        // Leave blank for now
    }

    void setCallback(std::function<void(const TargetTrajectories&)> callbackFunction) override {
        // Leave blank for now
    }
};

class DummyModeScheduleReceiver : public TopicReceiverWrapper<ModeSchedule> {
public:
    DummyModeScheduleReceiver() = default;
    ~DummyModeScheduleReceiver() override = default;

    void init() override {
        // Leave blank for now
    }

    void getMsg(ModeSchedule& msg) override {
        // Leave blank for now
    }

    void setCallback(std::function<void(const ModeSchedule&)> callbackFunction) override {
        // Leave blank for now
    }
};

} // namespace ocs2