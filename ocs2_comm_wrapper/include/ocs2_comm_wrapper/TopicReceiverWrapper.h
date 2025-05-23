/*
Author: Yuntian Zhao
Last Date Modified: 05/23/2025 by YZ

Description:
This is the topic receive wrapper of the internode topic interface that to be used in ocs2_cmake
*/

#pragma once

#include <string>
#include <functional>

namespace ocs2 {

template<typename T>
class TopicReceiverWrapper
{
public:
    TopicReceiverWrapper() = default;
    virtual ~TopicReceiverWrapper() = default;

    virtual void setTopicName(const std::string& topicName) {
        topicName_ = topicName;
    }

    // Register a callback to be called on message receive
    virtual void setCallback(std::function<void(const T&)> callbackFunction) = 0;

    virtual void init() = 0;

    // Polling-style access to last message
    virtual void getMsg(T& msg) = 0;

protected:
    std::string topicName_;
};

} // namespace ocs2