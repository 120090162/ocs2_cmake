/*
Author: Yuntian Zhao
Last Date Modified: 05/13/2025 by YZ

Description:
This is the topic publish wrapper of the internode topic interface that to be used in ocs2_cmake
*/

#pragma once

#include <string>

namespace ocs2 {

template<typename T>
class TopicPublisherWrapper
{
public:
    TopicPublisherWrapper() = default;
    virtual ~TopicPublisherWrapper() = default;

    virtual void setTopicName(const std::string& topicName) {
        topicName_ = topicName;
    }

    virtual void init() = 0;

    virtual bool pubMsg(const T& msg) = 0;

protected:
    std::string topicName_;
};

} // namespace ocs2