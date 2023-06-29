#ifndef _ATTACHER_H__
#define _ATTACHER_H__

#include "ros/ros.h"
#include <string>
#include <gazebo/common/Plugin.hh>
#include "gazebo_ros_link_attacher/gazebo_ros_link_attacher.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"

namespace Attacher {
    void attach(std::string block, gazebo_ros_link_attacher::AttachRequest req, gazebo_ros_link_attacher::AttachResponse res) {
        req.model_name_1 = "robot";
        req.link_name_1 = "wrist_3_link";
        req.model_name_2 = block;
        req.link_name_2 = "link";

        ros::service::waitForService("/link_attacher_node/attach", ros::Duration(-1));
        ros::service::call("/link_attacher_node/attach", req, res);

        sleep(1);
    }

    void detach(std::string block, gazebo_ros_link_attacher::AttachRequest req, gazebo_ros_link_attacher::AttachResponse res) {
        req.model_name_1 = "robot";
        req.link_name_1 = "wrist_3_link";
        req.model_name_2 = block;
        req.link_name_2 = "link";

        ros::service::waitForService("/link_attacher_node/detach", ros::Duration(-1));
        ros::service::call("/link_attacher_node/detach", req, res);

        sleep(1);
    }
}

#endif
