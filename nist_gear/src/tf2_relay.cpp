/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
 *
*/

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>


bool
string_in(const std::string & needle, const std::vector<std::string> & haystack)
{
  return haystack.cend() != std::find(haystack.cbegin(), haystack.cend(), needle);
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ariac_tf_relay");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<tf2_msgs::TFMessage>("out/tf", 100);
  ros::Publisher pub_static = nh.advertise<tf2_msgs::TFMessage>("out/tf_static", 100, true);

  std::string prefix;
  nh.param("prefix", prefix, std::string("foobar_"));

  std::vector<std::string> frame_list;
  nh.getParam("frames", frame_list);

  if (frame_list.empty()) {
    ROS_WARN("'frames' param is empty, rewriting all frames");
  }

  ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>("in/tf", 100,
    [prefix, &pub, frame_list](const boost::shared_ptr<const tf2_msgs::TFMessage> message)
  {
    tf2_msgs::TFMessage output_msg;
    for (const auto & transform : message->transforms) {
      // Append prefix to frame names
      geometry_msgs::TransformStamped output_tf = transform;
      if (frame_list.empty() || string_in(transform.header.frame_id, frame_list)) {
        output_tf.header.frame_id = prefix + transform.header.frame_id;
      } else {
        output_tf.header.frame_id = transform.header.frame_id;
      }
      if (frame_list.empty() || string_in(transform.child_frame_id, frame_list)) {
        output_tf.child_frame_id = prefix + transform.child_frame_id;
      } else {
        output_tf.child_frame_id = transform.child_frame_id;
      }

      output_msg.transforms.push_back(output_tf);
    }
    pub.publish(output_msg);
  });


  ros::Subscriber sub_static = nh.subscribe<tf2_msgs::TFMessage>("in/tf_static", 100,
    [prefix, &pub_static, frame_list](const boost::shared_ptr<const tf2_msgs::TFMessage> message)
  {
    static tf2_msgs::TFMessage static_tf_msg;

    for (const auto & transform : message->transforms) {
      // Append prefix to frame names
      geometry_msgs::TransformStamped output_tf = transform;
      if (frame_list.empty() || string_in(transform.header.frame_id, frame_list)) {
        output_tf.header.frame_id = prefix + transform.header.frame_id;
      } else {
        output_tf.header.frame_id = transform.header.frame_id;
      }
      if (frame_list.empty() || string_in(transform.child_frame_id, frame_list)) {
        output_tf.child_frame_id = prefix + transform.child_frame_id;
      } else {
        output_tf.child_frame_id = transform.child_frame_id;
      }

      // Did the transform already exist in the message?
      auto iter = static_tf_msg.transforms.begin();
      for (; iter != static_tf_msg.transforms.end(); ++iter) {
        if (iter->header.frame_id == output_tf.header.frame_id
            && iter->child_frame_id == output_tf.child_frame_id)
        {
          // This transform already exists
          break;
        }
      }
      if (iter == static_tf_msg.transforms.end()) {
        // Insert new transform since one didn't already exist
        static_tf_msg.transforms.push_back(output_tf);
      } else {
        // Replace existing transform with same frame names
        iter = static_tf_msg.transforms.insert(iter, output_tf);
        ++iter;
        static_tf_msg.transforms.erase(iter);
      }
    }
    pub_static.publish(static_tf_msg);
  });

  ros::spin();

  return 0;
}
