/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef SCENE_MANAGER_H_
#define SCENE_MANAGER_H_

#include <map>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <wviz_tf_manager/PublishAllTransforms.h>
#include <tf/transform_listener.h>
#include <XmlRpcValue.h>

namespace wviz_tf_manager {

class FramePair
{
public:
  FramePair(const std::string& source_frame, const std::string& target_frame, double translational_update_distance, double angular_update_distance) :
    source_frame_(source_frame),
    target_frame_(target_frame),
    translational_update_distance_(translational_update_distance),
    angular_update_distance_(angular_update_distance)
  {
    pose_in_ = tf::Stamped<tf::Pose>(tf::Pose(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)), ros::Time(), source_frame_);
  }

public:
  std::string source_frame_;
  std::string target_frame_;

  tf::Stamped<tf::Pose> pose_in_;
  tf::Stamped<tf::Pose> pose_out_;
  tf::Stamped<tf::Pose> last_sent_pose_;

  double translational_update_distance_;
  double angular_update_distance_;
};

/**
 * @class TransformManager
 * @brief
 */
class TransformManager {
public:
  /**
   * @brief  Constructor
   * @return
   */
  TransformManager(ros::NodeHandle& node);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~TransformManager();

  /**
   * @brief
   */
  bool publishAllTransforms(wviz_tf_manager::PublishAllTransforms::Request& req, wviz_tf_manager::PublishAllTransforms::Response& resp);

  /**
   * @brief  Starts the server and spins
   */
  void spin();

  /**
   * @brief
   */
  void tfCallback(const tf::tfMessageConstPtr& msg_ptr);

  void publishAll();
  void publishChanged();

private:
  ros::NodeHandle node_;
  double polling_frequency_;
  double translational_update_distance_;
  double angular_update_distance_;

  boost::mutex frame_pairs_mutex_;
  std::map<std::string,FramePair> frame_pairs_;
  tf::TransformListener tfl_;
  ros::Publisher pub_;

  bool publish_all_;
  ros::ServiceServer publish_all_srv_;
  ros::Subscriber subscriber_tf_;
};

}

#endif

