/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */
#include "wviz_tf_manager/wviz_tf_manager.h"

namespace wviz_tf_manager {

TransformManager::TransformManager(ros::NodeHandle& node)
: node_(node), publish_all_(false)
{
  ros::NodeHandle local_node("~");

  local_node.param(std::string("polling_frequency"),             polling_frequency_,              10.0);
  local_node.param(std::string("translational_update_distance"), translational_update_distance_,  0.10);
  local_node.param(std::string("angular_update_distance"),       angular_update_distance_,        0.10);

  // subscribe to tf messages
  subscriber_tf_ = node_.subscribe<tf::tfMessage>("tf", 100, boost::bind(&TransformManager::tfCallback, this, _1));

  // Advertise the service
  pub_ = node.advertise<tf::tfMessage>("tf_changes", 1, true);
  publish_all_srv_ = local_node.advertiseService("publish_all_transforms", &TransformManager::publishAllTransforms, this);
}

TransformManager::~TransformManager()
{
}

void TransformManager::tfCallback(const tf::tfMessageConstPtr& msg_ptr)
{
  const tf::tfMessage& message = *msg_ptr;
  boost::mutex::scoped_lock my_lock(frame_pairs_mutex_);
  for (unsigned int i = 0; i < message.transforms.size(); i++)
  {
    std::string frame_id = message.transforms[i].header.frame_id;
    std::string child_frame_id = message.transforms[i].child_frame_id;
    std::string id = frame_id + "<--" + child_frame_id;
    if(frame_pairs_.find(id) == frame_pairs_.end()) {
      ROS_INFO("Adding frame pair %s",id.c_str());
      FramePair frame_pair(child_frame_id, frame_id, translational_update_distance_, angular_update_distance_);
      frame_pairs_.insert(std::make_pair(id,frame_pair));
    }
  }
}

bool TransformManager::publishAllTransforms(wviz_tf_manager::PublishAllTransforms::Request& req, wviz_tf_manager::PublishAllTransforms::Response& resp)
{
  publish_all_ = true;
  return true;
}


void TransformManager::publishAll()
{
  tf::tfMessage msg;
  for (std::map<std::string,FramePair>::iterator i = frame_pairs_.begin(); i != frame_pairs_.end(); i++)
  {
   FramePair& fp = i->second;

   tfl_.transformPose(fp.target_frame_, fp.pose_in_, fp.pose_out_);

   const tf::Vector3&    origin   = fp.pose_out_.getOrigin();
   const tf::Quaternion& rotation = fp.pose_out_.getRotation();

   tf::StampedTransform stampedTf(tf::Transform(rotation, origin), fp.pose_out_.stamp_, fp.target_frame_, fp.source_frame_);
   geometry_msgs::TransformStamped msgtf;
   transformStampedTFToMsg(stampedTf, msgtf);
   msg.transforms.push_back(msgtf);
  }

  if (msg.transforms.size() > 0)
   pub_.publish(msg);
}

void TransformManager::publishChanged()
{
  tf::tfMessage msg;
  for (std::map<std::string,FramePair>::iterator i = frame_pairs_.begin(); i != frame_pairs_.end(); i++)
  {
    FramePair& fp = i->second;

    tfl_.transformPose(fp.target_frame_, fp.pose_in_, fp.pose_out_);

    const tf::Vector3&    origin   = fp.pose_out_.getOrigin();
    const tf::Quaternion& rotation = fp.pose_out_.getRotation();

    if (origin.distance(fp.last_sent_pose_.getOrigin()) > fp.translational_update_distance_ ||
        rotation.angle(fp.last_sent_pose_.getRotation()) > fp.angular_update_distance_)
    {
      fp.last_sent_pose_ = fp.pose_out_;

      tf::StampedTransform stampedTf(tf::Transform(rotation, origin), fp.pose_out_.stamp_, fp.target_frame_, fp.source_frame_);
      geometry_msgs::TransformStamped msgtf;
      transformStampedTFToMsg(stampedTf, msgtf);
      msg.transforms.push_back(msgtf);
    }
  }

  if (msg.transforms.size() > 0)
    pub_.publish(msg);
}

void TransformManager::spin()
{

  while (node_.ok())
  {
    try
    {
      boost::mutex::scoped_lock my_lock(frame_pairs_mutex_);

      if(publish_all_) {
        publishAll();
        publish_all_ = false;
      }
      else {
        publishChanged();
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_DEBUG("Exception: %s\n", ex.what());
    }
    
    // Sleep until next polling
    if (polling_frequency_ > 0)
      ros::Duration().fromSec(1.0 / polling_frequency_).sleep();
  }
}

}

/** This is a program to provide notifications of changes of state within tf
 * It was written for providing an easy way to on demand update a web graphic of
 * where the robot is located.  It's not designed or recommended for use in live
 * operation for feedback.  */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wviz_tf_manager");

  ros::NodeHandle nh;
  boost::thread spinner( boost::bind( &ros::spin ));
  wviz_tf_manager::TransformManager manager(nh);
  manager.spin();
  spinner.join();
  return 0;
}
