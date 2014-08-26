/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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
/* Author: Dave Hershberger */

#include "motoman_driver/industrial_robot_client/stop_monitor.h"
#include <ros/ros.h>

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{

StopMonitor::StopMonitor()
  : time_window_(1.0)
  , threshold_(1e-5)
{
}

void StopMonitor::addState(const sensor_msgs::JointStateConstPtr& msg)
{
  if(history_.size() == 0 && msg->name.size() > 0)
  {
    initialize(msg);
  }
  Entry new_entry = msgToEntry(msg);
  history_.push_back(new_entry);
  while(history_.size() > 0 && new_entry.time_ - history_.front().time_ > time_window_)
  {
    history_.pop_front();
  }
}

void StopMonitor::initialize(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_to_index_map_.clear();
  for(size_t i = 0; i < msg->name.size(); i++)
  {
    joint_to_index_map_[msg->name[i]] = i;
  }
}

StopMonitor::Entry StopMonitor::msgToEntry(const sensor_msgs::JointStateConstPtr& msg)
{
  Entry entry;
  entry.time_ = msg->header.stamp.toSec();
  entry.joint_values_.resize(joint_to_index_map_.size(), 0);
  for(size_t i = 0; i < msg->name.size(); i++)
  {
    std::map<std::string, size_t>::const_iterator mi = joint_to_index_map_.find(msg->name[i]);
    if(mi != joint_to_index_map_.end())
    {
      double index = mi->second;
      entry.joint_values_[index] = msg->position[i];
    }
    else
    {
      ROS_WARN("StopMonitor received unknown joint name '%s', ignoring.", msg->name[i].c_str());
    }
  }
  return entry;
}

bool StopMonitor::justStopped()
{
  if(history_.size() == 0)
  {
    is_stopped_ = false;
    return false;
  }
  Entry most_recent = history_.back();
  
  for(size_t hi = 0; hi + 1 < history_.size(); hi++)
  {
    Entry entry = history_[hi];
    for(size_t i = 0; i < entry.joint_values_.size(); i++)
    {
      if(fabs(entry.joint_values_[i] - most_recent.joint_values_[i]) > threshold_)
      {
        is_stopped_ = false;
        return false;
      }
    }
  }
  // If it was already stopped, return false because it didn't stop
  // just now.
  if(is_stopped_)
  {
    return false;
  }
  is_stopped_ = true;
  return true;
}

} //joint_trajectory_interface
} //industrial_robot_client
