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

#ifndef STOP_MONITOR_H
#define STOP_MONITOR_H

#include <deque>
#include <map>
#include <sensor_msgs/JointState.h>

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{

class StopMonitor
{
public:
  StopMonitor();

  void addState(const sensor_msgs::JointStateConstPtr& msg);
  /// Check to see if the queue indicates that the robot is stopped
  /// and we haven't reported that yet.
  bool justStopped();

  void setTimeWindow(double seconds) { time_window_ = seconds; }
  double getTimeWindow() const { return time_window_; }

  void setThreshold(double threshold) { threshold_ = threshold; }
  double getThreshold() const { return threshold_; }

private:
  struct Entry
  {
    double time_;
    std::vector<double> joint_values_;
  };

  void initialize(const sensor_msgs::JointStateConstPtr& msg);
  Entry msgToEntry(const sensor_msgs::JointStateConstPtr& msg);

  std::map<std::string, size_t> joint_to_index_map_;
  std::deque<Entry> history_;
  double time_window_;
  double threshold_;
  bool is_stopped_;
};

} // end namespace joint_trajectory_interface
} // end namespace industrial_robot_client

#endif // STOP_MONITOR_H
