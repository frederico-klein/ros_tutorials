/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
	//This file has been havily modified to serve as an example on how you could
	//use multithreading in ROS to do a round-robin task divider. It works well in this example
	//because the time is deterministic, in a real world scenario, you'd have to also
	//adjust the order of the arriving messages somehow.


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/thread.hpp>

	/**
	 * This tutorial demonstrates simple receipt of messages over the ROS system, using
	 * a threaded Spinner object to receive callbacks in multiple threads at the same time.
	 */

	ros::Duration d(10);

	class Listener
	{
		public:
			ros::NodeHandle node_handle_;
			ros::V_Subscriber subs_;
			ros::Subscriber main_subs_;
			std::vector<ros::Publisher> pubs_;
			ros::Publisher outcome_pub;
			int counter = 0;
			int num_processes_;
			Listener(const ros::NodeHandle& node_handle, const int num_processes)
				: node_handle_(node_handle), num_processes_(num_processes)
			{
				main_subs_ = node_handle_.subscribe("chatter",1000, &Listener::roundRobin, this);
				for (int i=0; i<num_processes_; i++)
				{
					std::string pub_name = "chatter"+std::to_string(i);
					pubs_.push_back(node_handle_.advertise<std_msgs::String>(pub_name,1000));
				}
				outcome_pub = node_handle_.advertise<std_msgs::String>("output",1000);
			}

			void init()
			{
				for (int i=0; i<num_processes_; i++)
				{
					std::string sub_name= "chatter"+std::to_string(i);
					subs_.push_back(node_handle_.subscribe<std_msgs::String>(sub_name, 1000, boost::bind(&Listener::chatterCallback, this, boost::placeholders::_1, std::to_string(i))));
				}
			}

			void roundRobin(const std_msgs::String::ConstPtr& msg)
			{

				pubs_[counter%num_processes_].publish(msg);
				counter++;


			}
			void chatterCallback(const std_msgs::String::ConstPtr& msg, std::string user_string)
			{
				ROS_INFO_STREAM("I heard: ["<<  msg->data << "] with user string [" << user_string << "] [thread=" << boost::this_thread::get_id() << "]");
				ROS_INFO("This is a long thread and takes long to execute each thing");
				d.sleep();
				std_msgs::String out_msg;
				std::stringstream outcome_of_thread;
				outcome_of_thread << "Thread " << user_string << " finished exectution. [thread=" << boost::this_thread::get_id() <<"]"; 
				out_msg.data = outcome_of_thread.str();
				outcome_pub.publish(out_msg);
			}
	};

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "listener");
		ros::NodeHandle n;

		Listener l(n,4);
		l.init();
		/**
		 * The MultiThreadedSpinner object allows you to specify a number of threads to use
		 * to call callbacks.  If no explicit # is specified, it will use the # of hardware
		 * threads available on your system.  Here we explicitly specify 4 threads.
		 */
		ros::MultiThreadedSpinner s(4);
		ros::spin(s);

		return 0;
	}

