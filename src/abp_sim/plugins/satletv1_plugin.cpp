#ifndef SATLETV1_PLUGIN
#define SATLETV1_PLUGIN

#include <thread>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"

namespace gazebo {
	class SatletV1Plugin : public ModelPlugin {
		public: SatletV1Plugin() : ModelPlugin() {
			// nothing		
		}

		public: void Load(physics::ModelPtr parent, sdf::ElementPtr) {
			this->model = parent;

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					std::bind(&SatletV1Plugin::OnUpdate, this));

			if (!ros::isInitialized()) {
				int argc = 0;
				char **argv = NULL;
				ros::init(argc,
						argv,
						this->model->GetName() + "_plugin",
						ros::init_options::NoSigintHandler);
			}
			this->rosNode.reset(new ros::NodeHandle(this->model->GetName() + "_plugin"));
			ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
					"/" + this->model->GetName() + "/velocity",
					1,
					boost::bind(&SatletV1Plugin::OnRosMsg, this, _1),
					ros::VoidPtr(),
					&this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);
			this->rosQueueThread = std::thread(std::bind(&SatletV1Plugin::QueueThread, this));
			
		}
	

		public: void OnUpdate() {
			// nothing		
		}

		public: void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr &msg) {
			auto lin = ignition::math::Vector3d(msg->data[0], msg->data[1], 0.0);
			auto ang = ignition::math::Vector3d(0.0, 0.0, msg->data[2]);
			this->model->SetLinearVel(lin);
			this->model->SetAngularVel(ang);
		}

		public: void QueueThread() {
			static const double timeout = 0.01;
			while (this->rosNode->ok())
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}

			
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: std::unique_ptr<ros::NodeHandle> rosNode;
		private: ros::Subscriber rosSub;
		private: ros::CallbackQueue rosQueue;
		private: std::thread rosQueueThread;
	};
	GZ_REGISTER_MODEL_PLUGIN(SatletV1Plugin);
}

#endif
