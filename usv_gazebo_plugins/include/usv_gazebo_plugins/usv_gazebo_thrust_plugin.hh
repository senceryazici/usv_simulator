/*
  * Copyright (C) 2017  Brian Bingham
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
 
 #ifndef USV_GAZEBO_PLUGINS_THRUST_HH
 #define USV_GAZEBO_PLUGINS_THRUST_HH
 
 #include <ros/ros.h>
 #include <std_msgs/Float32.h>
 #include <sensor_msgs/JointState.h>
 #include <memory>
 #include <mutex>
 #include <string>
 #include <vector>
 #include <gazebo/common/CommonTypes.hh>
 #include <gazebo/common/Plugin.hh>
 #include <gazebo/common/Time.hh>
 #include <gazebo/physics/physics.hh>
 #include <sdf/sdf.hh>
 
 namespace gazebo
 {
   // Foward declaration of UsvThrust class
   class UsvThrust;
 
   class Thruster
   {
     public: explicit Thruster(UsvThrust *_parent);
 
     public: void OnThrustCmd(const std_msgs::Float32::ConstPtr &_msg);
 
     public: void OnThrustAngle(const std_msgs::Float32::ConstPtr &_msg);
 
     public: double maxCmd;
 
     public: double maxForceFwd;
 
     public: double maxForceRev;
 
     public: double maxAngle;
 
     public: physics::LinkPtr link;
 
     public: int mappingType;
 
     public: std::string cmdTopic;
 
     public: ros::Subscriber cmdSub;
 
     public: bool enableAngle;
 
     public: std::string angleTopic;
 
     public: ros::Subscriber angleSub;
 
     public: double currCmd;
 
     public: double desiredAngle;
 
     public: common::Time lastCmdTime;
 
     public: common::Time lastAngleUpdateTime;
 
     public: physics::JointPtr propJoint;
 
     public: physics::JointPtr engineJoint;
 
     public: common::PID engineJointPID;
 
     protected: UsvThrust *plugin;
   };
 
 
   class UsvThrust : public ModelPlugin
   {
     public: UsvThrust() = default;
 
     public: virtual ~UsvThrust() = default;
 
     // Documentation inherited.
     public: virtual void Load(physics::ModelPtr _parent,
                               sdf::ElementPtr _sdf);
 
     protected: virtual void Update();
 
     private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                    const std::string &_paramName,
                                    const double _defaultVal) const;
 
     private: double ScaleThrustCmd(const double _cmd,
                                    const double _max_cmd,
                                    const double _max_pos,
                                    const double _max_neg) const;
 
     private: double Glf(const double _x,
                         const float  _A,
                         const float  _K,
                         const float  _B,
                         const float  _v,
                         const float  _C,
                         const float  _M) const;
 
     private: double GlfThrustCmd(const double _cmd,
                                  const double _maxPos,
                                  const double _maxNeg) const;
 
     private: void RotateEngine(size_t _i,
                                common::Time _stepTime);
 
     private: void SpinPropeller(size_t _i);
 
     public: std::mutex mutex;
 
     private: std::unique_ptr<ros::NodeHandle> rosnode;
 
     public: physics::WorldPtr world;
 
     private: physics::ModelPtr model;
 
     private: double cmdTimeout;
 
     private: std::vector<Thruster> thrusters;
 
     private: event::ConnectionPtr updateConnection;
 
     private: ros::Publisher jointStatePub;
 
     private: sensor_msgs::JointState jointStateMsg;
   };
 }
 
 #endif
