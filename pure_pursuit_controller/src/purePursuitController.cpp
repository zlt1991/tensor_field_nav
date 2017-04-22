/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "pure_pursuit_controller/purePursuitController.h"

#include <cmath>

#include <geometry_msgs/Twist.h>

#include <visualization_msgs/Marker.h>

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

PurePursuitControllerNode::PurePursuitControllerNode(const ros::NodeHandle &nh) :
    _nodeHandle(nh),
    _nextWayPoint(-1)
{
   getParameters();
  _cmdVelocityPublisher = _nodeHandle.advertise<geometry_msgs::Twist>(_cmdVelocityTopicName, 1);///cmd_vel_mux/input/teleop ///cmd_vel
  _timer = nh.createTimer(ros::Duration(1.0 / _frequency), &PurePursuitControllerNode::timerCallback, this);
  _goTriExecute=_nodeHandle.advertiseService("/execute_goTri",&PurePursuitControllerNode::goTriExecute,this);
  _pathExecute = _nodeHandle.advertiseService("/execute_path",&PurePursuitControllerNode::pathExecute, this);
  _pathCancel=_nodeHandle.advertiseService("/cancel_path",&PurePursuitControllerNode::pathCancel,this);
  _pathCut=_nodeHandle.advertiseService("/cut_path",&PurePursuitControllerNode::pathCut,this);
  _pathRecover=_nodeHandle.advertiseService("/recover_path",&PurePursuitControllerNode::pathRecover,this);
  _turnDirect=_nodeHandle.advertiseService("/turn_direction",&PurePursuitControllerNode::turnDirect,this);
  _finishGoTri=_nodeHandle.advertise<std_msgs::String>("finish_go_tri",1);
  _finishRecover=_nodeHandle.advertise<std_msgs::String>("finish_recover",1);
  _finishTurn=_nodeHandle.advertise<std_msgs::String>("finish_turn",1);
  isFinish = false;
  recoverMode=false;
  recoverTurn=false;
  ifGoTrisector=false;
  ifFirstPoint=false;
  safeWayNum=std::numeric_limits<double>::max();
  //zlt
  //
}

PurePursuitControllerNode::~PurePursuitControllerNode()
{

}

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/
// 由欧拉角创建四元数
void PurePursuitControllerNode::angle2quat(vector<float> &angle, vector<float> &quaternion)
{
  float cx = cos(angle[0] / 2);
  float sx = sin(angle[0] / 2);
  float cy = cos(angle[1] / 2);
  float sy = sin(angle[1] / 2);
  float cz = cos(angle[2] / 2);
  float sz = sin(angle[2] / 2);

  quaternion[0] = sx * cy * cz - cx * sy * sz; // x
  quaternion[1] = cx * sy * cz + sx * cy * sz; // y
  quaternion[2] = cx * cy * sz - sx * sy * cz; // z
  quaternion[3] = cx * cy * cz + sx * sy * sz; // w
}
//
bool PurePursuitControllerNode::pathExecute(pure_pursuit_controller::executePath::Request &req,pure_pursuit_controller::executePath::Response &res)
{
    resetParam();
    _currentReferencePath = req.curPath;
    ifFirstPoint=req.ifFirstPoint;
    ROS_INFO("receive path msg");
    res.success=true;
    return true;
}
bool PurePursuitControllerNode::goTriExecute(pure_pursuit_controller::executePath::Request &req,pure_pursuit_controller::executePath::Response &res)
{
    resetParam();
    _currentReferencePath = req.curPath;
    ifFirstPoint=req.ifFirstPoint;
    ifGoTrisector=true;
    ROS_INFO("receive branch msg");
    res.success=true;
    return true;
}

bool PurePursuitControllerNode::pathCancel(pure_pursuit_controller::cancelPath::Request &req, pure_pursuit_controller::cancelPath::Response &res){
    resetParam();
    res.success=true;
    return true;
}

bool PurePursuitControllerNode::pathCut(pure_pursuit_controller::cutPath::Request &req, pure_pursuit_controller::cutPath::Response &res){
    safeWayNum=req.safeWayPoint;
    ifCutPath=true;
    res.success=true;
    return true;
}

bool PurePursuitControllerNode::pathRecover(pure_pursuit_controller::recoverPath::Request &req, pure_pursuit_controller::recoverPath::Response &res){
    resetParam();
    _currentReferencePath = req.recoverPath;
    recoverMode=true;
    recoverTurn=true;
    backup_pose=getCurrentPose();
    ROS_INFO("receive recover path msg");
    res.success=true;
    return true;
}

bool PurePursuitControllerNode::turnDirect(pure_pursuit_controller::recoverPath::Request &req, pure_pursuit_controller::recoverPath::Response &res){
    resetParam();
    ifTurnDirect=true;
    _currentReferencePath = req.recoverPath;
    geometry_msgs::PoseStamped origin = getCurrentPose();
    //
    double x = origin.pose.orientation.x;
    double y = origin.pose.orientation.y;
    double z = origin.pose.orientation.z;
    double w = origin.pose.orientation.w;
    double robot_angel = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    turnAngle=robot_angel+M_PI;

    ROS_INFO("receive turn direction msg");
    return true;
}

void PurePursuitControllerNode::odometryCallback(const nav_msgs::Odometry &
                                                     msg)
{
  _currentVelocity = msg.twist.twist;
}

void PurePursuitControllerNode::resetParam(){
    _nextWayPoint=-1;
    _currentReferencePath.poses.clear();
    safeWayNum=std::numeric_limits<int>::max();
    isFinish=false;
    recoverMode=false;
    recoverTurn=false;
    ifFirstPoint=false;
    ifGoTrisector=false;
    ifCutPath=false;
    ifTurnDirect=false;
}

void PurePursuitControllerNode::spin()
{
  ros::spin();
}

void PurePursuitControllerNode::timerCallback(const ros::TimerEvent &
                                                  event)
{
  if(_nextWayPoint>safeWayNum){
     resetParam();
     return ;
  }
  geometry_msgs::Twist cmdVelocity;

  if (step(cmdVelocity))
      _cmdVelocityPublisher.publish(cmdVelocity);

  if(ifGoTrisector){
      if(isFinish){
            //do something
          std_msgs::String msg;
          for(int i=0;i<2;i++)
            _finishGoTri.publish(msg);
          ifGoTrisector=false;
      }
  }

  if(!recoverMode){
    if(isFinish)
        resetParam();
  }

}

bool PurePursuitControllerNode::step(geometry_msgs::Twist &twist)
{
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  if(_currentReferencePath.poses.size()==0) return false;

  _nextWayPoint = getNextWayPoint(_nextWayPoint);
  if(_nextWayPoint>safeWayNum){
     resetParam();
     return false;
  }

  if (_nextWayPoint >= 0)
  {
    double angularVelocity = 0.0;
    double linearVelocity = _velocity;
    //
    geometry_msgs::PoseStamped origin = getCurrentPose();
    //
    double x = origin.pose.orientation.x;
    double y = origin.pose.orientation.y;
    double z = origin.pose.orientation.z;
    double w = origin.pose.orientation.w;
    double robot_angel = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    geometry_msgs::PoseStamped next= _currentReferencePath.poses[_nextWayPoint];
    double x1 = next.pose.position.x;
    double y1 = next.pose.position.y;
    double x0 = origin.pose.position.x;
    double y0 = origin.pose.position.y;
    double d = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
    double distance = std::sqrt(d);
    // double r = distance / (2*std::sin(angle));

    //
    double line_angel = asin((y1 - y0) / distance);
    //
    if (line_angel > 0)
    {
      if (x1 < x0)
      {
        line_angel = M_PI - line_angel;
      }
    }
    else
    {
      if (x1 < x0)
      {
        line_angel = -M_PI - line_angel;
      }
      if (line_angel == -M_PI)
      {
        line_angel = M_PI;
      }
    }

    double angel = robot_angel - line_angel;
    if(ifTurnDirect){
        angel=turnAngle-robot_angel;
        twist.linear.x = 0;
        if(std::sin(angel)>0)
          twist.angular.z = -0.4;
        else
          twist.angular.z = 0.4;
        if (std::abs(angel) < M_PI/6)
        {
            resetParam();
            std_msgs::String msg;
            for(int i=0;i<2;i++)
              _finishTurn.publish(msg);
        }
        return true;
    }

    double r = distance / (2 * std::sin(angel));
    angularVelocity = -linearVelocity / r;

    if(recoverMode){
      if(recoverTurn){
          twist.linear.x = 0;
          if(std::sin(angel)>0)
            twist.angular.z = -0.4;
          else
            twist.angular.z = 0.4;
          if (std::abs(angel) < M_PI/8)
              recoverTurn=false;
          return true;
      }

    }
    if(recoverMode)
      if(isFinish){
          double x = backup_pose.pose.orientation.x;
          double y = backup_pose.pose.orientation.y;
          double z = backup_pose.pose.orientation.z;
          double w = backup_pose.pose.orientation.w;
          double robot_origin_angel = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
           double angel = robot_angel - robot_origin_angel;
           twist.linear.x = 0;
           if(std::sin(angel)>0)
             twist.angular.z = -0.4;
           else
             twist.angular.z = 0.4;
           if (std::abs(angel) < M_PI/8)
           {
               std_msgs::String msg;
               for(int i=0;i<2;i++)
                 _finishRecover.publish(msg);
               recoverMode=false;
           }
           return true;
      }

    if(ifFirstPoint){
        twist.linear.x = 0;
        if(std::sin(angel)>0)
          twist.angular.z = -0.4;
        else
          twist.angular.z = 0.4;
        if (std::abs(angel) < M_PI/8)
            ifFirstPoint=false;
        return true;
    }
    twist.linear.x = linearVelocity;
    twist.angular.z = angularVelocity;

    return true;
  }
  return false;
}

geometry_msgs::PoseStamped PurePursuitControllerNode::getCurrentPose() const
{
  geometry_msgs::PoseStamped pose, transformedPose;
  pose.header.frame_id = "base_footprint";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;
  try
  {
    _tfListener.transformPose(_currentReferencePath.header.frame_id,
                              pose, transformedPose);
  }
  catch (tf::TransformException &exception)
  {
    ROS_ERROR("_poseFrameId: %s ", _poseFrameId.c_str());
  }

  return transformedPose;
}


double PurePursuitControllerNode::getLookAheadThreshold() const
{
  return _lookAheadRatio * 0.1;
}

int PurePursuitControllerNode::getNextWayPoint(int wayPoint)
{
  if (!_currentReferencePath.poses.empty())
  {
    if (_nextWayPoint >= 0)
    {
      geometry_msgs::PoseStamped origin = getCurrentPose();
      tf::Vector3 v_1(origin.pose.position.x,
                      origin.pose.position.y,
                      0);
      double lookAheadThreshold = getLookAheadThreshold();

      for (int i = _nextWayPoint; i < _currentReferencePath.poses.size();
           ++i)
      {
        tf::Vector3 v_2(_currentReferencePath.poses[i].pose.position.x,
                        _currentReferencePath.poses[i].pose.position.y,
                        0);
        if(ifCutPath){
            if (i >= safeWayNum-3)
            {
                //ROS_INFO("Path finish!");
                isFinish = true;
            }
        }else
        {
            if(_currentReferencePath.poses.size()>3){
                if (i >= _currentReferencePath.poses.size()-3)
                {
                    //ROS_INFO("Path finish!");
                    isFinish = true;
                }
            }
        }
        if (tf::tfDistance(v_1, v_2) > lookAheadThreshold)
        {
          return i;
        }
      }
      if (_nextWayPoint == _currentReferencePath.poses.size()-1)
      {
          //ROS_INFO("Path finish!");
          isFinish = true;
      }
      return _nextWayPoint;
    }
    else
      return 0;
  }

  return -1;
}

void PurePursuitControllerNode::getParameters()
{
  _nodeHandle.param<int>("pure_pursuit_controller/ros/queue_depth", _queueDepth, 100);
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/odometry_topic_name", _odometryTopicName, "/odom");
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/cmd_velocity_topic_name", _cmdVelocityTopicName, "/cmd_vel_mux/input/teleop");
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/pose_frame_id", _poseFrameId, "base_footprint");

  _nodeHandle.param<double>("pure_pursuit_controller/controller/frequency", _frequency, 20.0);
  _nodeHandle.param<int>("pure_pursuit_controller/controller/initial_waypoint", _initialWayPoint, -1);
  _nodeHandle.param<double>("pure_pursuit_controller/controller/velocity", _velocity, 0.2);
  _nodeHandle.param<double>("pure_pursuit_controller/controller/look_ahead_ratio", _lookAheadRatio, 4.0);
  _nodeHandle.param<double>("pure_pursuit_controller/controller/epsilon", _epsilon, 1e-6);
}

