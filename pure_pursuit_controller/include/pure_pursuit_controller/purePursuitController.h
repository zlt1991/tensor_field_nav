#ifndef PURE_PURSUIT_CONTROLLER_NODE_H
#define PURE_PURSUIT_CONTROLLER_NODE_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <pure_pursuit_controller/executePath.h>
#include <pure_pursuit_controller/cancelPath.h>
#include <pure_pursuit_controller/cutPath.h>
#include <pure_pursuit_controller/recoverPath.h>

using namespace std;
/** The class PurePursuitControllerNode implements a pure pursuit controller.
      \brief Pure pursuit controller
    */
class PurePursuitControllerNode
{
public:
  /** \name Constructors/destructor
      @{
      */
  /// Constructor
  PurePursuitControllerNode(const ros::NodeHandle &nh);

  /// Destructor
  virtual ~PurePursuitControllerNode();
  /** @}
      */

  /** \name Methods
      @{
      */
  /// Spin once
  void spin();
  /// Step once
  bool step(geometry_msgs::Twist &twist);
  /// Returns the current pose of the robot
  geometry_msgs::PoseStamped getCurrentPose() const;
  /// Returns the current lookahead distance threshold
  double getLookAheadThreshold() const;
  /// Returns the next way point by linear search from the current waypoint
  int getNextWayPoint(int wayPoint);
  void resetParam();
  /** @}
      */
  void angle2quat(vector<float> &angle, vector<float> &quaternion);

protected:
  /** \name Protected methods
      @{
      */
//  /// Retrieves parameters
//  void getParameters();
  /// Path message execute
  bool pathExecute(pure_pursuit_controller::executePath::Request &req,pure_pursuit_controller::executePath::Response &res);
  bool goTriExecute(pure_pursuit_controller::executePath::Request &req,pure_pursuit_controller::executePath::Response &res);
  bool pathCancel(pure_pursuit_controller::cancelPath::Request &req, pure_pursuit_controller::cancelPath::Response &res);
  bool pathCut(pure_pursuit_controller::cutPath::Request &req, pure_pursuit_controller::cutPath::Response &res);

  /// Odometry message callback
  void odometryCallback(const nav_msgs::Odometry &msg);
  /// Timer callback
  void timerCallback(const ros::TimerEvent &event);
  void getParameters();

  /** @}
      */

  /** \name Protected members
      @{
      */
  /// ROS node handle
  ros::NodeHandle _nodeHandle;
  /// Path message subscriber
  ///
  ros::ServiceServer _goTriExecute;
  ros::ServiceServer _pathExecute;
  ros::ServiceServer _pathCancel;
  ros::ServiceServer _pathCut;
  ///
  ros::Publisher _finishGoTri;
  /// Path message topic name
  std::string _pathExecuteServerName;

  /// Odometry message topic name
  std::string _odometryTopicName;
  /// Frame id of pose estimates
  std::string _poseFrameId;

  /// Queue size for receiving messages
  int _queueDepth;
  bool isFinish;
  int safeWayNum;
  bool ifGoTrisector;
  bool ifFirstPoint;
  bool ifCutPath;

  /// Current reference path
  nav_msgs::Path _currentReferencePath;
  /// Current velocity
  geometry_msgs::Twist _currentVelocity;
  /// Controller frequency
  double _frequency;
  /// Next way point
  int _nextWayPoint;
  /// Commanded velocity publisher
  ros::Publisher _cmdVelocityPublisher;
  /// Commanded velocity topic name
  std::string _cmdVelocityTopicName;

  /// Initial way point
  int _initialWayPoint;
  /// Velocity
  double _velocity;
  /// Lookahead ratio
  double _lookAheadRatio;
  /// Epsilon
  double _epsilon;
  /// Transform listener for robot's pose w.r.t. map
  tf::TransformListener _tfListener;
  /// Timer
  ros::Timer _timer;
};
#endif // PURE_PURSUIT_CONTROLLER_NODE_H
