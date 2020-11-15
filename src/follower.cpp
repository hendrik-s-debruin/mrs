#include <uvdar_leader_follower/follower.h>
#include <uvdar_leader_follower/FollowerConfig.h>
#include <cmath>

bool is_initialized     = false;
bool got_odometry       = false;
bool got_tracker_output = false;
bool got_uvdar          = false;

Eigen::Vector3d follower_position_odometry;
Eigen::Vector3d follower_linear_velocity_odometry;
double          follower_heading_odometry;
double          follower_heading_rate_odometry;

Eigen::Vector3d follower_position_tracker;
Eigen::Vector3d follower_linear_velocity_tracker;
double          follower_heading_tracker;
double          follower_heading_rate_tracker;

double speed_track_max_vel;
double speed_track_attractor_width;
double speed_track_attractor_gain;
double speed_track_attractor_offset;
double speed_track_evade_min_safe;
double speed_track_evade_min;
double speed_track_evade_max_safe;
double speed_track_evade_max;
double speed_track_evade_min_gain;
double speed_track_evade_max_gain;
double speed_track_heading_attractor_gain;
double speed_track_attractor_vel_gain;

Eigen::Vector3d leader_position;
ros::Time       last_leader_contact;

// dynamically reconfigurable
Eigen::Vector3d position_offset          = Eigen::Vector3d(0.0, 0.0, 0.0);
double          heading_offset           = 0.0;
double          uvdar_msg_interval       = 0.1;
bool            use_estimator            = false;
bool            use_speed_tracker        = false;
bool            use_trajectory_reference = false;

VelocityEstimator estimator;
Eigen::Vector3d   leader_predicted_position;
Eigen::Vector3d   leader_predicted_velocity;

/* initialize //{ */
uvdar_leader_follower::FollowerConfig FollowerController::initialize(mrs_lib::ParamLoader& param_loader) {

  ROS_INFO("[Follower]: Waiting for odometry and uvdar");
  while (ros::ok()) {
    if (got_odometry && got_uvdar) {
      break;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 3, 3> R;
  param_loader.loadMatrixStatic("Q", Q);
  param_loader.loadMatrixStatic("R", R);
  param_loader.loadParam("control_action_interval", control_action_interval);
  param_loader.loadParam("desired_offset/x", position_offset.x());
  param_loader.loadParam("desired_offset/y", position_offset.y());
  param_loader.loadParam("desired_offset/z", position_offset.z());
  param_loader.loadParam("heading_offset", heading_offset);

  param_loader.loadParam("use_estimator",                    use_estimator);
  param_loader.loadParam("use_speed_tracker",                use_speed_tracker);
  param_loader.loadParam("use_trajectory_reference",         use_trajectory_reference);

	param_loader.loadParam("speed_track_max_vel",                speed_track_max_vel);
	param_loader.loadParam("speed_track_attractor_width",        speed_track_attractor_width);
	param_loader.loadParam("speed_track_attractor_gain",         speed_track_attractor_gain);
	param_loader.loadParam("speed_track_attractor_offset",       speed_track_attractor_offset);
	param_loader.loadParam("speed_track_evade_min_safe",         speed_track_evade_min_safe);
	param_loader.loadParam("speed_track_evade_min",              speed_track_evade_min);
	param_loader.loadParam("speed_track_evade_max_safe",         speed_track_evade_max_safe);
	param_loader.loadParam("speed_track_evade_max",              speed_track_evade_max);
	param_loader.loadParam("speed_track_evade_min_gain",         speed_track_evade_min_gain);
	param_loader.loadParam("speed_track_evade_max_gain",         speed_track_evade_max_gain);
	param_loader.loadParam("speed_track_heading_attractor_gain", speed_track_heading_attractor_gain);
	param_loader.loadParam("speed_track_attractor_vel_gain",     speed_track_attractor_vel_gain);

  //// initialize the dynamic reconfigurables with values from YAML file and values set above
  uvdar_leader_follower::FollowerConfig config;
  config.desired_offset_x         = position_offset.x();
  config.desired_offset_y         = position_offset.y();
  config.desired_offset_z         = position_offset.z();
  config.heading_offset           = heading_offset;
  config.filter_data              = use_estimator;
  config.use_trajectory_reference = use_trajectory_reference;
  config.use_speed_tracker        = use_speed_tracker;
  ////

  VelocityEstimator::kalman3D::x_t initial_states;

  // set initial state of estimator as follows: leader position: (current follower pos - desired offset), leader velocity: (0,0,0)
  initial_states << follower_position_odometry.x() - position_offset.x(), follower_position_odometry.y() - position_offset.y(),
      follower_position_odometry.z() - position_offset.z(), 0, 0, 0;
  estimator = VelocityEstimator(Q, R, initial_states, uvdar_msg_interval);


  is_initialized = true;
  return config;
}
//}

/* dynamicReconfigureCallback //{ */
void FollowerController::dynamicReconfigureCallback(uvdar_leader_follower::FollowerConfig& config, [[maybe_unused]] uint32_t level) {
  position_offset          = Eigen::Vector3d(config.desired_offset_x, config.desired_offset_y, config.desired_offset_z);
  heading_offset           = config.heading_offset;
  use_speed_tracker        = config.use_speed_tracker;
  use_trajectory_reference = config.use_trajectory_reference;

  if (!use_estimator && config.filter_data) {
    ROS_INFO("[%s]: Estimator started", ros::this_node::getName().c_str());
  }
  use_estimator = config.filter_data;
}
//}

/* receiveOdometry //{ */
void FollowerController::receiveOdometry(const nav_msgs::Odometry& odometry_msg) {

  follower_position_odometry.x() = odometry_msg.pose.pose.position.x;
  follower_position_odometry.y() = odometry_msg.pose.pose.position.y;
  follower_position_odometry.z() = odometry_msg.pose.pose.position.z;

  mrs_lib::AttitudeConverter ac(odometry_msg.pose.pose.orientation);
  follower_heading_odometry = ac.getHeading();

  follower_linear_velocity_odometry.x() = odometry_msg.twist.twist.linear.x;
  follower_linear_velocity_odometry.y() = odometry_msg.twist.twist.linear.y;
  follower_linear_velocity_odometry.z() = odometry_msg.twist.twist.linear.z;

  follower_heading_rate_odometry =
      ac.getHeadingRate(Eigen::Vector3d(odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z));

  got_odometry = true;
}
//}

/* receiveTrackerOutput //{ */
void FollowerController::receiveTrackerOutput(const mrs_msgs::PositionCommand& position_cmd) {

  follower_position_tracker.x() = position_cmd.position.x;
  follower_position_tracker.y() = position_cmd.position.y;
  follower_position_tracker.z() = position_cmd.position.z;

  follower_heading_tracker = position_cmd.heading;

  follower_linear_velocity_tracker.x() = position_cmd.velocity.x;
  follower_linear_velocity_tracker.y() = position_cmd.velocity.y;
  follower_linear_velocity_tracker.z() = position_cmd.velocity.z;

  follower_heading_rate_tracker = position_cmd.heading_rate;

  got_tracker_output = true;
}
//}

/* receiveUvdar //{ */
void FollowerController::receiveUvdar(const geometry_msgs::PoseWithCovarianceStamped& uvdar_msg) {

  Eigen::Vector3d leader_new_position;

  leader_new_position.x() = uvdar_msg.pose.pose.position.x;
  leader_new_position.y() = uvdar_msg.pose.pose.position.y;
  leader_new_position.z() = uvdar_msg.pose.pose.position.z;

  last_leader_contact = uvdar_msg.header.stamp;
  got_uvdar           = true;

  leader_position = leader_new_position;

  if (use_estimator && is_initialized) {
    estimator.fuse(leader_new_position);
  }
}
//}

/* createReferencePoint //{ */
ReferencePoint FollowerController::createReferencePoint() {
  ReferencePoint point;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    point.position        = Eigen::Vector3d(0, 0, 0);
    point.heading         = 0;
    point.use_for_control = false;
    return point;
  }

  if (use_estimator) {
    point.position.x() = leader_predicted_position.x() + position_offset.x();
    point.position.y() = leader_predicted_position.y() + position_offset.y();
    point.position.z() = leader_predicted_position.z() + position_offset.z();
  } else {
    point.position.x() = leader_position.x() + position_offset.x();
    point.position.y() = leader_position.y() + position_offset.y();
    point.position.z() = leader_position.z() + position_offset.z();
  }
  point.heading         = heading_offset;
  point.use_for_control = true;


  // cap the commands
  ReferencePoint current_position;
  current_position.position = follower_position_odometry;
  current_position.heading  = follower_heading_odometry;

  point = capReferencePointDelta(point, current_position);

  return point;
}
//}

ReferencePoint FollowerController::capReferencePointDelta(
		ReferencePoint const &desired_reference,
		ReferencePoint const &from_reference
	) const
{
	auto r = desired_reference.position - from_reference.position;
	ReferencePoint capped_reference_point;
	capped_reference_point.heading         = desired_reference.heading;
	capped_reference_point.use_for_control = desired_reference.use_for_control;

	if(r.norm() <= 13)
	{
		capped_reference_point.position = desired_reference.position;
	}
	else
	{
		auto d = r.normalized();
		capped_reference_point.position = from_reference.position + d*13;
		ROS_INFO("CAPPING REFERENCE POINT EARLY");
	}

	return capped_reference_point;
}

geometry_msgs::Pose FollowerController::calculatePerpendicularPoint(
    nav_msgs::Odometry const &leader,
    Eigen::Vector3d const &   follower_position,
    double                    distance) const
{
	Eigen::Vector3d leader_vel;
	leader_vel <<
		leader.twist.twist.linear.x,
		leader.twist.twist.linear.y,
		leader.twist.twist.linear.z;

	auto leader_heading = leader_vel.normalized();

	Eigen::Matrix<double, 3, 3> rotate_90_deg;
	rotate_90_deg << 0, -1, 0, 1, 0, 0, 0, 0, 1;

	auto perp_to_heading = rotate_90_deg*leader_heading;

	Eigen::Vector3d leader_position;
	leader_position <<
		leader.pose.pose.position.x,
		leader.pose.pose.position.y,
		leader.pose.pose.position.z;

	Eigen::Vector3d target_position_candidate_1 = leader_position + perp_to_heading*distance;
	Eigen::Vector3d target_position_candidate_2 = leader_position - perp_to_heading*distance;
	Eigen::Vector3d target_position;

	if((follower_position - target_position_candidate_1).norm() <
	   (follower_position - target_position_candidate_2).norm())
	{
		target_position = target_position_candidate_1;
	}
	else
	{
		target_position = target_position_candidate_2;
	}

	geometry_msgs::Pose return_pose;
	return_pose.position.x = target_position(0);
	return_pose.position.y = target_position(1);
	return_pose.position.z = target_position(2);

	return return_pose;
}

/* createReferenceTrajectory //{ */
ReferenceTrajectory FollowerController::createReferenceTrajectory() {
	auto disable_trajectory_tracking_msg = []() {
		ReferenceTrajectory trajectory;
		trajectory.positions.push_back(Eigen::Vector3d::Zero());
		trajectory.headings.push_back(0.0);
		trajectory.sampling_time   = 0.0;
		trajectory.use_for_control = false;
		return trajectory;
	};
	// sanity check
	if(!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output)
	{
		return disable_trajectory_tracking_msg();
	}

  // Trajectory generation here does not work when the follower needs to evade
  // the leader. Disable and fall back to position tracking instead
	if(std::abs(follower_position_odometry.z() - 3) > 0.3)
	{
		return disable_trajectory_tracking_msg();
	}

  ReferenceTrajectory trajectory;

  // Example - start trajectory at current UAV position and move in the predicted direction of leader motion
  // No subsampling, only two points are created in this example
  Eigen::Vector3d point_1;
  double          heading_1;

  Eigen::Vector3d point_2;
  double          heading_2;

  trajectory.use_for_control = false;
  if (use_trajectory_reference) {
    if (use_estimator) {
      point_1   = follower_position_tracker;
      heading_1 = follower_heading_tracker;

      point_2   = leader_predicted_position + position_offset + (leader_predicted_velocity * control_action_interval);
      heading_2 = heading_offset;

      trajectory.positions.push_back(point_1);
      trajectory.positions.push_back(point_2);

      trajectory.headings.push_back(heading_1);
      trajectory.headings.push_back(heading_2);
      trajectory.sampling_time   = control_action_interval;
      trajectory.use_for_control = true;
    } else {
      ROS_WARN("[%s]: Tried to plan a trajectory without leader velocity estimation", ros::this_node::getName().c_str());
    }
  }

  return trajectory;
}
//}

/* createSpeedCommand //{ */
SpeedCommand FollowerController::createSpeedCommand() {
  SpeedCommand command;

  if (!got_odometry || !got_uvdar || !got_tracker_output) {
    command.velocity        = Eigen::Vector3d(0, 0, 0);
    command.heading         = 0;
    command.height          = 0;
    command.use_for_control = false;
  }

  if(use_estimator && use_speed_tracker)
  {
	// =======================================================================
	// Attractor Location
	// =======================================================================
	Eigen::Vector3d attractor_point;
	{
		Eigen::Vector3d attractor_vel_comp = Eigen::Vector3d::Zero();
		if(leader_predicted_velocity.norm() > 0.1 &&
		   follower_linear_velocity_odometry.norm() > 1)
		{
			Eigen::Vector3d leader_dir = leader_predicted_velocity.normalized();
			ROS_INFO("Update  position offset");
			Eigen::Matrix<double, 3, 3> rot;
			// clockwise 90deg
			rot << 0,  1, 0,
				   -1, 0, 0,
				   0,  0, -1;

			Eigen::Vector3d perp_dir = rot*leader_dir;

			position_offset = perp_dir * speed_track_attractor_offset;
			attractor_vel_comp = leader_dir *
			(
				leader_predicted_velocity.norm() - follower_linear_velocity_odometry.dot(leader_predicted_velocity)
			) *
		   speed_track_attractor_vel_gain;
		}
		Eigen::Vector3d attractor_point1 = leader_position + position_offset;
		Eigen::Vector3d attractor_point2 = leader_position - position_offset;

		if((attractor_point1 - follower_position_odometry).norm() <
		   (attractor_point2 - follower_position_odometry).norm())
		{
			attractor_point = attractor_point1;
		}
		else
		{
			attractor_point = attractor_point2;
		}

		attractor_point += attractor_vel_comp;

	}

	// =========================================================================
	// Heading Attractor
	// =========================================================================
	double reference_heading = 0;
	{
		Eigen::Vector3d to_leader_dir =
			(leader_position - follower_position_odometry).normalized();
		double angle_to_leader = std::atan2(to_leader_dir(1), to_leader_dir(0));
		double target_heading = angle_to_leader + heading_offset;
		double offset_from_heading = target_heading - follower_heading_odometry;
		reference_heading =
			follower_heading_odometry +
			(
				offset_from_heading *
				speed_track_heading_attractor_gain * control_action_interval
			);
	}


	// =========================================================================
	// Attractor to target
	// =========================================================================
	Eigen::Vector3d attractor_vel;
	{
		Eigen::Vector3d drone_to_attractor =
			attractor_point - follower_position_odometry;

		double distance_to_attractor = drone_to_attractor.norm();


		double attractor_speed = speed_track_attractor_gain *
			                     distance_to_attractor * distance_to_attractor;

		if(attractor_speed > speed_track_max_vel)
		{
			attractor_speed = speed_track_max_vel;
		}

		attractor_vel =
			attractor_speed * drone_to_attractor.normalized();

	}

	// =========================================================================
	// Emergency Collision Evasion Control
	// =========================================================================
	Eigen::Vector3d evasion_vel;
	{
		double distance_between_drones =
			(leader_position - follower_position_odometry).norm();

		Eigen::Vector3d evasion_dir =
			(leader_position - follower_position_odometry).normalized();

		double evasion_speed = 0;

		if(distance_between_drones < speed_track_evade_min_safe &&
		   distance_between_drones > speed_track_evade_min)
		{
			evasion_speed = speed_track_evade_max_gain *
				tan
				(
					M_PI/
					(
						2*(speed_track_evade_min_safe - speed_track_evade_min)
					)
					*
					(
						distance_between_drones - speed_track_evade_max_safe
					)
				);
		}
		else if (distance_between_drones < speed_track_evade_min)
		{
			evasion_speed = 100;
		}
		else if(distance_between_drones > speed_track_evade_max_safe &&
			      distance_between_drones < 15)
		{
			evasion_speed = speed_track_evade_max_gain *
				tan
				(
					M_PI/
					(
						2*(speed_track_evade_max - speed_track_evade_max_safe)
					)
					*
					(
						distance_between_drones - speed_track_evade_max_safe
					)
				);
		}
		else if( distance_between_drones >= 15)
		{
			evasion_speed = 100;
		}

		evasion_vel = evasion_dir * evasion_speed;
	}

	// =========================================================================
	// Final Velocity Command
	// =========================================================================
	Eigen::Vector3d target_velocity = attractor_vel + evasion_vel;


	if(target_velocity.norm() > speed_track_max_vel)
	{
		target_velocity = target_velocity.normalized() * speed_track_max_vel;
	}

	  command.velocity = target_velocity;
	  command.height   = 3;
	  command.heading  = reference_heading;//follower_heading_odometry;
	  command.use_for_control = true;
  }
  else
  {
	  command.use_for_control = false;
  }

  return command;
}
//}

/* getCurrentEstimate //{ */

// You can use this method for debugging purposes.
// It allows you to visualize the leader predictions in rviz
// It is called once per control action of the summer_schoo_supervisor

nav_msgs::Odometry FollowerController::getCurrentEstimate() {
  nav_msgs::Odometry leader_est;

  if (use_estimator) {
    auto leader_prediction          = estimator.predict(Eigen::Vector3d(0, 0, 0), control_action_interval);
    leader_predicted_position       = Eigen::Vector3d(leader_prediction[0], leader_prediction[1], leader_prediction[2]);
    leader_predicted_velocity       = Eigen::Vector3d(leader_prediction[3], leader_prediction[4], leader_prediction[5]);
    leader_est.pose.pose.position.x = leader_prediction[0];
    leader_est.pose.pose.position.y = leader_prediction[1];
    leader_est.pose.pose.position.z = leader_prediction[2];
    leader_est.twist.twist.linear.x = leader_prediction[3];
    leader_est.twist.twist.linear.y = leader_prediction[4];
    leader_est.twist.twist.linear.z = leader_prediction[5];
  } else {
    leader_est.pose.pose.position.x = leader_position.x();
    leader_est.pose.pose.position.y = leader_position.y();
    leader_est.pose.pose.position.z = leader_position.z();
  }

  return leader_est;
}
//}
