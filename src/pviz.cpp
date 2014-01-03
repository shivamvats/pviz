/* \author Ben Cohen */

#include <pviz/pviz.h>

static std::string RIGHT_CHAIN_RTIP_NAME = "r_gripper_r_finger_tip_link";
static std::string RIGHT_CHAIN_LTIP_NAME = "r_gripper_l_finger_tip_link";
static std::string LEFT_CHAIN_RTIP_NAME = "l_gripper_r_finger_tip_link";
static std::string LEFT_CHAIN_LTIP_NAME = "l_gripper_l_finger_tip_link";

void HSVtoRGB( double *r, double *g, double *b, double h, double s, double v )
{
	int i;
	double f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;        // sector 0 to 5
	i = floor(h);
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
    default:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

PViz::PViz(const std::string &ns)
{
  num_joints_ = 8; //arm + torso
  reference_frame_ = "/map";

  srand (time(NULL));
  arm_joint_names_.push_back("_shoulder_pan_joint");
  arm_joint_names_.push_back("_shoulder_lift_joint");
  arm_joint_names_.push_back("_upper_arm_roll_joint");
  arm_joint_names_.push_back("_elbow_flex_joint");
  arm_joint_names_.push_back("_forearm_roll_joint");
  arm_joint_names_.push_back("_wrist_flex_joint");
  arm_joint_names_.push_back("_wrist_roll_joint");

  arm_link_names_.push_back("_shoulder_pan_link");
  arm_link_names_.push_back("_shoulder_lift_link");
  arm_link_names_.push_back("_upper_arm_roll_link");
  arm_link_names_.push_back("_elbow_flex_link");
  arm_link_names_.push_back("_forearm_roll_link");
  arm_link_names_.push_back("_wrist_flex_link");
  arm_link_names_.push_back("_wrist_roll_link");

  torso_link_names_.push_back("torso_lift_link");
  torso_joint_names_.push_back("torso_lift_joint");

  // arm meshes
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_pan.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/upper_arm_roll.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/upper_arm.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/elbow_flex.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/forearm_roll.stl");  
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/forearm.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_flex.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_roll.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/gripper_v0/gripper_palm.dae");

  // gripper meshes
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger.dae");
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae");
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger.dae");  
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae");

  // torso_meshes
  torso_meshes_.push_back("package://pr2_description/meshes/torso_v0/torso_lift.dae");
  
  // base meshes
  base_meshes_.push_back("package://pr2_description/meshes/base_v0/base.dae");

  // head meshes
  head_meshes_.push_back("package://pr2_description/meshes/head_v0/head_pan.dae");
  head_meshes_.push_back("package://pr2_description/meshes/head_v0/head_tilt.dae");
  head_meshes_.push_back("package://pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.dae");

  robot_meshes_.insert(robot_meshes_.end(),base_meshes_.begin(), base_meshes_.end());  // 1
  robot_meshes_.insert(robot_meshes_.end(),torso_meshes_.begin(), torso_meshes_.end()); // 1
  robot_meshes_.insert(robot_meshes_.end(),arm_meshes_.begin(), arm_meshes_.end());  // 10
  robot_meshes_.insert(robot_meshes_.end(),gripper_meshes_.begin(), gripper_meshes_.end());  // 4
  robot_meshes_.insert(robot_meshes_.end(),arm_meshes_.begin(), arm_meshes_.end());  // 10
  robot_meshes_.insert(robot_meshes_.end(),gripper_meshes_.begin(), gripper_meshes_.end());  // 4
  robot_meshes_.insert(robot_meshes_.end(),head_meshes_.begin(), head_meshes_.end());  // 3

  if(!initKDLChain())
  {
    ROS_ERROR("[pviz] Failed to initiliaze the KDL chain. This should exit but instead it will crash."); 
    fflush(stdout);
  }

  if(ns.size() > 0)
  {
    std::stringstream ss;
    ss << "/" << ns << "/visualization_marker_array";
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(ss.str(), 500);
    ss.str(std::string());
    ss << "/" << ns << "/visualization_marker";
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(ss.str(), 1000);
  }
  else
  {
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  }
}

PViz::~PViz()
{
  for(size_t i = 0; i < fk_rsolver_.size(); ++i)
    delete fk_rsolver_[i];
  for(size_t i = 0; i < fk_lsolver_.size(); ++i)
    delete fk_lsolver_[i];
}

bool PViz::initKDLChain()
{
  std::string robot_description;
  std::string robot_param = "";
  if(!nh_.searchParam("robot_description",robot_param))
  {
    ROS_ERROR("[pviz] Unable to find the robot_description on the param server.");
    return false;
  }
  nh_.param<std::string>(robot_param,robot_description,"");
  if (!kdl_parser::treeFromString(robot_description, kdl_tree_))
  {
    ROS_ERROR("Failed to parse tree from robot description file.");
    return false;
  }

  // a total of 4 FK solvers are used for both arms & torso
  fk_rsolver_.resize(2);
  fk_lsolver_.resize(2);

  // right arm - right finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", RIGHT_CHAIN_RTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_rsolver_[RIGHT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the right arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("right arm - right finger", chain_);

  // right arm - left finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", RIGHT_CHAIN_LTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_lsolver_[RIGHT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the right arm-left finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("right arm - left finger", chain_);

  // left arm - right finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", LEFT_CHAIN_RTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_rsolver_[LEFT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the left arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("left arm - right finger", chain_);

  // left arm - left finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", LEFT_CHAIN_LTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_lsolver_[LEFT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the left arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("left arm - left finger", chain_);

  jnt_pos_in_.resize(chain_.getNrOfJoints());
  jnt_pos_out_.resize(chain_.getNrOfJoints());
  num_joints_ = chain_.getNrOfJoints();
  ROS_DEBUG("[pviz] jnt_pos_in: rows: %d cols: %d", jnt_pos_in_.rows(), jnt_pos_out_.columns());

  // head (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", "head_tilt_link", chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_hsolver_ = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the head chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("head", chain_);

  // tilt laser (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", "laser_tilt_link", chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the tilt laser. Exiting."); 
    return false;
  }
  fk_tsolver_ = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the tilt laser chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("laser", chain_);

  return true;
}

bool PViz::computeFKwithKDL(const std::vector<double> &angles, std::vector<double> &base_pos, double torso_pos, int arm, int frame_num, geometry_msgs::Pose &pose)
{
  KDL::Frame frame_out;

  // head
  if(arm == HEAD)
  {
    KDL::JntArray head_pos_in;
    head_pos_in.resize(3);
    head_pos_in(0) = torso_pos;
    head_pos_in(1) = 0; // pan
    head_pos_in(2) = 0; // tilt
    if(base_pos.size() > 3)
    {
      head_pos_in(1) = base_pos[3]; // pan
      if(base_pos.size() > 4)
        head_pos_in(2) = base_pos[4]; // tilt
    }

    if(fk_hsolver_->JntToCart(head_pos_in, frame_out, frame_num) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
      return false;
    }
  }
  else if(arm == TILT) // tilt laser
  {
    KDL::JntArray tilt_pos_in;
    tilt_pos_in.resize(2);
    tilt_pos_in(0) = torso_pos;
    tilt_pos_in(1) = 0;
    if(base_pos.size() > 5)
      tilt_pos_in(1) = base_pos[5]; // tilt mount

    if(fk_tsolver_->JntToCart(tilt_pos_in, frame_out, frame_num) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
      return false;
    }
  }
  else // body
  {
    jnt_pos_in_(0) = torso_pos;
    for(size_t i = 0; i < angles.size(); ++i)
      jnt_pos_in_(i+1) = angles[i];

    if(angles.size() == 7)
    {
      jnt_pos_in_(8) = 0;
      jnt_pos_in_(9) = 0;
    }
    else if(angles.size() == 8)
    {
      jnt_pos_in_(9) = jnt_pos_in_(8);
    }

    // a negative frame number means that frame is in the left finger chain
    if(frame_num > 0)
    {
      if(fk_rsolver_[arm]->JntToCart(jnt_pos_in_, frame_out, frame_num) < 0)
      {
        ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
        return false;
      }
    }
    else
    {
      if(fk_lsolver_[arm]->JntToCart(jnt_pos_in_, frame_out, -1*frame_num) < 0)
      {
        ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
        return false;
      }
    }
  }

  KDL::Frame map_to_robot;
  getMaptoRobotTransform(base_pos[0],base_pos[1],base_pos[2],map_to_robot);

  frame_out = map_to_robot*frame_out;
  
  pose.position.x = frame_out.p[0];
  pose.position.y = frame_out.p[1];
  pose.position.z = frame_out.p[2];

  frame_out.M.GetQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
  return true;
}

void PViz::visualizeObstacles(const std::vector<std::vector<double> > &obstacles)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(obstacles.size());

  ROS_INFO("[pviz] Displaying %d obstaclesin the %s frame", (int)obstacles.size(), reference_frame_.c_str());

  std::string ns = "obstacles"+boost::lexical_cast<std::string>(rand());

  for(int i = 0; i < int(obstacles.size()); i++)
  {
    if(obstacles[i].size() < 6)
    {
      ROS_WARN("[pviz] Obstacle description doesn't have length = 6");
      continue;
    }

    //TODO: Change this to use a CUBE_LIST
    marker_array_.markers[i].header.stamp = ros::Time::now();
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].id = rand();
    marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose.position.x = obstacles[i][0];
    marker_array_.markers[i].pose.position.y = obstacles[i][1];
    marker_array_.markers[i].pose.position.z = obstacles[i][2];
    marker_array_.markers[i].scale.x = obstacles[i][3];
    marker_array_.markers[i].scale.y = obstacles[i][4];
    marker_array_.markers[i].scale.z = obstacles[i][5];
    marker_array_.markers[i].color.r = 0.0;
    marker_array_.markers[i].color.g = 0.0;
    marker_array_.markers[i].color.b = 0.5;
    marker_array_.markers[i].color.a = 0.9;
    marker_array_.markers[i].lifetime = ros::Duration(0.0);
  }

  marker_array_publisher_.publish(marker_array_);
}

void PViz::getCubeMsg(std::vector<double> &cube, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker)
{
  if(cube.size() < 6)
  {
    ROS_WARN("[pviz] Three dimensions are needed to visualize a cube.");
    return;
  }
  if(color.size() < 4)
  {
    ROS_ERROR("[pviz] No color specified.");
    return;
  }

  for(size_t i = 0; i < color.size(); ++i)
  {
    if(color[i] > 1)
      color[i] = color[i] / 255.0;
  }

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base_footprint";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cube[0];
  marker.pose.position.y= cube[1];
  marker.pose.position.z = cube[2];
  marker.pose.orientation.w = 1;
  marker.scale.x = cube[3];
  marker.scale.y = cube[4];
  marker.scale.z = cube[5];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(0.0);
}

void PViz::getCubeMsg(geometry_msgs::Pose &pose, std::vector<double> &dim, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker)
{
  if(dim.size() < 3)
  {
    ROS_WARN("[pviz] Three dimensions are needed to visualize a cube.");
    return;
  }
  if(color.size() < 4)
  {
    ROS_ERROR("[pviz] No color specified.");
    return;
  }

  for(size_t i = 0; i < color.size(); ++i)
  {
    if(color[i] > 1)
      color[i] = color[i] / 255.0;
  }

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base_footprint";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = dim[0];
  marker.scale.y = dim[1];
  marker.scale.z = dim[2];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(0.0);
}

void PViz::publish(const visualization_msgs::Marker& marker)
{
  marker_publisher_.publish(marker);
}

void PViz::publish(const visualization_msgs::MarkerArray &marker_array)
{
  marker_array_publisher_.publish(marker_array);
}

void PViz::publishMarker(visualization_msgs::Marker& marker)
{
  marker_publisher_.publish(marker);
}

void PViz::publishMarkerArray(visualization_msgs::MarkerArray &marker_array)
{
  marker_array_publisher_.publish(marker_array);
}

void PViz::visualizePoses(const std::vector<std::vector<double> > &poses)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(poses.size()*3);
  tf::Quaternion pose_quaternion;
  geometry_msgs::Quaternion quaternion_msg;

  int mind = -1;

  ros::Time time = ros::Time::now();

  for(int i = 0; i < (int)poses.size(); ++i)
  {
    pose_quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
    tf::quaternionTFToMsg(pose_quaternion, quaternion_msg);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_arrows";
    marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].pose.orientation = quaternion_msg;
    marker_array_.markers[mind].scale.x = 0.1;
    marker_array_.markers[mind].scale.y = 0.015;
    marker_array_.markers[mind].scale.z = 0.015;
    marker_array_.markers[mind].color.r = 0.0;
    marker_array_.markers[mind].color.g = 0.7;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.7;
    marker_array_.markers[mind].lifetime = ros::Duration(0.0);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_spheres";
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].pose.orientation = quaternion_msg;
    marker_array_.markers[mind].scale.x = 0.07;
    marker_array_.markers[mind].scale.y = 0.07;
    marker_array_.markers[mind].scale.z = 0.07;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 0.0;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.8;
    marker_array_.markers[mind].lifetime = ros::Duration(0.0);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_text_blocks";
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].scale.x = 0.015; //0.3
    marker_array_.markers[mind].scale.y = 0.015;
    marker_array_.markers[mind].scale.z = 0.015;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 1.0;
    marker_array_.markers[mind].color.b = 1.0;
    marker_array_.markers[mind].color.a = 0.95;
    marker_array_.markers[mind].text = boost::lexical_cast<std::string>(i+1);
    marker_array_.markers[mind].lifetime = ros::Duration(0.0);
  }

  ROS_DEBUG("[pviz] %d markers in the array",(int)marker_array_.markers.size());
  marker_array_publisher_.publish(marker_array_);
}

void PViz::visualizePose(const std::vector<double> &pose, std::string text)
{
  tf::Quaternion pose_quaternion;
  geometry_msgs::Pose pose_msg;

  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];

  pose_quaternion.setRPY(pose[3],pose[4],pose[5]);
  tf::quaternionTFToMsg(pose_quaternion, pose_msg.orientation);

  ROS_DEBUG("[pviz] [%s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose[0], pose[1], pose[2], pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w, reference_frame_.c_str());

  visualizePose(pose_msg, text);
}

void PViz::visualizePose(const geometry_msgs::Pose &pose, std::string text)
{
  int mind = -1;
  marker_array_.markers.clear();
  marker_array_.markers.resize(3);
  ros::Time time = ros::Time::now();

  ROS_DEBUG("[pviz] [%s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, reference_frame_.c_str());
  
  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text;
  marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
  marker_array_.markers[mind].id = 0;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.1;
  marker_array_.markers[mind].scale.y = 0.015;
  marker_array_.markers[mind].scale.z = 0.015;
  marker_array_.markers[mind].color.r = 0.0;
  marker_array_.markers[mind].color.g = 0.7;
  marker_array_.markers[mind].color.b = 0.6;
  marker_array_.markers[mind].color.a = 0.7;
  marker_array_.markers[mind].lifetime = ros::Duration(0.0);

  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text;
  marker_array_.markers[mind].id = 1;
  marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.07;
  marker_array_.markers[mind].scale.y = 0.07;
  marker_array_.markers[mind].scale.z = 0.07;
  marker_array_.markers[mind].color.r = 1.0;
  marker_array_.markers[mind].color.g = 0.0;
  marker_array_.markers[mind].color.b = 0.6;
  marker_array_.markers[mind].color.a = 0.6;
  marker_array_.markers[mind].lifetime = ros::Duration(0.0);

  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text;
  marker_array_.markers[mind].id = 2;
  marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].pose.position.z += 0.05;
  marker_array_.markers[mind].scale.x = 0.03;
  marker_array_.markers[mind].scale.y = 0.03;
  marker_array_.markers[mind].scale.z = 0.03;
  marker_array_.markers[mind].color.r = 1.0;
  marker_array_.markers[mind].color.g = 1.0;
  marker_array_.markers[mind].color.b = 1.0;
  marker_array_.markers[mind].color.a = 0.9;
  marker_array_.markers[mind].text = text;
  marker_array_.markers[mind].lifetime = ros::Duration(0.0);

  marker_array_publisher_.publish(marker_array_);
}

void PViz::visualizePose(const geometry_msgs::Pose &pose, std::string text, std::string frame_id)
{
  int mind = -1;
  marker_array_.markers.clear();
  marker_array_.markers.resize(3);
  ros::Time time = ros::Time::now();

  ROS_DEBUG("[pviz] [%s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, frame_id.c_str());
  
  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = frame_id;
  marker_array_.markers[mind].ns = text;
  marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
  marker_array_.markers[mind].id = 0;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.1;
  marker_array_.markers[mind].scale.y = 0.015;
  marker_array_.markers[mind].scale.z = 0.015;
  marker_array_.markers[mind].color.r = 0.0;
  marker_array_.markers[mind].color.g = 0.7;
  marker_array_.markers[mind].color.b = 0.6;
  marker_array_.markers[mind].color.a = 0.7;
  marker_array_.markers[mind].lifetime = ros::Duration(0.0);

  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = frame_id;
  marker_array_.markers[mind].ns = text;
  marker_array_.markers[mind].id = 1;
  marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.07;
  marker_array_.markers[mind].scale.y = 0.07;
  marker_array_.markers[mind].scale.z = 0.07;
  marker_array_.markers[mind].color.r = 1.0;
  marker_array_.markers[mind].color.g = 0.0;
  marker_array_.markers[mind].color.b = 0.6;
  marker_array_.markers[mind].color.a = 0.6;
  marker_array_.markers[mind].lifetime = ros::Duration(0.0);

  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = frame_id;
  marker_array_.markers[mind].ns = text;
  marker_array_.markers[mind].id = 2;
  marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].pose.position.z += 0.05;
  marker_array_.markers[mind].scale.x = 0.03;
  marker_array_.markers[mind].scale.y = 0.03;
  marker_array_.markers[mind].scale.z = 0.03;
  marker_array_.markers[mind].color.r = 1.0;
  marker_array_.markers[mind].color.g = 1.0;
  marker_array_.markers[mind].color.b = 1.0;
  marker_array_.markers[mind].color.a = 0.9;
  marker_array_.markers[mind].text = text;
  marker_array_.markers[mind].lifetime = ros::Duration(0.0);

  marker_array_publisher_.publish(marker_array_);
}

void PViz::visualizeSphere(std::vector<double> pose, int color, std::string text, double radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = text + "-sphere";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];
  marker.scale.x = radius*2;
  marker.scale.y = radius*2;
  marker.scale.z = radius*2;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.0);

  marker_publisher_.publish(marker);
}

void PViz::visualizeSphere(double x, double y, double z, double radius, int hue, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale.x = radius*2;
  marker.scale.y = radius*2;
  marker.scale.z = radius*2;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.0);
  marker_publisher_.publish(marker);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = "spheres-" + text;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = radius*2.0;
  marker.scale.y = radius*2.0;
  marker.scale.z = radius*2.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.6;
  marker.lifetime = ros::Duration(0.0);
  marker.id = 1;

  marker.points.resize(pose.size());
  for(size_t i = 0; i < pose.size(); i++)
  {
    marker.points[i].x = pose[i][0];
    marker.points[i].y = pose[i][1];
    marker.points[i].z = pose[i][2];
  }

  marker_publisher_.publish(marker);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, std::vector<double> &radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  for(size_t i = 0; i < pose.size(); ++i)
  {
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = text;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = radius[i]*2.0;
    marker.scale.y = radius[i]*2.0;
    marker.scale.z = radius[i]*2.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(0.0);
    marker.id = i;

    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];

    marker_publisher_.publish(marker);
    usleep(100);
  }
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  for(size_t i = 0; i < pose.size(); ++i)
  {
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = text;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = pose[i][3]*2.0;
    marker.scale.y = pose[i][3]*2.0;
    marker.scale.z = pose[i][3]*2.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(0.0);
    marker.id = i;

    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];

    marker_array.markers.push_back(marker);
  }
  marker_array_publisher_.publish(marker_array);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, const std::vector<int> &hue, std::string text)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;

  if(pose.size() != hue.size())
  {
    ROS_WARN("[pviz] Didn't receive as many colors as I did spheres. Not visualizing. (spheres: %d, colors: %d)", int(pose.size()), int(hue.size()));
    return;
  }

  for(std::size_t i = 0; i < pose.size(); ++i)
  {
    HSVtoRGB(&r, &g, &b, hue[i], 1.0, 1.0);
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = text;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = pose[i][3]*2.0;
    marker.scale.y = pose[i][3]*2.0;
    marker.scale.z = pose[i][3]*2.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(0.0);
    marker.id = i;
    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];
    marker_array.markers.push_back(marker);
  }
  marker_array_publisher_.publish(marker_array);
}

void PViz::deleteVisualizations(std::string ns, int max_id)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(max_id);

  for(int j = 0; j < max_id; j++)
  {
    marker_array_.markers[j].header.stamp = ros::Time::now();
    marker_array_.markers[j].header.frame_id = reference_frame_;
    marker_array_.markers[j].ns = ns;
    marker_array_.markers[j].action = visualization_msgs::Marker::DELETE;
    marker_array_.markers[j].id = j;
  }
  marker_array_publisher_.publish(marker_array_);
}

void PViz::visualize3DPath(std::vector<std::vector<double> > &dpath)
{
  if(dpath.empty())
  {
    ROS_INFO("[visualizeShortestPath] The shortest path is empty.");
    return;
  }
  else
    ROS_INFO("[visualizeShortestPath] There are %i waypoints in the shortest path.",int(dpath.size()));

  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = reference_frame_;
  obs_marker.header.stamp = ros::Time();
  obs_marker.header.seq = 0;
  obs_marker.ns = "path";
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  obs_marker.action = 0;
  obs_marker.scale.x = 3*0.02;
  obs_marker.scale.y = 3*0.02;
  obs_marker.scale.z = 3*0.02;
  obs_marker.color.r = 0.45;
  obs_marker.color.g = 0.3;
  obs_marker.color.b = 0.4;
  obs_marker.color.a = 0.8;
  obs_marker.lifetime = ros::Duration(0.0);

  obs_marker.points.resize(dpath.size());

  for (int k = 0; k < int(dpath.size()); k++)
  {
    if(int(dpath[k].size()) < 3)
      continue;

    obs_marker.points[k].x = dpath[k][0];
    obs_marker.points[k].y = dpath[k][1];
    obs_marker.points[k].z = dpath[k][2];
  }

  marker_publisher_.publish(obs_marker);
}

void PViz::visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size)
{
  unsigned int inc = 1;
  visualization_msgs::Marker marker;
  
  //check if the list is empty
  if(states.empty())
  {
    ROS_DEBUG("[visualizeBasicStates] There are no states in the %s states list.", name.c_str());
    return;
  }

  //if there are too many states, rviz will crash and burn when drawing
  if(states.size() > 50000)
    inc = 4;
  else if(states.size() > 10000)
    inc = 1;   //2
  else
    inc = 1;

  marker.points.resize(states.size()/inc + 1);

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;

  marker.ns = name;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(0.0);

  unsigned int m_ind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    if(states[i].size() >= 3)
    {
      marker.points[m_ind].x = states[i][0];
      marker.points[m_ind].y = states[i][1];
      marker.points[m_ind].z = states[i][2];
      ++m_ind;
    }
  }

  marker_publisher_.publish(marker);
  ROS_DEBUG("[visualizeBasicStates] published %d markers for %s states", int(marker.points.size()), name.c_str());
}

void PViz::visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size)
{
  unsigned int inc = 1;
  std::vector<double> scaled_color(4,0);
  visualization_msgs::MarkerArray marker_array;

  //check if the list is empty
  if(states.empty())
  {
    ROS_INFO("[aviz] There are no states in the %s states list", name.c_str());
    return;
  } 
  else
    ROS_INFO("[aviz] There are %i states in the %s states list.",int(states.size()),name.c_str());
    
  if(color.size()<2)
  {
    ROS_INFO("[aviz] Not enough colors specified.");
    return;
  } 
  
  if(color[0].size() < 4 || color[1].size() < 4)
  {
    ROS_INFO("[aviz] RGBA must be specified for each color.");
    return;
  } 
  
  //if there are too many states, rviz will crash and burn when drawing
  /*
  if(states.size() > 50000)
    inc = 20;
  else if(states.size() > 5000)
    inc = 10;
  else if(states.size() > 500)
    inc = 1;   //changed  8/31/11
  else
    inc = 1;
  */

  unsigned int mind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    marker_array.markers.resize(marker_array.markers.size()+1);
    marker_array.markers[mind].header.frame_id = reference_frame_;
    marker_array.markers[mind].header.stamp = ros::Time::now();
    marker_array.markers[mind].ns = "expanded_states";
    marker_array.markers[mind].id = mind;
    marker_array.markers[mind].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[mind].action =  visualization_msgs::Marker::ADD;
    marker_array.markers[mind].scale.x = size;
    marker_array.markers[mind].scale.y = size;
    marker_array.markers[mind].scale.z = size;
    
    for(unsigned int j = 0; j < 4; ++j)
      scaled_color[j] = color[0][j] - ((color[0][j] - color[1][j]) * (double(i)/double(states.size()/inc)));
      
    marker_array.markers[mind].color.r = scaled_color[0];
    marker_array.markers[mind].color.g = scaled_color[1];
    marker_array.markers[mind].color.b = scaled_color[2];
    marker_array.markers[mind].color.a = 1;
    marker_array.markers[mind].lifetime = ros::Duration(0.0);
    
    marker_array.markers[mind].pose.position.x = states[i][0];
    marker_array.markers[mind].pose.position.y = states[i][1];
    marker_array.markers[mind].pose.position.z = states[i][2];
    
    ++mind;
  } 
  
  ROS_DEBUG("[aviz] published %d markers for %s states", (int)marker_array.markers.size(), name.c_str());
  marker_array_publisher_.publish(marker_array);
}

void PViz::visualizeLine(const std::vector<geometry_msgs::Point> points, std::string ns, int id, int hue, double thickness)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points = points;
  marker.scale.x = thickness;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(0.0);

  ROS_DEBUG("[pviz] Visualizing a line with %d points", int(points.size()));
  marker_publisher_.publish(marker);
}

void PViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, int hue)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.pose = pose;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.text = text;
  marker.lifetime = ros::Duration(0.0);

  marker_publisher_.publish(marker);
}

void PViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, std::vector<double> color, double size)
{
  visualization_msgs::Marker marker;

  if(color.size() < 4)
    color.resize(4,1);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.pose = pose;
  
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.text = text;
  marker.lifetime = ros::Duration(0.0);

  marker_publisher_.publish(marker);
}

void PViz::visualizeCube(geometry_msgs::PoseStamped pose, int color, std::string ns, int id, std::vector<double> dim)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  if(dim.size() < 3)
  {
    ROS_INFO("[aviz] Three dimensions are needed to visualize a cube.");
    if(dim.size() > 1)
      dim.resize(3,dim[0]);
    else
      return;
  }

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = pose.header.frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose.pose;
  marker.scale.x = dim[0];
  marker.scale.y = dim[1];
  marker.scale.z = dim[2];
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.0);

  marker_publisher_.publish(marker);
}

void PViz::visualizeMesh(const std::string& mesh_resource, const geometry_msgs::PoseStamped& pose, int color,
                         std::string ns, int id)
{
	double r = 0.0, g = 0.0, b = 0.0;
	HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;
	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.mesh_resource = mesh_resource;

	marker_publisher_.publish(marker);
}

void PViz::visualizeMeshTriangles(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
                         const geometry_msgs::PoseStamped& pose, int color, std::string ns, int id, bool psychadelic)
{
	double r = 0.0, g = 0.0, b = 0.0;
	HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

	std_msgs::ColorRGBA red; red.a = 1.0f; red.r = 1.0f; red.g = 0.0f; red.b = 0.0f;
	std_msgs::ColorRGBA green; green.a = 1.0f; green.r = 0.0f; green.g = 1.0f; green.b = 0.0f;
	std_msgs::ColorRGBA blue; blue.a = 1.0f; blue.r = 0.0f; blue.g = 0.0f; blue.b = 1.0f;

	std::vector<std_msgs::ColorRGBA> colors;
	for (int i = 0; i < (int)vertices.size(); i++) {
		if (i % 3 == 0) colors.push_back(red);
		if (i % 3 == 1) colors.push_back(green);
		if (i % 3 == 2) colors.push_back(blue);
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;
	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.points = vertices;

	if (psychadelic) {
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.colors = colors;
	}
	else {
		marker.color.a = 1.0;
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
	}

	marker_publisher_.publish(marker);
}

void PViz::printKDLChain(std::string name, KDL::Chain &chain)
{
  ROS_INFO("chain: %s", name.c_str());
  for(unsigned int j = 0; j < chain.getNrOfSegments(); ++j)
  {
    ROS_INFO("  frame %2d: segment: %0.3f %0.3f %0.3f  joint: %0.3f %0.3f %0.3f   joint_type: %s",j,
        chain.getSegment(j).pose(0).p.x(),
        chain.getSegment(j).pose(0).p.y(),
        chain.getSegment(j).pose(0).p.z(),
        chain.getSegment(j).getJoint().pose(0).p.x(),
        chain.getSegment(j).getJoint().pose(0).p.y(),
        chain.getSegment(j).getJoint().pose(0).p.z(),
        chain.getSegment(j).getJoint().getTypeName().c_str());
  }
  ROS_INFO(" ");
}

bool PViz::computeFKforVisualizationWithKDL(const std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, std::vector<geometry_msgs::PoseStamped> &poses)
{
  //output is PoseStamped only so it has same interface as computeFKforVisualization
  //KDL::Frame map_to_robot; 
  geometry_msgs::Pose pose;

/*
[ INFO]:   frame  0: segment: 0.000 0.000 0.051  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  1: segment: -0.050 0.000 0.740  joint: -0.050 0.000 0.740   joint_type: TransAxis
[ INFO]:   frame  2: segment: 0.000 0.188 0.000  joint: 0.000 0.188 0.000   joint_type: RotAxis
[ INFO]:   frame  3: segment: 0.100 0.000 0.000  joint: 0.100 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  4: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  5: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  6: segment: 0.400 0.000 0.000  joint: 0.400 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  7: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  8: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  9: segment: 0.321 0.000 0.000  joint: 0.321 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame 10: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame 11: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame 12: segment: 0.077 0.010 0.000  joint: 0.077 0.010 0.000   joint_type: RotAxis
[ INFO]:   frame 13: segment: 0.091 0.005 0.000  joint: 0.091 0.005 0.000   joint_type: RotAxis

  0 base_v0/base.stl
  1 torso_v0/torso_lift.stl
  2 shoulder_v0/shoulder_yaw.stl
  3 shoulder_v0/shoulder_lift.stl
  4 upper_arm_roll
  5 upper_arm_v0/upper_arm.stl
  6 upper_arm_v0/elbow_flex.stl
  7 upper_arm_v0/forearm_roll.stl
  8 forearm_v0/forearm.stl
  9 forearm_v0/wrist_flex.stl
  10 forearm_v0/wrist_roll.stl
  11 gripper_v0/gripper_palm.stl
  12 gripper_v0/upper_finger_r.stl
  13 gripper_v0/finger_tip_r.stl
 
  12 gripper_v0/upper_finger_l.stl
  13 gripper_v0/finger_tip_l.stl
  

  2 shoulder_v0/shoulder_yaw.stl
  3 shoulder_v0/shoulder_lift.stl
  4 upper_arm_roll
  5 upper_arm_v0/upper_arm.stl
  6 upper_arm_v0/elbow_flex.stl
  7 upper_arm_v0/forearm_roll.stl
  8 forearm_v0/forearm.stl
  9 forearm_v0/wrist_flex.stl
  10 forearm_v0/wrist_roll.stl
  11 gripper_v0/gripper_palm.stl
  12 gripper_v0/upper_finger_r.stl
  13 gripper_v0/finger_tip_r.stl
  12 gripper_v0/upper_finger_l.stl
  13 gripper_v0/finger_tip_l.stl
 
  30 meshes total

*/

  //getMapToRobotTransform(base_pos[0],base_pos[1],base_pos[2],map_to_robot)

  poses.resize(robot_meshes_.size());
  for(int i=0; i <  int(poses.size()); i++)
  {
    // base, torso, right arm thru the right finger
    if(i < 14)
    {
      if(!computeFKwithKDL(jnt0_pos, base_pos, torso_pos, RIGHT, i+1, poses[i].pose))
        return false;

      // right arm, right finger (to rotate the mesh)
      if(i == 13 || i == 12)
      {
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
        multiply(poses[i].pose, p, poses[i].pose);
      }
    }
    // right arm - left finger only
    else if(13 < i && i < 16)
    {
      if(!computeFKwithKDL(jnt0_pos, base_pos, torso_pos, RIGHT, -1*(i-1), poses[i].pose))
        return false;
    }
    // left arm thru the right finger
    else if(15 < i && i < 28)
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, LEFT, i-13, poses[i].pose))
        return false;

      if(i == 26 || i == 27)
      {
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
        multiply(poses[i].pose, p, poses[i].pose);
      }
    }
    // left arm - left finger only
    else if(27 < i && i < 30) 
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, LEFT, -1*(i-15), poses[i].pose))
        return false;
    }
    // head pan, head tilt
    else if(29 < i && i < 32)
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, HEAD, i-27, poses[i].pose))
        return false;
    }
    // tilt laser
    else if(31 < i)
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, TILT, i-29, poses[i].pose))
        return false;
    }
  }

  return true;
}

void PViz::visualizeRobotMeshes(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials)
{
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(robot_meshes_.size());
  ros::Time time = ros::Time();

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].header.stamp = time;
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array_.markers[i].id = id + i;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose = poses.at(i).pose;
    marker_array_.markers[i].scale.x = 1.0;
    marker_array_.markers[i].scale.y = 1.0;
    marker_array_.markers[i].scale.z = 1.0;
    if(use_embedded_materials)
    {
      marker_array_.markers[i].color.r = 0;
      marker_array_.markers[i].color.g = 0;
      marker_array_.markers[i].color.b = 0;
      marker_array_.markers[i].color.a = 0;
      marker_array_.markers[i].mesh_use_embedded_materials = true;
    }
    else
    {
      marker_array_.markers[i].color.r = r;
      marker_array_.markers[i].color.g = g;
      marker_array_.markers[i].color.b = b;
      marker_array_.markers[i].color.a = 0.4;
    }
    marker_array_.markers[i].lifetime = ros::Duration(0.0);
    marker_array_.markers[i].mesh_resource = robot_meshes_[i];
  }
  marker_array_publisher_.publish(marker_array_);
}

visualization_msgs::MarkerArray PViz::getRobotMeshesMarkerMsg(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials)
{
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(robot_meshes_.size());
  ros::Time time = ros::Time();
  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].header.stamp = time;
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array_.markers[i].id = id + i;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose = poses.at(i).pose;
    marker_array_.markers[i].scale.x = 1.0;
    marker_array_.markers[i].scale.y = 1.0;
    marker_array_.markers[i].scale.z = 1.0;
    if(use_embedded_materials)
    {
      marker_array_.markers[i].color.r = 0;
      marker_array_.markers[i].color.g = 0;
      marker_array_.markers[i].color.b = 0;
      marker_array_.markers[i].color.a = 0;
      marker_array_.markers[i].mesh_use_embedded_materials = true;
    }
    else
    {
      marker_array_.markers[i].color.r = r;
      marker_array_.markers[i].color.g = g;
      marker_array_.markers[i].color.b = b;
      marker_array_.markers[i].color.a = 0.4;
    }
    marker_array_.markers[i].lifetime = ros::Duration(0.0);
    marker_array_.markers[i].mesh_resource = robot_meshes_[i];
  }
  return marker_array_;
}

void PViz::getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame)
{
  KDL::Rotation r1;
  r1.DoRotZ(theta);
  KDL::Vector t1(x,y,0.0);
  KDL::Frame base_footprint_in_map(r1,t1);
  frame = base_footprint_in_map;
}

void PViz::visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, double hue, std::string ns, int id, bool use_embedded_materials)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))

    ROS_WARN("Unable to compute forward kinematics.");
  else
    visualizeRobotMeshes(hue, ns, id, poses, use_embedded_materials);
}

void PViz::visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, bool use_embedded_materials)
{
  double torso_pos;
  std::vector<double> base_pos(3,0);

  base_pos[0] = body_pos.x;
  base_pos[1] = body_pos.y;
  base_pos[2] = body_pos.theta;
  torso_pos = body_pos.z;

  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))

    ROS_WARN("Unable to compute forward kinematics.");
  else
    visualizeRobotMeshes(hue, ns, id, poses, use_embedded_materials);
}

visualization_msgs::MarkerArray PViz::getRobotMarkerMsg(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, bool use_embedded_materials)
{
  double torso_pos;
  std::vector<double> base_pos(3,0);

  base_pos[0] = body_pos.x;
  base_pos[1] = body_pos.y;
  base_pos[2] = body_pos.theta;
  torso_pos = body_pos.z;

  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))
  {
    ROS_WARN("[pviz] Failed to compute forward kinematics. Cannot visualize robot.");
    return visualization_msgs::MarkerArray();
  } 
  else
    return getRobotMeshesMarkerMsg(hue, ns, id, poses, use_embedded_materials);
}

bool PViz::parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data)
{
  // NOTE: This function requires that every line ends with a comma as well.
  std::ifstream input_file(filename.c_str());
  if(!input_file.good())
  {
    printf("[pviz] Unable to open '%s' for reading.\n",filename.c_str());
    return false;
  }

  int row(0), col(0);
  char line[256];
  input_file.seekg(0);

  std::vector<std::vector<double> > raw_data;

  row = -1;
  raw_data.clear();

  while (input_file.getline(line, 256, ','))
  {
    raw_data.resize(raw_data.size()+1);
    raw_data[++row].resize(num_cols);
    raw_data[row][0] = atof(line);

    for(col = 1; col < num_cols; col++)
    {
      input_file.getline(line, 256, ',');
      raw_data[row][col] = atof(line);
    }
  }

  std::vector<double> zero_line(num_cols,0);
  for(int i = 0; i < int(raw_data.size()); i++)
  {
    if(raw_data[i] != zero_line)
      data.push_back(raw_data[i]);
  }

  ROS_DEBUG("[pviz] raw_data: num rows: %d",(int)raw_data.size());
  ROS_DEBUG("[pviz] data: num rows: %d num_cols: %d",(int)data.size(), int(data[0].size()));

  /*
  printf("Trajectory:\n");
  for(size_t i = 0; i < data.size(); ++i)
  {
    printf("%d: ", int(i));
    for(size_t j = 0; j < data[i].size(); ++j)
    {
      printf("% 0.2f ", data[i][j]);
    }
    printf("\n");
  }
  */
  return true;
}

bool PViz::visualizeTrajectoryFromFile(std::string filename, bool use_embedded_materials)
{
  double torso = 0;
  std::vector<double> jnt0_pos(7,0), jnt1_pos(7,0), base_pos(3,0);
  std::vector<std::vector<double> > traj;
  if(!parseCSVFile(filename, 18, traj))
  {
    ROS_ERROR("[pviz] Failed to parse trajectory file: %s", filename.c_str());
    return false;
  }

  for(size_t i = 0; i < traj.size(); ++i)
  {
    for(size_t j = 0; j < 7; ++j)
    {
      jnt0_pos[j] = traj[i][4+j];
      jnt1_pos[j] = traj[i][12+j];
    }

    base_pos[0] = traj[i][0];
    base_pos[1] = traj[i][1];
    base_pos[2] = traj[i][2];
    torso = traj[i][3];

    ROS_INFO("[pviz] %d: visualizing robot...", int(i));
    visualizeRobot(jnt0_pos,jnt1_pos,base_pos,torso,(30*i)%300,"full_body_waypoint_"+boost::lexical_cast<std::string>(i),i, use_embedded_materials);
    ros::spinOnce();
    usleep(10000);
  }
  return true;
}

void PViz::visualizeRobotWithTitle(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, std::string title)
{
  visualizeRobot(jnt0_pos, jnt1_pos, body_pos, hue, ns, id);

  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns + "_title";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.pose.position.x = body_pos.x;
  marker.pose.position.y = body_pos.y;
  marker.pose.position.z = body_pos.z+1.5;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.5;
  marker.text = title;
  marker.lifetime = ros::Duration(0.0);
  marker_publisher_.publish(marker);
}

void PViz::visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath, int throttle)
{
  int length = rpath.size();
  std::vector<double> rangles(7, 0), langles(7, 0);
  BodyPose body_pos;
  visualization_msgs::MarkerArray ma, ma1;

  if (rpath.size() != lpath.size() || rpath.size() != bpath.size()) {
    ROS_ERROR("[pviz] The right arm, left arm and body trajectories are of unequal lengths.");
    return;
  }

  int color_inc = 240.0 / (length / throttle); // hue: red -> blue
  ROS_DEBUG("[pviz] length: %d color_inc: %d throttle: %d)", length, color_inc, throttle);

  for (int i = 0; i < length; ++i) {
    for (std::size_t j = 0; j < rangles.size(); ++j) {
      rangles[j] = rpath[i].positions[j];
      langles[j] = lpath[i].positions[j];
    }
    body_pos.x = bpath[i].positions[0];
    body_pos.y = bpath[i].positions[1];
    body_pos.z = bpath[i].positions[2];
    body_pos.theta = bpath[i].positions[3];

    if ((i != length - 1) && (i % throttle != 0))
      continue;

    ROS_DEBUG("[pviz] length: %d color_inc: %d throttle: %d", length, color_inc, throttle);
    ROS_DEBUG("[pviz] Visualizing waypoint #%d (i mod color_inc: %d) with color: %d (color_inc: %d, throttle: %d)", i, (i / throttle), (i / throttle) * color_inc, color_inc, throttle);

    ma1 = getRobotMarkerMsg(rangles, langles, body_pos, (i / throttle) * color_inc, "robot_path", (i+1)*30);
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end()); 
  }
  ROS_INFO("[pviz] Visualizing a robot path with %d waypoints. (throttle = %d)", int(rpath.size()), throttle);
  marker_array_publisher_.publish(ma);
}

void PViz::visualizeSpheres(const std::vector<geometry_msgs::Point>  &poses, int hue, std::string ns, int id, double radius)
{
  if(poses.empty())
  {
    ROS_INFO("[visualizeShortestPath] The shortest path is empty.");
    return;
  }

  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;
  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.frame_id = reference_frame_;
  marker.header.stamp = ros::Time();
  marker.header.seq = 0;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = 0;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.8;
  marker.lifetime = ros::Duration(0.0);
  marker.points = poses;
  marker_publisher_.publish(marker);
}

void PViz::visualizeGripper(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open)
{
  visualization_msgs::MarkerArray m;
  getGripperMeshesMarkerMsg(pose, hue, ns, id, open, m.markers);
  publishMarkerArray(m);
}

void PViz::getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open, std::vector<visualization_msgs::Marker> &markers)
{
  static geometry_msgs::Pose rot;
  rot.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
  static geometry_msgs::Pose g1, g2, g3, g4, g5, g6, g7, g8;
  // g1 = r_gripper_r_finger_link in r_gripper_palm - OPEN
  g1.position.x = 0.077; g1.position.y = -0.010; g1.position.z = 0.00;
  g1.orientation.x = 0.0; g1.orientation.y = 0.0; g1.orientation.z = -0.231; g1.orientation.w = 0.973;
  multiply(g1, rot, g1);
  // g2 = r_gripper_l_finger_link in r_gripper_palm - OPEN
  g2.position.x = 0.077; g2.position.y = 0.010; g2.position.z = 0.00;
  g2.orientation.x = 0.0; g2.orientation.y = 0.0; g2.orientation.z = 0.231; g2.orientation.w = 0.973;
  // g2 = g1; g2.position.y = 0.010; g2.orientation.z = 0.231;
  // g3 = r_gripper_r_finger_tip_link in r_gripper_palm - OPEN
  g3.position.x = 0.156; g3.position.y = -0.056; g3.position.z = 0.00;
  g3.orientation.x = 0.0; g3.orientation.y = 0.0; g3.orientation.z = 0.0; g3.orientation.w = 1.000;
  multiply(g3, rot, g3);
  // g4 = r_gripper_l_finger_tip_link in r_gripper_palm - OPEN
  g4.position.x = 0.156; g4.position.y = 0.056; g4.position.z = 0.00;
  g4.orientation.x = 0.0; g4.orientation.y = 0.0; g4.orientation.z = 0.0; g4.orientation.w = 1.000;
  // g4 = g3; g4.position.y = 0.056;
  // g5 = r_gripper_r_finger_link in r_gripper_palm - CLOSED
  g5.position.x = 0.077; g5.position.y = -0.010; g5.position.z = 0.00;
  g5.orientation.x = 0.0; g5.orientation.y = 0.0; g5.orientation.z = 0.004; g5.orientation.w = 1.000;
  // g5 = g1; g5.orientation.x = 0.0; g5.orientation.y = 0.0; g5.orientation.z = 0.004; g5.orientation.w = 1.000;
  multiply(g5, rot, g5);
  // g6 = r_gripper_l_finger_link in r_gripper_palm - CLOSED
  g6.position.x = 0.077; g6.position.y = 0.010; g6.position.z = 0.00;
  g6.orientation.x = 0.0; g6.orientation.y = 0.0; g6.orientation.z = -0.004; g6.orientation.w = 1.000;
  // g6 = g5; g6.position.y = 0.010; g6.orientation.z = -0.004;
  // g7 = r_gripper_r_finger_tip_link in r_gripper_palm - CLOSED
  g7.position.x = 0.168; g7.position.y = -0.014; g7.position.z = 0.00;
  g7.orientation.x = 0.0; g7.orientation.y = 0.0; g7.orientation.z = 0.0; g7.orientation.w = 1.000;
  multiply(g7, rot, g7);
  // g8 = r_gripper_l_finger_tip_link in r_gripper_palm - CLOSED
  g8.position.x = 0.168; g8.position.y = 0.014; g8.position.z = 0.00;
  g8.orientation.x = 0.0; g8.orientation.y = 0.0; g8.orientation.z = 0.0; g8.orientation.w = 1.000;
  // g8 = g7; g8.position.y = 0.014;

  double r,g,b;
  visualization_msgs::Marker m;
  std::vector<geometry_msgs::Pose> p(4);

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  m.header.stamp = ros::Time::now();
  m.header.frame_id = "base_footprint";
  m.ns = ns;
  m.type = visualization_msgs::Marker::MESH_RESOURCE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0.0);

  if(open)
  {
    p[0] = g1;
    p[1] = g2;
    p[2] = g3;
    p[3] = g4;
  }
  else
  {
    p[0] = g5;
    p[1] = g6;
    p[2] = g7;
    p[3] = g8;
  }

  // palm
  m.mesh_resource = arm_meshes_[9];
  m.id = id;
  m.pose = pose;
  markers.push_back(m);

  // upper_finger_r
  m.mesh_resource = gripper_meshes_[0];
  m.id++;
  multiply(pose, p[0], m.pose);
  markers.push_back(m);

  // upper_finger_l
  m.mesh_resource = gripper_meshes_[2];
  m.id++;
  multiply(pose, p[1], m.pose);
  markers.push_back(m);

  m.color.r = 90.0/255.0;
  m.color.g = 90.0/255.0;
  m.color.b = 90.0/255.0;

  // finger_tip_r
  m.mesh_resource = gripper_meshes_[1];
  m.id++;
  multiply(pose, p[2], m.pose);
  markers.push_back(m);

  // finger_tip_l
  m.mesh_resource = gripper_meshes_[3];
  m.id++;
  multiply(pose, p[3], m.pose);
  markers.push_back(m);
}

void PViz::getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, 
				     std::string ns, int id, double position, 
				     std::vector<visualization_msgs::Marker> &markers)
{
  static geometry_msgs::Pose rot;
  rot.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
  // r_gripper_r_finger_link in r_gripper_palm
  static geometry_msgs::Pose rgrfl_in_rgp;
  rgrfl_in_rgp.position.x = 0.077;
  rgrfl_in_rgp.position.y = -0.010;
  rgrfl_in_rgp.position.z = 0.00;
  rgrfl_in_rgp.orientation.x = 0.0;
  rgrfl_in_rgp.orientation.y = 0.0;
  rgrfl_in_rgp.orientation.z = 0.004;
  rgrfl_in_rgp.orientation.w = 1.000;
  multiply(rgrfl_in_rgp, rot, rgrfl_in_rgp);

  // r_gripper_l_finger_link in r_gripper_palm
  static geometry_msgs::Pose rglfl_in_rgp;
  rglfl_in_rgp.position.x = 0.077;
  rglfl_in_rgp.position.y = 0.010;
  rglfl_in_rgp.position.z = 0.00;
  rglfl_in_rgp.orientation.x = 0.0;
  rglfl_in_rgp.orientation.y = 0.0;
  rglfl_in_rgp.orientation.z = -0.004;
  rglfl_in_rgp.orientation.w = 1.000;
    
  // r_gripper_r_finger_tip_link in r_gripper_r_finger_link
  static geometry_msgs::Pose rgrftl_in_rgrfl;
  rgrftl_in_rgrfl.position.x = 0.091;
  rgrftl_in_rgrfl.position.y = -0.005;
  rgrftl_in_rgrfl.position.z = 0.000;
  rgrftl_in_rgrfl.orientation.x = 0.000;
  rgrftl_in_rgrfl.orientation.y = 0.000;
  rgrftl_in_rgrfl.orientation.z = 0.001;
  rgrftl_in_rgrfl.orientation.w = 1.000;

  // r_gripper_l_finger_tip_link in r_gripper_l_finger_link
  static geometry_msgs::Pose rglftl_in_rglfl;
  rglftl_in_rglfl.position.x = 0.091;
  rglftl_in_rglfl.position.y = 0.005;
  rglftl_in_rglfl.position.z = 0.000;
  rglftl_in_rglfl.orientation.x = 0.000;
  rglftl_in_rglfl.orientation.y = 0.000;
  rglftl_in_rglfl.orientation.z = -0.001;
  rglftl_in_rglfl.orientation.w = 1.000;

  // Open the gripper appropriately.
  geometry_msgs::Pose open;
  open.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, position);
  multiply(rgrfl_in_rgp, open, rgrfl_in_rgp);
  multiply(rglfl_in_rgp, open, rglfl_in_rgp);

  open.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -position);
  multiply(rgrftl_in_rgrfl, open, rgrftl_in_rgrfl);
  multiply(rglftl_in_rglfl, open, rglftl_in_rglfl);

  geometry_msgs::Pose rgrftl_in_rgp;
  multiply(rgrfl_in_rgp, rgrftl_in_rgrfl, rgrftl_in_rgp);
  geometry_msgs::Pose rglftl_in_rgp;
  multiply(rglfl_in_rgp, rglftl_in_rglfl, rglftl_in_rgp);

  double r, g, b;
  visualization_msgs::Marker m;
  std::vector<geometry_msgs::Pose> p(4);
  p[0] = rgrfl_in_rgp;
  p[1] = rglfl_in_rgp;
  p[2] = rgrftl_in_rgp;
  p[3] = rglftl_in_rgp;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  m.header.stamp = ros::Time::now();
  m.header.frame_id = "/base_footprint";
  m.ns = ns;
  m.type = visualization_msgs::Marker::MESH_RESOURCE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;

  // palm
  m.mesh_resource = arm_meshes_[9];
  m.id = id;
  m.pose = pose;
  markers.push_back(m);

  // upper_finger_r
  m.mesh_resource = gripper_meshes_[0];
  m.id++;
  multiply(pose, p[0], m.pose);
  markers.push_back(m);

  // upper_finger_l
  m.mesh_resource = gripper_meshes_[2];
  m.id++;
  multiply(pose, p[1], m.pose);
  markers.push_back(m);

  m.color.r = 90.0/255.0;
  m.color.g = 90.0/255.0;
  m.color.b = 90.0/255.0;

  // finger_tip_r
  m.mesh_resource = gripper_meshes_[1];
  m.id++;
  multiply(pose, p[2], m.pose);
  markers.push_back(m);

  // finger_tip_l
  m.mesh_resource = gripper_meshes_[3];
  m.id++;
  multiply(pose, p[3], m.pose);
  markers.push_back(m);
}

void PViz::multiply(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, geometry_msgs::Pose &c)
{
  tf::Transform bta, btb, btc;
  tf::poseMsgToTF(a, bta);
  tf::poseMsgToTF(b, btb);
  btc = bta * btb;
  tf::poseTFToMsg(btc, c);
}

void PViz::visualizeText(double x, double y, double z, double size, std::string text, int hue, std::string ns, int id)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.text = text;
  marker.lifetime = ros::Duration(0.0);
  marker_publisher_.publish(marker);
}

