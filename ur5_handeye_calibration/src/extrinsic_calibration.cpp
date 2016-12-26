//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <ensenso/CapturePattern.h>
#include <ensenso/InitCalibration.h>
#include <ensenso/ComputeCalibration.h>
#include <ensenso/RegistImage.h>
#include <ensenso/ConfigureStreaming.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

//msg
#include <control_msgs/FollowJointTrajectoryAction.h>

//action
#include <actionlib/client/simple_action_client.h>

//trac_ik
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

//std
#include <string>
#include <iostream>

using namespace std;

Eigen::Affine3d generateRandomHemispherePose(const Eigen::Vector3d &obj_origin, const Eigen::Vector3d &tool_origin)
{
  // Generate random point on upper hemisphere :
  Eigen::Vector3d point;
  point[2] = obj_origin[2];
  double radius = (obj_origin - tool_origin).norm();

  while (point[2] < obj_origin[2] + 0.8 * radius)
  {
    double phy = rand() % 161 + 10;
    double teta = rand() % 360;

    point[0] = radius * cos(phy) * cos(teta) + obj_origin[0];
    point[1] = radius * cos(phy) * sin(teta) + obj_origin[1];
    point[2] = radius * sin(phy) + obj_origin[2];
  }

  // Z axis = obj_origin -> point
  Eigen::Vector3d z_axis;
  z_axis = obj_origin - point;
  z_axis.normalize();

  // Y axis = Generate random point on plane represented by Z vector
  Eigen::Vector3d y_axis;
  y_axis << z_axis[1], z_axis[0], z_axis[2];
  y_axis = y_axis.cross(z_axis); // Temporary y axis

  y_axis = y_axis + point;
  y_axis.normalize();

  // X axis = Cross product
  Eigen::Vector3d x_axis(y_axis.cross(z_axis));
  x_axis.normalize();

  // Recompute Y axis
  y_axis = (z_axis.cross(x_axis));
  y_axis.normalize();

  // Assign rotations and translation
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  pose.matrix().col(0) << x_axis, 0;
  pose.matrix().col(1) << y_axis, 0;
  pose.matrix().col(2) << z_axis, 0;
  pose.translation() = point;
  return pose;
}

void get_tool_to_camera_tf(tf::Vector3 cam_to_tool_position, tf::Quaternion cam_to_tool_orientation)
{
        tf::Transform cam_to_tool;
        cam_to_tool.setOrigin(cam_to_tool_position);
        cam_to_tool.setRotation(cam_to_tool_orientation);
        tf::Transform tool_to_cam=cam_to_tool.inverse();
        tf::Vector3 position=tool_to_cam.getOrigin();
        tf::Quaternion orientation=tool_to_cam.getRotation();
        std::cout<<"tool0 to camera Position: X Y Z "<<position.getX()<<" "<<position.getY()<<" "<<position.getZ()<<std::endl;
        std::cout<<"tool0 to camera Orientation: W X Y Z"<<orientation.getW()<<" "<<orientation.getX()<<" "<<orientation.getY()<<" "<<orientation.getZ()<<std::endl;

}

void tfToGeometryMsg(tf::StampedTransform& tf, geometry_msgs::Pose& pose)
{
    //Orientation
    pose.orientation.w=tf.getRotation().getW();
    pose.orientation.x=tf.getRotation().getX();
    pose.orientation.y=tf.getRotation().getY();
    pose.orientation.z=tf.getRotation().getZ();

    //Position
    pose.position.x=tf.getOrigin().getX();
    pose.position.y=tf.getOrigin().getY();
    pose.position.z=tf.getOrigin().getZ();

}

class ex_cal
{
    ros::NodeHandle nh;
    //Publisher
    ros::Publisher status_pub;
    ros::Publisher marker_pub;
    //Service client
    ros::ServiceClient capture_pattern_client;
    ros::ServiceClient init_cal_client;
    ros::ServiceClient compute_cal_client;
    ros::ServiceClient config_stream_client;
    //Service srv
    ensenso::InitCalibration init_cal_srv;
    ensenso::ComputeCalibration compute_cal_srv;
    //Default pose
    Eigen::Affine3d default_pose;
    //Grid space
    double grid_space_;

public:
    ex_cal(int num_poses,double ee_x,double ee_y,double ee_z,double calTabDist,double grid_space):
        num_poses_(num_poses),
        pos_x(ee_x),
        pos_y(ee_y),
        pos_z(ee_z),
        calTabDistance(calTabDist),
        grid_space_(grid_space)
    {
        //initialize action parameters
        nh.param("chain_start", chain_start_, std::string("base"));
        nh.param("chain_end", chain_end_, std::string("tool0"));
        base_name_=chain_start_;
        tcp_name_=chain_end_;
        nh.param("timeout", timeout_, 0.005);
        nh.param("urdf_param", urdf_param_, std::string("/robot_description"));
        nh.param("action_server",action_server_,std::string("/arm_controller/follow_joint_trajectory"));
        joint_names.push_back("shoulder_pan_joint");
        joint_names.push_back("shoulder_lift_joint");
        joint_names.push_back("elbow_joint");
        joint_names.push_back("wrist_1_joint");
        joint_names.push_back("wrist_2_joint");
        joint_names.push_back("wrist_3_joint");
        //initialize publisher
        status_pub=nh.advertise<std_msgs::String>("ensenso_calibration_status",1);
        marker_pub=nh.advertise<visualization_msgs::Marker>("hemisphere",1);
        //initialize service client
        capture_pattern_client = nh.serviceClient<ensenso::CapturePattern>("capture_pattern");
        init_cal_client = nh.serviceClient<ensenso::InitCalibration>("init_calibration");
        compute_cal_client = nh.serviceClient<ensenso::ComputeCalibration>("compute_calibration");
        config_stream_client = nh.serviceClient<ensenso::ConfigureStreaming>("configure_streaming");
        //Initialize default pose
        Eigen::Matrix4d m;
        m<< 0.0, 1.0, 0.0, 0.5,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, -1.0, 0.4,
            0.0, 0.0, 0.0, 1.0;
        default_pose = Eigen::Affine3d(m);
        int p=0;
        //Define marker
        sphere.header.frame_id="/base";
        sphere.lifetime = ros::Duration();
        sphere.color.r = 0.6f;
        sphere.color.g = 0.6f;
        sphere.color.b = 0.6f;
        sphere.color.a = 0.5;
        sphere.id = 0;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;

    }

    bool ur5_trac_ik(const std::string& chain_start, const std::string& chain_end,const Eigen::Affine3d& Pose,control_msgs::FollowJointTrajectoryGoal& actGoal,int t)
    {
        double eps = 1e-5;

        // This constructor parses the URDF loaded in rosparm urdf_param into the
        // needed KDL structures.  We then pull these out to compare against the KDL
        // IK solver.
        TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param_, timeout_, eps);

        KDL::Chain chain;
        KDL::JntArray ll, ul; //lower joint limits, upper joint limits

        bool valid = tracik_solver.getKDLChain(chain);
        if (!valid) {
          ROS_ERROR("There was no valid KDL chain found");
          return false;
        }

        valid = tracik_solver.getKDLLimits(ll,ul);
        if (!valid) {
          ROS_ERROR("There were no valid KDL joint limits found");
          return false;
        }

        assert(chain.getNrOfJoints() == ll.data.size());
        assert(chain.getNrOfJoints() == ul.data.size());

        // Create Nominal chain configuration midway between all joint limits
        KDL::JntArray nominal(chain.getNrOfJoints());
        for (uint j=0; j<nominal.data.size(); j++) {
          nominal(j) = (ll(j)+ul(j))/2.0;
        }

        //Asign value to KDL pose
        KDL::Frame end_effector_pose;
        end_effector_pose.M.data[0] = Pose(0,0);
        end_effector_pose.M.data[1] = Pose(0,1);
        end_effector_pose.M.data[2] = Pose(0,2);
        end_effector_pose.p.data[0] = Pose(0,3);
        end_effector_pose.M.data[3] = Pose(1,0);
        end_effector_pose.M.data[4] = Pose(1,1);
        end_effector_pose.M.data[5] = Pose(1,2);
        end_effector_pose.p.data[1] = Pose(1,3);
        end_effector_pose.M.data[6] = Pose(2,0);
        end_effector_pose.M.data[7] = Pose(2,1);
        end_effector_pose.M.data[8] = Pose(2,2);
        end_effector_pose.p.data[2] = Pose(2,3);

        //Compute IK
        KDL::JntArray result;
        int rc = tracik_solver.CartToJnt(nominal,end_effector_pose,result);
        if (rc>=0) {
//              ROS_INFO("J0=%f,J1=%f,J2=%f,J3=%f,J4=%f,J5=%f;",
//                                  result.data[0],result.data[1],result.data[2],result.data[3],result.data[4],result.data[5]);

        }
          else {
              ROS_INFO("No valid solution found!!!");
              return false;
          }
        //Define action Goal
        actGoal.trajectory.joint_names=joint_names;
        //Define single point in the trajectory
        trajectory_msgs::JointTrajectoryPoint joint_traj_point;
        for(int i=0;i<6;++i)
        {
            joint_traj_point.positions.push_back((double)result.data[i]);
            joint_traj_point.velocities.push_back(0.0);
            joint_traj_point.accelerations.push_back(0.0);

        }
        joint_traj_point.time_from_start=ros::Duration(t);
        //actGoal.goal_time_tolerance=ros::Duration(5.0);

        actGoal.trajectory.points.push_back(joint_traj_point);
        return true;

    }

    bool performCalibration()
    {
        //Config ensenso to stream only images
        ensenso::ConfigureStreaming conf_stream_srv;
        conf_stream_srv.request.cloud=false;
        conf_stream_srv.request.images=true;
        ros::service::waitForService("configure_streaming");
        config_stream_client.call(conf_stream_srv);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        //Initialize move group
        moveit::planning_interface::MoveGroup group("calibration");
        group.setPoseReferenceFrame("/base");
        group.setMaxVelocityScalingFactor(0.3);
        group.setMaxAccelerationScalingFactor(0.3);
        moveit::planning_interface::MoveGroup::Plan planner;

        //Move UR to "up" pose and get pose msg
        group.setNamedTarget("up_calibration");
        group.move();

        //Generate hemisphere for robot poses generation
        Eigen::Vector3d tool_origin(pos_x,pos_y,pos_z);
        Eigen::Vector3d obj_origin(pos_x,pos_y,pos_z-calTabDistance);

        sphere.header.stamp=ros::Time::now();
        sphere.pose.position.x=obj_origin[0];
        sphere.pose.position.y=obj_origin[1];
        sphere.pose.position.z=obj_origin[2];
        sphere.scale.x=2*(calTabDistance);
        sphere.scale.y=2*(calTabDistance);
        sphere.scale.z=2*(calTabDistance);

        marker_pub.publish(sphere);

        //vector for storing UR poses
        //std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses;
        std::vector<geometry_msgs::Pose> robot_poses;
        //Initialize Calibration (Set grid space & Clear buffer)
        ros::service::waitForService("init_calibration");
        init_cal_srv.request.grid_spacing=grid_space_;
        init_cal_client.call(init_cal_srv);
        if(!init_cal_srv.response.success){
            ROS_ERROR("Initialize calibration fail!");
            return false;
        }

        int fail_count=0;
        //TF listener for cellecting robot poses
        tf::TransformListener listener;
        tf::StampedTransform transform_stamped;
        std_msgs::String status;

        int pose_collected=0;
        int pattern_collected=0;
        int num_pattern=0;


        while(nh.ok() && robot_poses.size()<num_poses_)
        {
            geometry_msgs::Pose wayPoint;

//            if(fail_count>10){
//                //Deal with failure...
//                ROS_ERROR("Too many failures! Abort calibration!");

//                //Go back to initial pose
//                group.setNamedTarget("up");
//                group.move();

//                return false;
//            }

            //get a random point on the hemisphere and plan
            tf::poseEigenToMsg(generateRandomHemispherePose(obj_origin,tool_origin),wayPoint);
            group.setPoseTarget(wayPoint);
            bool success=group.plan(planner);
            if(!success)
            {
                fail_count++;
                std::cout<<"Motion Planning fail: "<<fail_count<<std::endl;
                continue;

            }
            else
            {
                //Move UR to the random point
                group.move();
                //sleep until UR reach the goal point
                sleep(1);

                //Collect patterns
                    //block until service available
                ros::service::waitForService("capture_pattern");
                ensenso::CapturePattern capture_pattern_srv;
                capture_pattern_client.call(capture_pattern_srv);

                if(!capture_pattern_srv.response.success)
                {
                    fail_count++;
                    ROS_ERROR("The %d th pattern captureing fail..",fail_count);
                    std::cout<<"Pattern capture fail: "<<fail_count<<std::endl;
                    continue;
                }
                pattern_collected++;
                cout<<"Pattern collected successfully: "<<pattern_collected<<endl;

                //collect robot pose
                try{
                    ros::Time now(ros::Time::now());
                    listener.waitForTransform("/base",tcp_name_,now,ros::Duration(1.5));
                    listener.lookupTransform("/base",tcp_name_,now,transform_stamped);
                    geometry_msgs::Pose robot_pose;
                    tfToGeometryMsg(transform_stamped,robot_pose);
                    robot_poses.push_back(robot_pose);
                    pose_collected++;
                    cout<<"Pose collected successfully: "<<pose_collected<<endl;
                }
                catch(tf::TransformException& ex){
                    status.data = ex.what();
                    status_pub.publish(status);
                    return false;
                }

                sleep(1);
                num_pattern=capture_pattern_srv.response.pattern_count;
            }


        }

        //Perform calibration
        status.data = "Computing calibration matrix...";
        status_pub.publish(status);

        if(robot_poses.size()!=num_pattern){
            ROS_ERROR("The number of robot poses is not consistent with the counts of pattern. Aborting calibraiton!");
            return false;
        }else{
            std::string result;
            //Initialize srv request
            compute_cal_srv.request.store_to_eeprom=true;
            tf::poseEigenToMsg(Eigen::Affine3d::Identity(),compute_cal_srv.request.seed);
                //Populate the srv poses
            compute_cal_srv.request.robotposes.poses.resize(robot_poses.size());
            compute_cal_srv.request.robotposes.poses.assign(robot_poses.begin(),robot_poses.end());
            //call the srv
            compute_cal_client.call(compute_cal_srv);
        }
        if(compute_cal_srv.response.success){
            ROS_INFO("Calibraiton computation finishes");
            ROS_INFO("Result: ");
            ROS_INFO("Position: x = %f, y = %f, z = %f",compute_cal_srv.response.result.position.x,compute_cal_srv.response.result.position.y,compute_cal_srv.response.result.position.z);
            ROS_INFO("Orientation: w = %f, x = %f, y = %f, z = %f",compute_cal_srv.response.result.orientation.w,compute_cal_srv.response.result.orientation.x,compute_cal_srv.response.result.orientation.y,compute_cal_srv.response.result.orientation.z);

            tf::Vector3 cam_to_tool_position(compute_cal_srv.response.result.position.x,compute_cal_srv.response.result.position.y,compute_cal_srv.response.result.position.z);
            tf::Quaternion cam_to_tool_orientation(compute_cal_srv.response.result.orientation.x,compute_cal_srv.response.result.orientation.y,compute_cal_srv.response.result.orientation.z,compute_cal_srv.response.result.orientation.w);
            get_tool_to_camera_tf(cam_to_tool_position,cam_to_tool_orientation);

        }else{
            ROS_ERROR("Fail to compute extrinsic calibration!");
            return false;
        }


        //save calibration
            //...

        return true;
    }

    void testURcontrol()
    {
        //Action client
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FJT_client(action_server_,true);

        //Go to the default pose
        control_msgs::FollowJointTrajectoryGoal home;
        ur5_trac_ik(chain_start_,chain_end_,default_pose,home,2.0);
        FJT_client.waitForServer();
        FJT_client.sendGoal(home);
        FJT_client.waitForResult();

        Eigen::Vector3d tool_origin(0.5,0.0,0.3);
        Eigen::Vector3d obj_origin(0.5,0.0,0.0);

        int i=0;
        while(i<num_poses_){
            control_msgs::FollowJointTrajectoryGoal goal;
            control_msgs::FollowJointTrajectoryGoal home_goal;
            Eigen::Affine3d wayPoint = generateRandomHemispherePose(obj_origin,tool_origin);
            if(ur5_trac_ik(chain_start_,chain_end_,wayPoint,goal,3.0))
            {
                FJT_client.waitForServer();
                FJT_client.sendGoal(goal);
                if(FJT_client.waitForResult(ros::Duration(5.0)))
                    {
                    ROS_INFO("UR has reach the %d desired position",i+1);
                    std::cout<<i+1<<"position"<<std::endl;
                }else{
                    ROS_INFO("Time out! UR cannot reach the %d desired position",i+1);
                }
                //Go home
                ur5_trac_ik(chain_start_,chain_end_,default_pose,home_goal,2.0);
                FJT_client.waitForServer();
                FJT_client.sendGoal(home_goal);
                FJT_client.waitForResult();
                std::cout<<"home"<<std::endl;

             }
            i++;
        }

    }

    void testURcontrol_withMoveit()
    {
        Eigen::Vector3d tool_origin(0.36216,-0.38496,0.42582);
        Eigen::Vector3d obj_origin(0.36216,-0.38496,0.0);

        //Move group definition
        moveit::planning_interface::MoveGroup group("calibration");
        group.setPoseReferenceFrame("/base");
        moveit::planning_interface::MoveGroup::Plan planner;

        ros::AsyncSpinner spinner(1);
        spinner.start();

        int i=0;
        while(i<num_poses_)
        {
            Eigen::Affine3d wayPoint = generateRandomHemispherePose(obj_origin,tool_origin);
            geometry_msgs::Pose goalPose;
            tf::poseEigenToMsg(wayPoint,goalPose);

            group.setPoseTarget(goalPose);
            bool is_success=group.plan(planner);

            if(is_success)
            {
                cout<<"Planning succeed! Ready to move the robot"<<endl;
                group.move();
                sleep(1);
            }else{
                cout<<"Planning fail"<<endl;
            }

            ++i;
        }
        spinner.stop();
    }

     void testURcontrol_withService()
     {
         //Initialize pose
         ros::AsyncSpinner spinner(1);
         spinner.start();

         //Move group definition
         moveit::planning_interface::MoveGroup group("calibration");
         group.setPoseReferenceFrame("/base");
         group.setPlanningTime(2);

         //Move to up position.
         group.setNamedTarget("up_calibration");
         group.move();
         sleep(2);

         // Initialize trajectory
         moveit_msgs::ExecuteKnownTrajectory srv;
         srv.request.wait_for_execution = true;
         ros::ServiceClient executeKnownTrajectoryServiceClient = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
             "/execute_kinematic_path");
         std::vector<geometry_msgs::Pose> way_points_msg(1);

         //Generate desired pose
         Eigen::Vector3d tool_origin(0.3,0.0,0.5);
         Eigen::Vector3d obj_origin(0.3,0.0,0.0);
         tf::poseEigenToMsg(generateRandomHemispherePose(obj_origin, tool_origin), way_points_msg[0]);

         //Plan a trajectory
         for(int i=0;i<num_poses_;++i)
         {
             double precision=group.computeCartesianPath(way_points_msg, 0.1, 0, srv.request.trajectory);
//             if (precision<0.95)
//            {
//                ROS_WARN_STREAM("Cannot reach pose: skipping to next pose");
//                cout<<"Planning fail"<<endl;
//                continue;
//            }
             for (unsigned i = 0; i < srv.request.trajectory.joint_trajectory.points.size(); ++i)
             {
               for (unsigned j = 0; j < 6; ++j)
                 srv.request.trajectory.joint_trajectory.points[i].velocities.push_back(0.5);
             }
             executeKnownTrajectoryServiceClient.call(srv);
            sleep(2);
         }

    }

    void testCalibration()
    {
        Eigen::Matrix4d m;
        m << 0.0, -1.0, 0.0, 0.217083,
             -1.0, 0.0, 0.0, -0.285758,
             0.0, 0.0, -1.0, 0.0403132,
             0.0, 0.0, 0.0, 1.0;
        Eigen::Affine3d robot_pose_(m);
        geometry_msgs::Pose robot_pose;
        tf::poseEigenToMsg(robot_pose_,robot_pose);
        //Z offset
        robot_pose.position.z+=0.142;

        ros::AsyncSpinner spinner(1);
        spinner.start();
        moveit::planning_interface::MoveGroup group("calibration");
        group.setPoseReferenceFrame("/base");
        group.setMaxVelocityScalingFactor(0.3);
        group.setMaxAccelerationScalingFactor(0.3);
        group.setPoseTarget(robot_pose);

        moveit::planning_interface::MoveGroup::Plan planner;
        if(group.plan(planner))
            {
            group.move();
        }else{
            std::cout<<"Planning fail!"<<std::endl;
        }
        spinner.stop();


    }

public:
    //number of poses for calibration
    int num_poses_;
    //Define kinematics chain for IK solver
    std::string chain_start_;
    std::string chain_end_;
    std::string tcp_name_;
    std::string base_name_;
    //Define joint names
    std::vector<std::string> joint_names;
    //Time duration for IK solver
    double timeout_;
    //URDF varable
    std::string urdf_param_;
    //Name of the action server
    std::string action_server_;
    //Initial calibration position
    double pos_x,pos_y,pos_z;
    //Approximate distance for cal board to camera
    double calTabDistance; //unit: meter
    //Visualization hemisphere for pose sampling
    visualization_msgs::Marker sphere;
};

//Utility to get tf
tf::StampedTransform get_Transform(std::string parent,std::string child)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform(parent,child,ros::Time(0),ros::Duration(1.5));
    listener.lookupTransform(parent,child,ros::Time(0),transform);
    std::cout<<parent<<" to "<<child<<" transfotm: "<<std::endl;
    std::cout<<"Position: "<<"x: "<<transform.getOrigin().getX()<<" y: "<<transform.getOrigin().getY()<<" z: "<<transform.getOrigin().getZ()<<std::endl;
    std::cout<<"Orientation: "<<"x: "<<transform.getRotation().getX()<<" y: "<<transform.getRotation().getY()<<" z: "<<transform.getRotation().getZ()<<" w: "<<transform.getRotation().getW()<<std::endl;

    return transform;
}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"extrin_cal");
    int num_poses=0;
    double ee_x=0.0;
    double ee_y=0.0;
    double ee_z=0.0;
    double cal_dist=0.0;
    double grid_space=0.0;

    //Defalut params
    if(argc < 6)
        {
        num_poses=50;
        ee_x=0.26233;
        ee_y=-0.31279;
        ee_z=0.45148;
        cal_dist=0.42;
        grid_space=12.5;    //Unit: cm
    }
    else{
        num_poses=atoi(argv[1]);
        ee_x=atof(argv[2]);
        ee_y=atof(argv[3]);
        ee_z=atof(argv[4]);
        cal_dist=atof(argv[5]);
        grid_space=atof(argv[6]);    //Unit: cm
    }
    ex_cal calibration(num_poses,ee_x,ee_y,ee_z,cal_dist,grid_space);
    calibration.performCalibration();
    return 0;
}
