#include <manip_project/grabObject.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_kdl.h>
#include <assert.h>
#include <boost/bind.hpp>

#define NUM_STABLE_FRAMES 5

using namespace std;
GrabObject::GrabObject():r_arm("right"), l_arm("left"){
    r_arm.setReferenceFrame("/torso_lift_link");
    l_arm.setReferenceFrame("/torso_lift_link");
    m_point_head_client = new PointHeadClient("/head_traj_controller/point_head_action", true);
    client = nh.serviceClient<pr2_mechanism_msgs::SwitchController>("/pr2_controller_manager/switch_controller");
    while(!m_point_head_client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the point_head_action server to come up");
    }
}

void GrabObject::lookAt(std::string frame_id, double x, double y, double z)
{
    //the goal message we will be sending
    control_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "head_mount_kinect_rgb_optical_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    m_point_head_client->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    m_point_head_client->waitForResult(ros::Duration(2));
}

void GrabObject::waitAndGrab(){
    switchToNormalControl();
    vector<double> init_l_arm;
    init_l_arm.push_back(0.8202582499433417);
    init_l_arm.push_back(0.8174119480040183);
    init_l_arm.push_back(1.175226343713942);
    init_l_arm.push_back(-0.9897705605674373);
    init_l_arm.push_back(-4.586757274289091);
    init_l_arm.push_back(-1.2633349113604524);
    init_l_arm.push_back(48.100199487910714);

    vector<double> init_r_arm;
    init_r_arm.push_back(0.019612841720892282);
    init_r_arm.push_back(1.0200406346429765);
    init_r_arm.push_back(0.3279690649941631);
    init_r_arm.push_back(-2.119561192204488);
    init_r_arm.push_back(-97.32515500755363);
    init_r_arm.push_back(-2.007254606158681);
    init_r_arm.push_back(-58.47777662854528);
    r_arm.sendArmToConfiguration(&init_r_arm[0], 3);
    l_arm.sendArmToConfiguration(&init_l_arm[0], 3);
    sleep(3);


    r_arm.openGripper();
    l_arm.openGripper();
    ROS_INFO("waiting for the object to stabilize");
    bool keep_waiting = true;
    ros::Rate rate(2);
    tf::StampedTransform ar_tf;
    while (keep_waiting){
        listener.waitForTransform("/torso_lift_link", 
                                  "/ar_marker_3", 
                                  ros::Time(0), 
                                  ros::Duration(1));
        listener.lookupTransform("/torso_lift_link",
                                  "/ar_marker_3", 
                                  ros::Time(0), 
                                  ar_tf);
        keep_waiting = !(isObjectStable(ar_tf));
        rate.sleep();
    }
    KDL::Frame obj;
    tf::transformTFToKDL(ar_tf, obj);
    ROS_INFO("object is deemed stable at %f %f %f!",
              obj.p.x(),
              obj.p.y(),
              obj.p.z());
    KDL::Rotation flat_gripper = KDL::Rotation::RPY(0,0,0);
    obj.M = flat_gripper;
    KDL::Frame obj_wrist = l_arm.transformToolFrameToWristFrame(obj);
    obj_wrist.M = flat_gripper;
    obj_wrist.p.x(obj_wrist.p.x()+.03);
    KDL::Frame pregrasp_wrist = obj_wrist;
    // make it come up slightly short at first
    pregrasp_wrist.p.x(obj_wrist.p.x()-.10);
    ROS_INFO("pregrasp pose is %f %f %f",
             pregrasp_wrist.p.x(),
             pregrasp_wrist.p.y(),
             pregrasp_wrist.p.z());
    ROS_INFO("grasp pose is %f %f %f",
             obj_wrist.p.x(),
             obj_wrist.p.y(),
             obj_wrist.p.z());

    vector<double> move_times(2,3); 
    vector<KDL::Frame> poses;
    poses.push_back(pregrasp_wrist);
    poses.push_back(obj_wrist);
    ROS_INFO("sending poses to move arm");
    bool move_arm_success = l_arm.sendArmToPoses(poses, move_times);
    if (!move_arm_success){
        ROS_ERROR("movearm failed! dying.");
        exit(1);
    }
    sleep(6.1);
    ROS_INFO("closing gripper");
    l_arm.moveGripper(0,100);
}

// computes the average change in position of the object over a few frames
bool GrabObject::isObjectStable(tf::StampedTransform tf){
    KDL::Frame incoming_frame;
    tf::transformTFToKDL(tf, incoming_frame);
    frames.push_back(incoming_frame);
    if (frames.size() < NUM_STABLE_FRAMES+1){
        ROS_INFO("not enough frames to decide if object is stable");
        return false;
    } else {
        frames.pop_front();
        assert(frames.size() == NUM_STABLE_FRAMES);
    }
    int stable_count = 0;
    double dist = 0;
    for (size_t i=0; i < frames.size()-1; i++){
        double del_x = frames[i+1].p.x()-frames[i].p.x();
        double del_y = frames[i+1].p.y()-frames[i].p.y();
        double del_z = frames[i+1].p.z()-frames[i].p.z();
        dist += pow(del_x*del_x + del_y*del_y + del_z*del_z,.5);
        if (dist < .04){
            stable_count++;
        } else {
            ROS_INFO("detected movement");
            stable_count = 0;
        }
    }
    if (stable_count == NUM_STABLE_FRAMES-1)
        return true;
    else
        return false;
}


void GrabObject::grabTop(double& x, double& y, double& z, KDL::Frame& grasp){
    switchToNormalControl();
   // - Translation: [0.720, 0.176, -0.033]
   // - Rotation: in Quaternion [0.152, -0.026, -0.658, 0.737]
    //- Translation: [0.601, 0.261, -0.003]
    //- Rotation: in Quaternion [-0.667, 0.700, -0.163, 0.198]
    r_arm.openGripper();
    KDL::Vector v(0.637315,0.379170,-0.196212);
    KDL::Rotation rot = KDL::Rotation::Quaternion(0.280329,-0.243020,-0.493952,0.786364);
    KDL::Frame view_top_frame(rot, v);


    ROS_INFO("repositioning bottle to view top");
    l_arm.sendArmToPose(view_top_frame, 2);
    sleep(2.2);
    ROS_INFO("looking at bottle");
    lookAt("l_gripper_tool_frame", 0, 0, 0);
    sleep(2);

    // in l_gripper_tool_frame
    vision.regrabBottleTop(); // makes sure we get a fresh frame
    vision.computeBottleGrab(x, y, z);
    x -=.02;
    z -= .02;
    ROS_INFO("computed grab point is %f %f %f", x, y, z);
    pviz.setReferenceFrame("l_gripper_tool_frame");
    for (int i=0; i < 3; i++){
        pviz.visualizeSphere(x, y, z, .02, 150, "bottle_grab", 1);
        //sleep(1);
    }
    
    KDL::Frame grasp_pose(KDL::Rotation::RPY(0,M_PI,0), KDL::Vector(x, y, z));
    geometry_msgs::Pose grasp_pose_geo;
    tf::poseKDLToMsg(grasp_pose, grasp_pose_geo);
    geometry_msgs::PoseStamped grasp_pose_geo_stamped;
    grasp_pose_geo_stamped.pose = grasp_pose_geo;
    grasp_pose_geo_stamped.header.frame_id = "/l_gripper_tool_frame";
    grasp_pose_geo_stamped.header.stamp = ros::Time(0);
    listener.waitForTransform("/torso_lift_link", "/l_gripper_tool_frame",
                         ros::Time(0), ros::Duration(10.0));
    geometry_msgs::PoseStamped grasp_pose_tool_frame;
    listener.transformPose("/torso_lift_link", grasp_pose_geo_stamped, grasp_pose_tool_frame);
    KDL::Frame grasp_tool_frame;
    tf::poseMsgToKDL(grasp_pose_tool_frame.pose, grasp_tool_frame);
    KDL::Frame r_grasp_wrist = r_arm.transformToolFrameToWristFrame(grasp_tool_frame);
    

    //KDL::Frame pregrasp_wrist = r_arm.transformToolFrameToWristFrame(pregrasp);

    // move l arm down
    KDL::Vector v2(0.602, 0.132, -0.313);
    KDL::Rotation rot2 = KDL::Rotation::Quaternion(0.054, -0.070, -0.704, 0.704);
    KDL::Frame lowered_pos(rot2, v2);
    //l_arm.sendArmToPose(lowered_pos, 2);

    KDL::Frame pregrasp(KDL::Rotation::RPY(0,M_PI,0), KDL::Vector(x, y, z+.2));
    geometry_msgs::Pose pregrasp_pose;
    tf::poseKDLToMsg(pregrasp, pregrasp_pose);
    geometry_msgs::PoseStamped pregrasp_geo;
    pregrasp_geo.pose = pregrasp_pose;
    pregrasp_geo.header.frame_id = "/l_gripper_tool_frame";
    pregrasp_geo.header.stamp = ros::Time(0);

    //{
    //    tf::TransformBroadcaster br;
    //    tf::Transform r_wrist_tf;
    //    tf::transformKDLToTF(pregrasp, r_wrist_tf);
    //    for (int i=0; i < 100; i++){
    //        br.sendTransform(tf::StampedTransform(r_wrist_tf, ros::Time::now(), 
    //                         "/l_gripper_tool_frame", "pregrasp"));
    //        sleep(1);
    //    }
    //}

    geometry_msgs::PoseStamped pregrasp_tool_frame;
    listener.waitForTransform("/torso_lift_link", "/l_gripper_tool_frame",
                         ros::Time(0), ros::Duration(10.0));
    listener.transformPose("/torso_lift_link", pregrasp_geo, pregrasp_tool_frame);
    KDL::Frame r_tool_frame;
    tf::poseMsgToKDL(pregrasp_tool_frame.pose, r_tool_frame);
    KDL::Frame r_wrist_frame = r_arm.transformToolFrameToWristFrame(r_tool_frame);

    //{
    //    tf::TransformBroadcaster br;
    //    tf::Transform r_wrist_tf;
    //    tf::transformKDLToTF(r_wrist_frame, r_wrist_tf);
    //    for (int i=0; i < 10; i++){
    //        br.sendTransform(tf::StampedTransform(r_wrist_tf, ros::Time::now(), 
    //                         "/torso_lift_link", "pregrasp"));
    //        //sleep(1);
    //    }
    //}

    KDL::Frame final_grasp(r_wrist_frame.M, KDL::Vector(x, y, z));
    std::vector<KDL::Frame> frames;
    frames.push_back(r_wrist_frame);
    frames.push_back(r_grasp_wrist);
    std::vector<double> move_times;
    move_times.push_back(2);
    move_times.push_back(3);
    //r_arm.sendArmToPoses(frames, move_times);
    ROS_INFO("sending wrist pose %f %f %f",
             r_grasp_wrist.p.x(),
             r_grasp_wrist.p.y(),
             r_grasp_wrist.p.z());

    //r_arm.sendArmToPose(r_wrist_frame, 3);
    //sleep(3.5);
    r_arm.sendArmToPose(r_grasp_wrist, 3);
    grasp = r_grasp_wrist;
    //{
    //    tf::TransformBroadcaster br;
    //    tf::Transform r_tool_frame_tf;
    //    tf::transformKDLToTF(grasp_tool_frame, r_tool_frame_tf);
    //    tf::Transform r_wrist_tf;
    //    tf::transformKDLToTF(r_grasp_wrist, r_wrist_tf);
    //    for (int i=0; i < 100; i++){
    //        br.sendTransform(tf::StampedTransform(r_wrist_tf, ros::Time::now(), 
    //                         "/torso_lift_link", "wrist_grasp"));
    //        br.sendTransform(tf::StampedTransform(r_tool_frame_tf, ros::Time::now(), 
    //                         "/torso_lift_link", "tool_frame_grasp"));
    //        sleep(1);
    //    }
    //}
    sleep(3.1);
    r_arm.moveGripper(0,100);
    sleep(2);
}

void GrabObject::regrasp(KDL::Frame& grasp){
    switchToNormalControl();
    r_arm.openGripper();
    sleep(2);
    r_arm.sendArmToPose(grasp, 1);
    sleep(1);
    r_arm.moveGripper(0,100);
    sleep(2);
}

void GrabObject::switchToImpedControl(){
    ROS_INFO("switching to imped control");
    pr2_mechanism_msgs::SwitchController srv_call;
    vector<string> starters;
    vector<string> stoppers;
    starters.push_back("r_arm_cart_imped_controller");
    starters.push_back("l_arm_cart_imped_controller");
    stoppers.push_back("r_arm_controller");
    stoppers.push_back("l_arm_controller");
    srv_call.request.start_controllers = starters;
    srv_call.request.stop_controllers = stoppers;
    srv_call.request.strictness = 2;
    if (client.call(srv_call)){
        if (srv_call.response.ok){
            ROS_INFO("switch completed");
        } else {
            ROS_INFO("switch failed");
        }
    } else { 
        ROS_INFO("couldn't contact srv");
    }
}

void GrabObject::switchToNormalControl(){
    ROS_INFO("switching to normal control");
    pr2_mechanism_msgs::SwitchController srv_call;
    vector<string> starters;
    vector<string> stoppers;
    starters.push_back("r_arm_controller");
    starters.push_back("l_arm_controller");
    stoppers.push_back("r_arm_cart_imped_controller");
    stoppers.push_back("l_arm_cart_imped_controller");
    srv_call.request.start_controllers = starters;
    srv_call.request.stop_controllers = stoppers;
    srv_call.request.strictness = 2;
    if (client.call(srv_call)){
        if (srv_call.response.ok){
            ROS_INFO("switch completed");
        } else {
            ROS_INFO("switch failed");
        }
    } else { 
        ROS_INFO("couldn't contact srv");
    }
}

void GrabObject::twistOff(double x, double y, double z){
    switchToImpedControl();

    double qx, qy, qz, qw;
    KDL::Rotation rot = KDL::Rotation::RPY(M_PI/8,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm arm("/r_arm_cart_imped_controller");
    ee_cart_imped_msgs::EECartImpedGoal traj;
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            .75, "/l_gripper_tool_frame");

    rot = KDL::Rotation::RPY(-M_PI/8,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            1.5, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");

    rot = KDL::Rotation::RPY(M_PI/8,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            2.25, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");
    rot = KDL::Rotation::RPY(-M_PI/8,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            3, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");
    rot = KDL::Rotation::RPY(M_PI/6,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            3.75, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");
    rot = KDL::Rotation::RPY(-M_PI/6,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            4.5, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");
    ROS_INFO("starting twistoff");
    rot = KDL::Rotation::RPY(M_PI/6,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            5.25, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");
    rot = KDL::Rotation::RPY(-M_PI/6,M_PI,0);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, x,y,z+.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            6, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");
    arm.startTrajectory(traj, true);
}


void GrabObject::twistBottle(double x, double y, double z){
    switchToImpedControl();



    EECartImpedArm r_arm("/r_arm_cart_imped_controller");
    KDL::Rotation rot = KDL::Rotation::RPY(0,M_PI,0);
    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);
    ee_cart_imped_msgs::EECartImpedGoal r_traj;
    EECartImpedArm::addTrajectoryPoint(r_traj, x,y,z, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            10, "/l_gripper_tool_frame");
    r_arm.startTrajectory(r_traj, false);

    rot = KDL::Rotation::RPY(0,0,M_PI/4);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm arm("/l_arm_cart_imped_controller");
    ee_cart_imped_msgs::EECartImpedGoal traj;
    EECartImpedArm::addTrajectoryPoint(traj, 0,0,-.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            2, "/l_gripper_tool_frame");

    rot = KDL::Rotation::RPY(0,0,-M_PI/4);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, 0,0,-.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            4, "/l_gripper_tool_frame");
    ROS_INFO("starting twistoff");

    rot = KDL::Rotation::RPY(0,0,M_PI/4);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, 0,0,-.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            6, "/l_gripper_tool_frame");

    rot = KDL::Rotation::RPY(0,0,-M_PI/4);
    rot.GetQuaternion(qx, qy, qz, qw);
    EECartImpedArm::addTrajectoryPoint(traj, 0,0,-.2, qx, qy, qz, qw,
            1000, 1000, 1000, 75, 75, 75,
            false, false, false, 
            false, false, false, 
            8, "/l_gripper_tool_frame");
    arm.startTrajectory(traj, true);
}

VisionStuff::VisionStuff():received_top(false){
    m_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/bottle", 3);
    m_bottle_top_pub = nh.advertise<sensor_msgs::PointCloud2>("/bottle_top",1);

    std::string topic("/camera/rgb/object_modeling_points_filtered");
    m_cloud_sub.subscribe(nh, topic, 1);

    m_tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(
                    m_cloud_sub, listener, "/l_gripper_tool_frame", 1);
    m_tf_filter->registerCallback(boost::bind(&VisionStuff::storeBottleTop, this, _1));

}

void VisionStuff::visualizePC(std::string frame_id, const PointCloud& pc){
    sensor_msgs::PointCloud2 msg;
    toROSMsg(pc, msg);
    msg.header.frame_id = frame_id;
    for (int i=0; i < 10; i++){
        m_pc_pub.publish(msg);
        sleep(1);
    }
}

void VisionStuff::storeBottleTop(const sensor_msgs::PointCloud2ConstPtr& cloud){
    sensor_msgs::PointCloud2 tf_cloud;
    pcl_ros::transformPointCloud("/l_gripper_tool_frame", *cloud, tf_cloud, listener);
    PointCloud bottle;
    fromROSMsg(tf_cloud, bottle);
    ROS_INFO("storing botle msg of size %lu in frame %s", 
            bottle.points.size(),
            bottle.header.frame_id.c_str());
    PointCloud::Ptr bottle_top = boundPointCloud(bottle);
    if (bottle_top->size()){
        received_top = true;
        m_bottle_top = *bottle_top;
    } else {
        ROS_WARN("got point cloud, but couldn't find bottletop. trying again...");
    }
}

PointCloud VisionStuff::getBottleTop(){
    while (!received_top){
        ros::spinOnce();
    }
    //visualizePC("/l_gripper_tool_frame", m_bottle_top);
    return m_bottle_top;
}

PointCloud::Ptr VisionStuff::boundPointCloud(const PointCloud& pc_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xchopped (new PointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_chopped(new PointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_zchopped(new PointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr bottle_top(new PointCloud);

    double halfheight = .35;
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(pc_msg.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0-halfheight, 0+halfheight);
    pass.filter(*cloud_zchopped);

    pass.setInputCloud(cloud_zchopped);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-.1, .1);
    pass.filter(*cloud_xchopped);

    pass.setInputCloud(cloud_xchopped);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-.1, .1);
    pass.filter(*cloud_all_chopped);

    double max_z = 0;
    for (size_t i=0; i < cloud_all_chopped->points.size(); i++){
        if (cloud_all_chopped->points[i].z > max_z){
            max_z = cloud_all_chopped->points[i].z;
        }
    }

    pass.setInputCloud(cloud_all_chopped);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(max_z-.08, max_z);
    pass.filter(*bottle_top);
    if (!bottle_top->points.size()){
        ROS_ERROR("Couldn't find the bottle top!!!!");
    }
    ROS_INFO("bottle top size %lu", bottle_top->points.size());
    return bottle_top;
}

void VisionStuff::computeBottleGrab(double& x, double& y, double& z){
    PointCloud cloud = getBottleTop();
    ROS_INFO("computing top location from cloud of size %lu", cloud.size());
    double avgx = 0;
    double avgy = 0;
    double avgz = 0;
    for (size_t i=0; i < cloud.size(); i++){
        avgx += cloud.points[i].x;
        avgy += cloud.points[i].y;
        avgz += cloud.points[i].z;
    }
    avgx /= cloud.points.size();
    avgy /= cloud.points.size();
    avgz /= cloud.points.size();

    x = avgx;
    y = avgy;
    z = avgz;
}

bool VisionStuff::findTop(const PointCloud& bottle, PointCloud& bottle_top){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    Eigen::Vector3f search_axis;
    search_axis << 0,0,1;
    seg.setAxis(search_axis);
    seg.setEpsAngle(.10);

    seg.setInputCloud (bottle.makeShared ());
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_ERROR("Could not estimate a planar model for the given dataset.");
        return false;
    }


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (bottle.makeShared());
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (bottle_top);

    sensor_msgs::PointCloud2 bottle_top_msg;
    pcl::toROSMsg(bottle_top, bottle_top_msg);
    m_pc_pub.publish(bottle_top_msg);

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "grabber");
    GrabObject go;
    go.waitAndGrab();
    double x, y, z;
    KDL::Frame grasp;
    go.grabTop(x,y,z, grasp);
    go.twistBottle(x,y,z);
    //go.twistOff(x,y,z);

}
