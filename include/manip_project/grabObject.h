#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <pr2_arm_utils/arm.h>
#include <deque>
#include <pcl/point_types.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>
#include <pviz/pviz.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
class VisionStuff {
    public:
        VisionStuff();
        PointCloud::Ptr boundPointCloud(const PointCloud& pc_msg);
        bool findTop(const PointCloud& cloud, PointCloud& bottle_top);
        void computeBottleGrab(double& x, double& y, double& z);
        void storeBottleTop(const sensor_msgs::PointCloud2ConstPtr& cloud);
        PointCloud getBottleTop();
        void regrabBottleTop(){ received_top = false; };
    private:
        void visualizePC(std::string frame_id, const PointCloud& pc);
        message_filters::Subscriber<sensor_msgs::PointCloud2> m_cloud_sub;
        tf::MessageFilter<sensor_msgs::PointCloud2>*  m_tf_filter;
        tf::TransformListener listener;
        PointCloud m_bottle_top;
        bool received_top;
        ros::Publisher m_pc_pub;
        ros::Publisher m_bottle_top_pub;
        ros::NodeHandle nh;
};

class GrabObject {
    public:
        GrabObject();
        void waitAndGrab();
        void grabTop(double& x, double& y, double& z, KDL::Frame& grasp);
        void lookAt(std::string frame_id, double x, double y, double z);
        void twistOff(double x, double y, double z);
        void twistBottle(double x, double y, double z);
        void regrasp(KDL::Frame& grasp);
    private:
        void switchToImpedControl();
        void switchToNormalControl();
        ros::NodeHandle nh;
        PointHeadClient* m_point_head_client;
        Arm r_arm;
        Arm l_arm;
        void waitForStableObject();
        bool isObjectStable(tf::StampedTransform tf);
        tf::TransformListener listener;
        std::deque<KDL::Frame> frames;
        VisionStuff vision;
        ros::ServiceClient client;
        PViz pviz;
};

