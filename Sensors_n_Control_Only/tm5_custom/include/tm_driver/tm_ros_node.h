
#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_ethernet_slave_connect.h"
#include "tm_listen_node_connect.h"
#include <memory>

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//#include <visualization_msgs/InteractiveMarkerUpdate.h>

//#include <boost/chrono/chrono.hpp>
//#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/condition_variable.hpp>

#include "tm_driver/tm_pose_conversion.h"

#include "tm5_custom/FeedbackState.h"
#include "tm5_custom/SvrResponse.h"
#include "tm5_custom/SctResponse.h"
#include "tm5_custom/StaResponse.h"
#include "tm5_custom/ConnectTM.h"
#include "tm5_custom/WriteItem.h"
#include "tm5_custom/AskItem.h"
#include "tm5_custom/SendScript.h"
#include "tm5_custom/SetEvent.h"
#include "tm5_custom/SetIO.h"
//#include "tm5_custom/SetPayload"
#include "tm5_custom/SetPositions.h"
#include "tm5_custom/AskSta.h"



class TmRosNode {
protected:
    std::condition_variable svr_cv_;
    std::condition_variable sct_cv_;
    TmDriver iface_;

    ros::NodeHandle nh_;

    ////////////////////////////////
    // Action
    ////////////////////////////////

    // joint_trajectory_action
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh_;
    control_msgs::FollowJointTrajectoryResult result_;
    bool has_goal_;

    ////////////////////////////////
    // Param.
    ////////////////////////////////

    std::vector<std::string> joint_names_;
    std::string base_frame_name_;
    std::string tool_frame_name_;

    ////////////////////////////////
    // Topic
    ////////////////////////////////

    struct PubMsg {
        ros::Publisher fbs_pub;
        ros::Publisher joint_pub;
        ros::Publisher tool_pose_pub;
        ros::Publisher svr_pub;

        tm5_custom::FeedbackState fbs_msg;
        sensor_msgs::JointState joint_msg;
        geometry_msgs::PoseStamped tool_pose_msg;

        //tf::Transform transform;
        tf::TransformBroadcaster tfbc;

        tm5_custom::SvrResponse svr_msg;
    } pm_;

    struct SctAndStaMsg {
        ros::Publisher sct_pub;
        ros::Publisher sta_pub;

        tm5_custom::SctResponse sct_msg;
        tm5_custom::StaResponse sta_msg;
    } sm_;

    int diconnectTimes = 0;
    uint64_t initialNotConnectTime = 0;
    uint64_t notConnectTimeInS = 0;
    int maxTrialTimeInMinute = -1;
    uint64_t maxNotConnectTimeInS = 0;
    int publishTimeMs = 15;
    bool connect_recovery_is_halt = false;
    bool svr_updated_;
    boost::mutex svr_mtx_;
    boost::condition_variable svr_cond_;

    int pub_reconnect_timeout_ms_;
    int pub_reconnect_timeval_ms_;
    boost::thread pub_thread_;
    std::thread getDataThread;
    std::thread pubDataThread;

    boost::condition_variable sta_cond_;

    boost::thread sct_thread_;

    ////////////////////////////////
    // Service for connection
    ////////////////////////////////

    ros::ServiceServer connect_srv_;

    ////////////////////////////////
    // Service
    ////////////////////////////////

    ros::ServiceServer write_item_srv_;
    ros::ServiceServer ask_item_srv_;

    ros::ServiceServer send_script_srv_;

    ros::ServiceServer set_event_srv_;
    ros::ServiceServer set_io_srv_;

    ros::ServiceServer set_positions_srv_;

    ros::ServiceServer ask_sta_srv_;
    bool isRun = true;

    std::shared_ptr<ListenNodeConnection> listenNodeConnection;
    std::shared_ptr<EthernetSlaveConnection> ethernetSlaveConnection;

    ////////////////////////////////
    // Init.
    ////////////////////////////////
public:
    TmRosNode(const std::string &host);
    ~TmRosNode();
    void halt();

private:
    ////////////////////////////////
    // Action
    ////////////////////////////////

    // helper function

    bool has_points(const trajectory_msgs::JointTrajectory &traj);
    bool has_limited_velocities(const trajectory_msgs::JointTrajectory &traj);
    bool is_traj_finite(const trajectory_msgs::JointTrajectory &traj);
    void reorder_traj_joints(trajectory_msgs::JointTrajectory &traj);
    bool is_start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps);
    void set_result(int32_t err_code, const std::string &err_str);
    //void print_traj(const trajectory_msgs::JointTrajectory &traj);

    void set_pvt_traj(TmPvtTraj &pvts, const trajectory_msgs::JointTrajectory &traj);
    void traj_action(TmPvtTraj pvts);

    // action function

    void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
    void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);

    ////////////////////////////////
    // Topic
    ////////////////////////////////

    void publish_fbs();
    void publish_svr();

    void publisher();
    void pub_data();
    void svr_connect_recover();

    void sct_msg(TmSctData data);
    void sta_msg(std::string subcmd, std::string subdata);

    ////////////////////////////////
    // Service
    ////////////////////////////////

    bool connect_tm(tm5_custom::ConnectTMRequest &req, tm5_custom::ConnectTMResponse &res);

    bool write_item(tm5_custom::WriteItemRequest &req, tm5_custom::WriteItemResponse &res);
    bool ask_item(tm5_custom::AskItemRequest &req, tm5_custom::AskItemResponse &res);

    bool send_script(tm5_custom::SendScriptRequest &req, tm5_custom::SendScriptResponse &res);

    bool set_event(tm5_custom::SetEventRequest &req, tm5_custom::SetEventResponse &res);
    bool set_io(tm5_custom::SetIORequest &req, tm5_custom::SetIOResponse &res);

    bool set_positions(tm5_custom::SetPositionsRequest &req, tm5_custom::SetPositionsResponse &res);

    bool ask_sta(tm5_custom::AskStaRequest &req, tm5_custom::AskStaResponse &res);
};
