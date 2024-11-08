#ifndef MULTIMAP_NAVIGATION_ACTION_SERVER_H
#define MULTIMAP_NAVIGATION_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <multimap_navigation/MultimapNavigationActionAction.h>  // Adjust if this is not the correct header

class MultimapNavigationActionServer {
public:
    MultimapNavigationActionServer(const std::string& name);

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<multimap_navigation::MultimapNavigationAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    std::string action_name_;

    // Callback for action server
    void executeCB(const multimap_navigation::MultimapNavigationActionGoalConstPtr &goal);

    // Helper methods
    bool sendMoveBaseGoal(double x, double y);  // Update signature if yaw is needed
};

#endif
