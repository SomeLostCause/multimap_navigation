#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sqlite3.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "multimap_navigation/MultimapNavigationActionAction.h"
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <cstdlib> 

// Action server class
class MultimapNavigationActionServer {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<multimap_navigation::MultimapNavigationActionAction> as_;
    std::string action_name_;
    std::string current_map_;
    sqlite3 *db_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

public:
    MultimapNavigationActionServer(const std::string &name)
        : as_(nh_, name, boost::bind(&MultimapNavigationActionServer::executeCB, this, _1), false),
          action_name_(name), move_base_client_("move_base", true) {

        //store the file path to a variable by using find_package /data/multimap.db
        std::string db_path = ros::package::getPath("multimap_navigation") + "/data/multimap.db";

        // Open database
        if (sqlite3_open(db_path, &db_) != SQLITE_OK) {
            ROS_ERROR("Failed to open database!");
        }
        current_map_ = "map1";
        as_.start();
        ROS_INFO("Multimap Navigation Action Server started with action name: %s", action_name_.c_str());
    }

    ~MultimapNavigationActionServer() {
        sqlite3_close(db_);
    }

    void executeCB(const multimap_navigation::MultimapNavigationActionGoalConstPtr &goal) {
        std::string target_map = goal->target_map;
        double target_x = goal->target_x;
        double target_y = goal->target_y;

        if (target_map != current_map_) {
            //save name of current map to a temporary variable
            std::string previous_current_map = current_map_;
            
            if (!navigateToWormhole(target_map)) {
                as_.setAborted();
                return;
            }
            if (!switchMap(target_map)) {
                as_.setAborted();
                return;
            }
            reinitializeAtExit(target_map, previous_current_map);
        }

        if (sendMoveBaseGoal(target_x, target_y)) {
            as_.setSucceeded();
        } else {
            as_.setAborted();
        }
    }

private:

    bool navigateToWormhole(const std::string &target_map) {
        sqlite3_stmt *stmt;
        std::string query = "SELECT entry_x, entry_y, entry_yaw FROM wormholes WHERE source_map = ? AND target_map = ?";
        if (sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            ROS_ERROR("Failed to prepare wormhole query!");
            return false;
        } ROS_INFO("Reached wormhole entry, preparing to switch map.");

        sqlite3_bind_text(stmt, 1, current_map_.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);

        ROS_INFO("current_map = %s", current_map_.c_str());
        ROS_INFO("target_map = %s", target_map.c_str());

        if (sqlite3_step(stmt) == SQLITE_ROW) {
            double entry_x = sqlite3_column_double(stmt, 0);
            double entry_y = sqlite3_column_double(stmt, 1);
            double entry_yaw = sqlite3_column_double(stmt, 2);
            sqlite3_finalize(stmt);

            // Send goal to wormhole entry in the current map
            if (sendMoveBaseGoal(entry_x, entry_y)) {
                ROS_INFO("Reached wormhole entry, preparing to switch map.");
                return true;
            } else {
                ROS_ERROR("Failed to reach wormhole entry.");
                return false;
            }
        } else {
            ROS_ERROR("No wormhole found for map transition!");
            sqlite3_finalize(stmt);
            return false;
        }
    }



bool switchMap(const std::string &target_map) {
    current_map_ = target_map;

    // Get the path to the target map
    std::string map_file_path = ros::package::getPath("multimap_navigation") + "/maps/" + target_map + ".yaml";
    
    // Set the map file path as a parameter for the map_server node
    nh_.setParam("map_server/map_file", map_file_path);



    // Kill the existing map_server node (if it's running)
    ROS_INFO("Killing the map_server node...");
    if (system("rosnode kill /map_server") == 0) {
        ROS_INFO("Successfully killed the previous map_server node.");
    } else {
        ROS_WARN("Failed to kill the previous map_server node. It might not be running.");
    }
    

    ros::Duration(2.0).sleep();


    // Start a new map_server with the new map
    // std::string command = "rosrun map_server map_server map_file:=" + map_file_path;
    std::string command = "rosrun map_server map_server " + map_file_path + " &";
    

    if (system(command.c_str()) == 0) {
        ros::Duration(2.0).sleep();
        ROS_INFO("Successfully restarted map_server with map: %s", target_map.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to start map_server with map: %s", target_map.c_str());
        return false;
    }
}




    bool sendMoveBaseGoal(double x, double y) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = 1.0;

        move_base_client_.sendGoal(goal);
        move_base_client_.waitForResult();
        return move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    bool reinitializeAtExit(const std::string &target_map,const std::string &temp_current_map) {
        sqlite3_stmt *stmt;
        std::string query = "SELECT exit_x, exit_y, exit_yaw FROM wormholes WHERE source_map = ? AND target_map = ?";
        //print values of source_map and target_map
        ROS_INFO("current_map = %s", temp_current_map.c_str());
        ROS_INFO("target_map = %s", target_map.c_str());

        if (sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            ROS_ERROR("Failed to prepare exit position query!");
            return false;
        }

        sqlite3_bind_text(stmt, 1, temp_current_map.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);

        if (sqlite3_step(stmt) == SQLITE_ROW) {
            double exit_x = sqlite3_column_double(stmt, 0);
            double exit_y = sqlite3_column_double(stmt, 1);
            double exit_yaw = sqlite3_column_double(stmt, 2);
            sqlite3_finalize(stmt);



// Create the initial pose message
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time::now();    // Clear the costmaps to ensure fresh navigation in the new map
        ros::ServiceClient clear_costmaps_client = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
        std_srvs::Empty clear_srv;


        clear_costmaps_client.call(clear_srv);
        initial_pose.header.frame_id = "map";  // Make sure the frame matches your map frame

        initial_pose.pose.pose.position.x = exit_x;
        initial_pose.pose.pose.position.y = exit_y;
        initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(exit_yaw);

        // Print the exit coordinates for debugging
        ROS_INFO("Exit coordinates: x=%f, y=%f, yaw=%f", exit_x, exit_y, exit_yaw);

        // Publish the initial pose multiple times to ensure AMCL receives it
        ros::Publisher initial_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
        for (int i = 0; i < 5; ++i) {
            initial_pose_pub.publish(initial_pose);
            ros::Duration(0.5).sleep();  // Small delay to ensure consistent updates
        }

            ROS_INFO("Robot reinitialized at wormhole exit.");
            return true;
        } else {
            ROS_ERROR("Failed to find exit position in target map.");
            sqlite3_finalize(stmt);
            return false;
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "multimap_navigation_action_server");
    MultimapNavigationActionServer server("multimap_navigation");
    ros::spin();
    return 0;
}
