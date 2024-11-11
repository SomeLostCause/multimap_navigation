#include "multimap_navigation/MultimapNavigationActionAction.h"
#include "multimap_navigation/MultiMapNavigationActionServer.h"
#include <ros/package.h>
#include <cstdlib>

MultimapNavigationActionServer::MultimapNavigationActionServer(const std::string& name)
    : m_as(m_nh, name, boost::bind(&MultimapNavigationActionServer::executeCB, this, _1), false)
    , m_moveBaseClient("move_base", true)
    , m_actionName(name)
    , m_currentMap("map1") {

    m_dbPath = ros::package::getPath("multimap_navigation") + "/data/multimap.db";
    m_mapPath = ros::package::getPath("multimap_navigation") + "/maps/";

    if (sqlite3_open(m_dbPath.c_str(), &m_pDb) != SQLITE_OK) {
        ROS_ERROR("Failed to open database!");
    }

    // Initialize ROS publishers and service clients
    m_initialPosePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    m_clearCostmapsClient = m_nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

    m_as.start();
    ROS_INFO("Multimap Navigation Action Server started with action name: %s", m_actionName.c_str());
}

MultimapNavigationActionServer::~MultimapNavigationActionServer() {
    sqlite3_close(m_pDb);
}

void MultimapNavigationActionServer::executeCB(const multimap_navigation::MultimapNavigationActionGoalConstPtr& goal) {
    std::string targetMap = goal->target_map;
    double targetX = goal->target_x;
    double targetY = goal->target_y;

    if (targetMap != m_currentMap) {
        std::string previousCurrentMap = m_currentMap;
        
        if (!navigateToWormhole(targetMap)) {
            m_as.setAborted();
            return;
        }
        if (!switchMap(targetMap)) {
            m_as.setAborted();
            return;
        }
        reinitializeAtExit(targetMap, previousCurrentMap);
    }

    if (sendMoveBaseGoal(targetX, targetY)) {
        m_as.setSucceeded();
    } else {
        m_as.setAborted();
    }
}

bool MultimapNavigationActionServer::navigateToWormhole(const std::string& target_map) {
    sqlite3_stmt* stmt;
    std::string query = "SELECT entry_x, entry_y, entry_yaw FROM wormholes WHERE source_map = ? AND target_map = ?";
    
    if (sqlite3_prepare_v2(m_pDb, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare wormhole query!");
        return false;
    }

    sqlite3_bind_text(stmt, 1, m_currentMap.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);

    ROS_INFO("current_map = %s", m_currentMap.c_str());
    ROS_INFO("target_map = %s", target_map.c_str());

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        double entryX = sqlite3_column_double(stmt, 0);
        double entryY = sqlite3_column_double(stmt, 1);
        double entryYaw = sqlite3_column_double(stmt, 2);
        sqlite3_finalize(stmt);

        if (sendMoveBaseGoal(entryX, entryY)) {
            ROS_INFO("Reached wormhole entry, preparing to switch map.");
            return true;
        } else {
            ROS_ERROR("Failed to reach wormhole entry.");
            return false;
        }
    }
    
    ROS_ERROR("No wormhole found for map transition!");
    sqlite3_finalize(stmt);
    return false;
}

bool MultimapNavigationActionServer::switchMap(const std::string& target_map) {
    m_currentMap = target_map;
    std::string mapFilePath = m_mapPath + target_map + ".yaml";
    m_nh.setParam("map_server/map_file", mapFilePath);

    ROS_INFO("Killing the map_server node...");
    if (system("rosnode kill /map_server") == 0) {
        ROS_INFO("Successfully killed the previous map_server node.");
    } else {
        ROS_WARN("Failed to kill the previous map_server node. It might not be running.");
    }

    ros::Duration(m_mapSwitchDelay).sleep();

    std::string command = "rosrun map_server map_server " + mapFilePath + " &";
    
    if (system(command.c_str()) == 0) {
        ros::Duration(m_mapSwitchDelay).sleep();
        ROS_INFO("Successfully restarted map_server with map: %s", target_map.c_str());
        return true;
    }
    
    ROS_ERROR("Failed to start map_server with map: %s", target_map.c_str());
    return false;
}

bool MultimapNavigationActionServer::sendMoveBaseGoal(double x, double y) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;

    m_moveBaseClient.sendGoal(goal);
    m_moveBaseClient.waitForResult();
    return m_moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool MultimapNavigationActionServer::reinitializeAtExit(const std::string& target_map, const std::string& temp_current_map) {
    sqlite3_stmt* stmt;
    std::string query = "SELECT exit_x, exit_y, exit_yaw FROM wormholes WHERE source_map = ? AND target_map = ?";

    ROS_INFO("current_map = %s", temp_current_map.c_str());
    ROS_INFO("target_map = %s", target_map.c_str());

    if (sqlite3_prepare_v2(m_pDb, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare exit position query!");
        return false;
    }

    sqlite3_bind_text(stmt, 1, temp_current_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        double exitX = sqlite3_column_double(stmt, 0);
        double exitY = sqlite3_column_double(stmt, 1);
        double exitYaw = sqlite3_column_double(stmt, 2);
        sqlite3_finalize(stmt);

        geometry_msgs::PoseWithCovarianceStamped initialPose;
        initialPose.header.stamp = ros::Time::now();

        std_srvs::Empty clearSrv;
        m_clearCostmapsClient.call(clearSrv);

        initialPose.header.frame_id = "map";
        initialPose.pose.pose.position.x = exitX;
        initialPose.pose.pose.position.y = exitY;
        initialPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(exitYaw);

        ROS_INFO("Exit coordinates: x=%f, y=%f, yaw=%f", exitX, exitY, exitYaw);

        for (int i = 0; i < m_maxRetries; ++i) {
            m_initialPosePub.publish(initialPose);
            ros::Duration(m_retryDelay).sleep();
        }

        ROS_INFO("Robot reinitialized at wormhole exit.");
        return true;
    }
    
    ROS_ERROR("Failed to find exit position in target map.");
    sqlite3_finalize(stmt);
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multimap_navigation_action_server");
    MultimapNavigationActionServer server("multimap_navigation");
    ros::spin();
    return 0;
}