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
    m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("wormholes_markers", 1);


    m_as.start();
    publishWormholeMarkers();
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

    std::vector<std::tuple<double, double, double>> wormholes;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        double entryX = sqlite3_column_double(stmt, 0);
        double entryY = sqlite3_column_double(stmt, 1);
        double entryYaw = sqlite3_column_double(stmt, 2);

        wormholes.emplace_back(entryX, entryY, entryYaw);
    }
    sqlite3_finalize(stmt);

    if (wormholes.empty()) {
        ROS_ERROR("No wormhole found for map transition!");
        return false;
    }

    // Get current robot position
    double robotX, robotY;
    if (!getCurrentRobotPosition(robotX, robotY)) {
        ROS_ERROR("Failed to get current robot position!");
        return false;
    }

    // Find the closest wormhole based on cost
    auto closestWormhole = std::min_element(wormholes.begin(), wormholes.end(),
        [this, robotX, robotY](const auto& w1, const auto& w2) {
            double cost1 = calculatePathCost(robotX, robotY, std::get<0>(w1), std::get<1>(w1));
            double cost2 = calculatePathCost(robotX, robotY, std::get<0>(w2), std::get<1>(w2));
            return cost1 < cost2;
        });

    double entryX = std::get<0>(*closestWormhole);
    double entryY = std::get<1>(*closestWormhole);
    double entryYaw = std::get<2>(*closestWormhole);

    // Print selected coordinates
    ROS_INFO("Selected wormhole entry coordinates: x = %.2f, y = %.2f, yaw = %.2f", entryX, entryY, entryYaw);

    // Send goal to the closest wormhole entry
    if (sendMoveBaseGoal(entryX, entryY)) {
        ROS_INFO("Reached wormhole entry, preparing to switch map.");
        return true;
    } else {
        ROS_ERROR("Failed to reach wormhole entry.");
        return false;
    }
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

bool MultimapNavigationActionServer::getCurrentRobotPosition(double& x, double& y) {
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr amclPose;

    amclPose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", m_nh, ros::Duration(5.0));
    if (amclPose) {
        x = amclPose->pose.pose.position.x;
        y = amclPose->pose.pose.position.y;
        ROS_INFO("Robot position retrieved: x = %.2f, y = %.2f", x, y);
        return true;
    } else {
        ROS_ERROR("Failed to get robot position from /amcl_pose!");
        return false;
    }
}

// a function to publish a marker array to all the wormhole entries from the current map
void MultimapNavigationActionServer::publishWormholeMarkers() {
    visualization_msgs::MarkerArray markerArray;

    // Create a Marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "wormholes";
    marker.id = 0;  // Unique ID for each marker if there are multiple, change if needed
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.4;  // Set the size of the points
    marker.scale.y = 0.4;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);  // 0 means the marker will last forever


    sqlite3_stmt* stmt;
    std::string query = "SELECT entry_x, entry_y FROM wormholes WHERE source_map =?";

    if (sqlite3_prepare_v2(m_pDb, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare wormhole entry query!");
        return;
    }

    sqlite3_bind_text(stmt, 1, m_currentMap.c_str(), -1, SQLITE_STATIC);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        double entryX = sqlite3_column_double(stmt, 0);
        double entryY = sqlite3_column_double(stmt, 1);

        //print the values of entryX and entryY
        ROS_INFO("Wormhole entry: x = %.2f, y = %.2f", entryX, entryY);

        geometry_msgs::Point p;
        p.x = entryX;
        p.y = entryY;




        // Add the point to the marker
        marker.points.push_back(p);
    }

    sqlite3_finalize(stmt);

    // Add the marker to the MarkerArray
    markerArray.markers.push_back(marker);

    if (!markerArray.markers.empty()) {
        ROS_INFO("Publishing %lu markers", markerArray.markers.size());
        //add a two second delay
        ros::Duration(2.0).sleep();
        m_markerPub.publish(markerArray);
    } else {
        ROS_WARN("MarkerArray is empty; no markers to publish.");
    }


    ROS_INFO("Published wormhole markers for map: %s", m_currentMap.c_str());
}

double MultimapNavigationActionServer::calculatePathCost(double startX, double startY, double goalX, double goalY) {
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = startX;
    srv.request.start.pose.position.y = startY;
    srv.request.start.pose.orientation.w = 1.0;  // Assume facing forward
    srv.request.goal.pose.position.x = goalX;
    srv.request.goal.pose.position.y = goalY;
    srv.request.goal.pose.orientation.w = 1.0;  // Assume facing forward

    if (m_makePlanClient.call(srv)) {
        double cost = 0.0;
        for (size_t i = 1; i < srv.response.plan.poses.size(); ++i) {
            const auto& p1 = srv.response.plan.poses[i - 1].pose.position;
            const auto& p2 = srv.response.plan.poses[i].pose.position;
            cost += std::hypot(p2.x - p1.x, p2.y - p1.y);
        }
        return cost;
    } else {
        ROS_ERROR("Failed to call make_plan service");
        return std::numeric_limits<double>::infinity();  // High cost if no path is found
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multimap_navigation_action_server");
    MultimapNavigationActionServer server("multimap_navigation");
    ros::spin();
    return 0;
}