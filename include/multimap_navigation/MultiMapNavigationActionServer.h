#ifndef MULTIMAP_NAVIGATION_ACTION_SERVER_H
#define MULTIMAP_NAVIGATION_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sqlite3.h>
#include <tf/transform_listener.h>
#include <multimap_navigation/MultimapNavigationActionAction.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/GetPlan.h>

class MultimapNavigationActionServer {
public:
    /**
     * @brief Constructor for the MultimapNavigationActionServer
     * @param name The name of the action server
     */
    explicit MultimapNavigationActionServer(const std::string& name);
    
    /**
     * @brief Destructor to clean up database connection
     */
    ~MultimapNavigationActionServer();

protected:
    /**
     * @brief Callback function for handling navigation action goals
     * @param goal The navigation goal containing target map and coordinates
     */
    void executeCB(const multimap_navigation::MultimapNavigationActionGoalConstPtr& goal);

    /**
     * @brief Retrieve the robot's current position.
     * @param[out] x X coordinate of the robot's current position.
     * @param[out] y Y coordinate of the robot's current position.
     * @return true if position retrieval is successful, false otherwise.
     */
    bool getCurrentRobotPosition(double& x, double& y);

private:
    /**
     * @brief Navigate to the wormhole entry point in current map
     * @param target_map The name of the target map
     * @return true if navigation successful, false otherwise
     */
    bool navigateToWormhole(const std::string& target_map);

    /**
     * @brief Switch the current map to the target map
     * @param target_map The name of the map to switch to
     * @return true if map switch successful, false otherwise
     */
    bool switchMap(const std::string& target_map);

    /**
     * @brief Send a goal to move_base
     * @param x X coordinate of the goal
     * @param y Y coordinate of the goal
     * @return true if navigation successful, false otherwise
     */
    bool sendMoveBaseGoal(double x, double y);

    /**
     * @brief Reinitialize robot position at wormhole exit in new map
     * @param target_map The name of the target map
     * @param temp_current_map The name of the previous map
     * @return true if reinitialization successful, false otherwise
     */
    bool reinitializeAtExit(int wormholeID);

    /**
     * @brief Publish Wormhole markers for the current map
     */
    void publishWormholeMarkers();

    /**
     * @brief Calculate the cost of the path to a wormhole
     * @param startX The start positions x coordinate value
     * @param startY the start positions y coordinate value
     * @param goalX The goal positions x coordinate value
     * @param goalY The goal positions y coordinate value
     * @return true if reinitialization successful, false otherwise
     */
    double calculatePathCost(double startX, double startY, double goalX, double goalY);


    // ROS-related member variables
    ros::NodeHandle m_nh;                      ///< ROS node handle
    ros::Publisher m_initialPosePub;           ///< Publisher for initial pose
    ros::Publisher m_markerPub;               ///< Publisher for markers
    ros::ServiceClient m_clearCostmapsClient;  ///< Service client for clearing costmaps
    ros::ServiceClient m_makePlanClient;  

    // Action server related variables
    actionlib::SimpleActionServer<multimap_navigation::MultimapNavigationActionAction> m_as;  ///< Action server for multimap navigation
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_moveBaseClient;          ///< Action client for move_base
    std::string m_actionName;                  ///< Name of the action server

    // Database and map related variables
    sqlite3* m_pDb;                           ///< Pointer to SQLite database connection
    std::string m_currentMap;                 ///< Current map name
    std::string m_dbPath;                     ///< Path to the database file
    std::string m_mapPath;                    ///< Base path for map files
    int m_selectedWormholeID;                 ///< ID of the selected wormhole

    // Navigation parameters
    const int m_maxRetries = 5;               ///< Maximum number of retries for pose publishing
    const double m_retryDelay = 0.5;          ///< Delay between retries in seconds
    const double m_mapSwitchDelay = 2.0;      ///< Delay for map switching in seconds
};

#endif // MULTIMAP_NAVIGATION_ACTION_SERVER_H
