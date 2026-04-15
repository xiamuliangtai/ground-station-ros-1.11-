#include <ros/ros.h>
#include <gs_msgs/NoFlyCells.h>
#include <gs_msgs/Waypoint.h>
#include <gs_msgs/WaypointArray.h>

#include <set>

class PlannerNode
{
public:
    PlannerNode()
    {
        sub_ = nh_.subscribe("/ui/no_fly_cells", 1, &PlannerNode::noFlyCallback, this);
        pub_ = nh_.advertise<gs_msgs::WaypointArray>("/planner/path", 1, true);
    }

private:
    // uint8 编码规则：
    // code = (row - 1) * 9 + col
    static int encodeCell(int col, int row)
    {
        return (row - 1) * 9 + col;
    }

    static gs_msgs::Waypoint makeGridWaypoint(int col, int row)
    {
        gs_msgs::Waypoint wp;
        wp.x = static_cast<float>(col);   // 第一阶段：暂用格子列号
        wp.y = static_cast<float>(row);   // 第一阶段：暂用格子行号
        wp.z = 0.0f;
        wp.type = 0;
        wp.hold_ms = 0;
        return wp;
    }

    std::set<int> parseBlockedCells(const gs_msgs::NoFlyCells::ConstPtr& msg) const
    {
        std::set<int> blocked;
        for (auto code : msg->cells) {
            blocked.insert(static_cast<int>(code));
        }
        return blocked;
    }

    // 第一阶段占位规划器：
    // 从 (9,7) 附近开始做列方向蛇形扫描，跳过禁飞格。
    // 当前目标是验证ROS链路稳定，不追求最优路径。
    gs_msgs::WaypointArray generateStage1SnakePath(const std::set<int>& blocked) const
    {
        gs_msgs::WaypointArray pathMsg;
        bool down = true;

        for (int col = 9; col >= 1; --col) {
            if (down) {
                for (int row = 7; row >= 1; --row) {
                    if (blocked.count(encodeCell(col, row))) continue;
                    pathMsg.points.push_back(makeGridWaypoint(col, row));
                }
            } else {
                for (int row = 1; row <= 7; ++row) {
                    if (blocked.count(encodeCell(col, row))) continue;
                    pathMsg.points.push_back(makeGridWaypoint(col, row));
                }
            }
            down = !down;
        }
        return pathMsg;
    }

    void noFlyCallback(const gs_msgs::NoFlyCells::ConstPtr& msg)
    {
        ROS_INFO("[planner_node] receive /ui/no_fly_cells, count=%lu",
                 static_cast<unsigned long>(msg->cells.size()));

        std::set<int> blocked = parseBlockedCells(msg);
        gs_msgs::WaypointArray pathMsg = generateStage1SnakePath(blocked);

        ROS_INFO("[planner_node] publish /planner/path, size=%lu",
                 static_cast<unsigned long>(pathMsg.points.size()));

        pub_.publish(pathMsg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    PlannerNode node;
    ros::spin();
    return 0;
}
