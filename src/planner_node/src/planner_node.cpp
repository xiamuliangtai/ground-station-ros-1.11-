#include <ros/ros.h>
#include <gs_msgs/NoFlyCells.h>
#include <gs_msgs/Waypoint.h>
#include <gs_msgs/WaypointArray.h>

#include <set>
#include <vector>
#include <utility>

class PlannerNode
{
public:
    PlannerNode()
    {
        sub_ = nh_.subscribe("/ui/no_fly_cells", 1, &PlannerNode::noFlyCallback, this);
        pub_ = nh_.advertise<gs_msgs::WaypointArray>("/planner/path", 1, true);
    }

private:
    void noFlyCallback(const gs_msgs::NoFlyCells::ConstPtr& msg)
    {
        ROS_INFO("[planner_node] receive /ui/no_fly_cells, count=%lu",
                 static_cast<unsigned long>(msg->cells.size()));

        std::set<int> blocked;
        for (auto code : msg->cells) {
            blocked.insert(static_cast<int>(code));
        }

        gs_msgs::WaypointArray pathMsg;

        // 第一阶段占位规划器：
        // 从 (9,7) 开始，按列蛇形扫描，跳过禁飞格
        bool down = true;
        for (int col = 9; col >= 1; --col) {
            if (down) {
                for (int row = 7; row >= 1; --row) {
                    int code = (row - 1) * 9 + col;
                    if (blocked.count(code)) continue;

                    gs_msgs::Waypoint wp;
                    wp.x = static_cast<float>(col);   // 第一阶段：用格子列号
                    wp.y = static_cast<float>(row);   // 第一阶段：用格子行号
                    wp.z = 0.0f;
                    wp.type = 0;
                    wp.hold_ms = 0;
                    pathMsg.points.push_back(wp);
                }
            } else {
                for (int row = 1; row <= 7; ++row) {
                    int code = (row - 1) * 9 + col;
                    if (blocked.count(code)) continue;

                    gs_msgs::Waypoint wp;
                    wp.x = static_cast<float>(col);
                    wp.y = static_cast<float>(row);
                    wp.z = 0.0f;
                    wp.type = 0;
                    wp.hold_ms = 0;
                    pathMsg.points.push_back(wp);
                }
            }
            down = !down;
        }

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
