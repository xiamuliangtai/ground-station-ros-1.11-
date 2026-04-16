#include <ros/ros.h>
#include <gs_msgs/NoFlyCells.h>
#include <gs_msgs/Waypoint.h>
#include <gs_msgs/WaypointArray.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <ctime>
#include <set>
#include <utility>
#include <vector>

class PlannerNode
{
public:
    PlannerNode()
    {
        std::srand(static_cast<unsigned>(std::time(nullptr)));
        sub_ = nh_.subscribe("/ui/no_fly_cells", 1, &PlannerNode::noFlyCallback, this);
        pub_ = nh_.advertise<gs_msgs::WaypointArray>("/planner/path", 1, true);
    }

private:
    using Cell = std::pair<int, int>;    // external: (col, row)
    using CellRC = std::pair<int, int>;  // internal: (row, col)

    static gs_msgs::Waypoint makeGridWaypoint(int col, int row)
    {
        gs_msgs::Waypoint wp;
        wp.x = static_cast<float>(col);
        wp.y = static_cast<float>(row);
        wp.z = 0.0f;
        wp.type = 0;
        wp.hold_ms = 0;
        return wp;
    }

    static CellRC toRC(const Cell& p)
    {
        return {p.second, p.first};
    }

    static Cell fromRC(const CellRC& p)
    {
        return {p.second, p.first};
    }

    std::set<CellRC> parseBlockedCellsRC(const gs_msgs::NoFlyCells::ConstPtr& msg) const
    {
        std::set<CellRC> blocked;
        for (auto code : msg->cells) {
            int v = static_cast<int>(code);
            int row = (v - 1) / 9 + 1;
            int col = (v - 1) % 9 + 1;
            blocked.insert({row, col});
        }
        return blocked;
    }

    static std::vector<CellRC> getNeighborsRC(const CellRC& p)
    {
        std::vector<CellRC> neighbors;
        int r = p.first;
        int c = p.second;

        if (r > 1) neighbors.push_back({r - 1, c});
        if (r < 7) neighbors.push_back({r + 1, c});
        if (c > 1) neighbors.push_back({r, c - 1});
        if (c < 9) neighbors.push_back({r, c + 1});

        return neighbors;
    }

    static int manhattanRC(const CellRC& a, const CellRC& b)
    {
        return std::abs(a.first - b.first) + std::abs(a.second - b.second);
    }

    bool dfs(const CellRC& current,
             std::vector<CellRC>& path,
             bool visited[8][10],
             int remainSteps,
             const std::chrono::steady_clock::time_point& startTime,
             bool& timeout) const
    {
        if (timeout) return false;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();

        if (elapsed > 800) {
            timeout = true;
            return false;
        }

        if (remainSteps == 0) {
            return true;
        }

        for (const auto& nb : getNeighborsRC(current)) {
            if (!visited[nb.first][nb.second]) {
                visited[nb.first][nb.second] = true;
                path.push_back(nb);

                if (dfs(nb, path, visited, remainSteps - 1, startTime, timeout)) {
                    return true;
                }

                path.pop_back();
                visited[nb.first][nb.second] = false;
            }
        }

        return false;
    }

    int countUnvisitedNeighbors(const CellRC& p,
                                const std::set<CellRC>& blocked,
                                const std::set<CellRC>& visited) const
    {
        int cnt = 0;
        for (const auto& nb : getNeighborsRC(p)) {
            if (!blocked.count(nb) && !visited.count(nb)) {
                ++cnt;
            }
        }
        return cnt;
    }

    int countSecondDegree(const CellRC& p,
                          const std::set<CellRC>& blocked,
                          const std::set<CellRC>& visited) const
    {
        int cnt = 0;
        for (const auto& nb : getNeighborsRC(p)) {
            if (!blocked.count(nb) && !visited.count(nb)) {
                for (const auto& nb2 : getNeighborsRC(nb)) {
                    if (!blocked.count(nb2) && !visited.count(nb2) && nb2 != p) {
                        ++cnt;
                    }
                }
            }
        }
        return cnt;
    }

    std::vector<CellRC> twoOpt(const std::vector<CellRC>& path) const
    {
        std::vector<CellRC> best = path;
        bool improved = true;

        while (improved) {
            improved = false;
            for (size_t i = 0; i + 2 < best.size(); ++i) {
                for (size_t j = i + 2; j + 1 < best.size(); ++j) {
                    int oldDist = manhattanRC(best[i], best[i + 1]) +
                                  manhattanRC(best[j], best[j + 1]);
                    int newDist = manhattanRC(best[i], best[j]) +
                                  manhattanRC(best[i + 1], best[j + 1]);

                    if (newDist < oldDist) {
                        std::reverse(best.begin() + static_cast<long>(i + 1),
                                     best.begin() + static_cast<long>(j + 1));
                        improved = true;
                    }
                }
            }
        }

        return best;
    }

    std::vector<Cell> findPathWithRepeat(const Cell& start,
                                         const std::set<CellRC>& blocked) const
    {
        CellRC startRC = toRC(start);

        std::vector<CellRC> pathRC;
        std::set<CellRC> visited;
        std::vector<CellRC> stack;

        int totalCells = 7 * 9 - static_cast<int>(blocked.size());
        if (totalCells == 0) return {};

        pathRC.push_back(startRC);
        visited.insert(startRC);
        stack.push_back(startRC);

        while (static_cast<int>(visited.size()) < totalCells) {
            CellRC current = stack.back();
            std::vector<CellRC> candidates;

            for (const auto& nb : getNeighborsRC(current)) {
                if (!blocked.count(nb) && !visited.count(nb)) {
                    candidates.push_back(nb);
                }
            }

            if (!candidates.empty()) {
                std::sort(candidates.begin(), candidates.end(),
                          [&](const CellRC& a, const CellRC& b) {
                              int aCnt1 = countUnvisitedNeighbors(a, blocked, visited);
                              int bCnt1 = countUnvisitedNeighbors(b, blocked, visited);
                              if (aCnt1 != bCnt1) return aCnt1 < bCnt1;

                              int aCnt2 = countSecondDegree(a, blocked, visited);
                              int bCnt2 = countSecondDegree(b, blocked, visited);
                              return aCnt2 < bCnt2;
                          });

                int bestCnt1 = countUnvisitedNeighbors(candidates.front(), blocked, visited);
                int bestCnt2 = countSecondDegree(candidates.front(), blocked, visited);

                std::vector<CellRC> bestCandidates;
                for (const auto& cand : candidates) {
                    int cnt1 = countUnvisitedNeighbors(cand, blocked, visited);
                    int cnt2 = countSecondDegree(cand, blocked, visited);
                    if (cnt1 == bestCnt1 && cnt2 == bestCnt2) {
                        bestCandidates.push_back(cand);
                    }
                }

                CellRC next = bestCandidates[std::rand() % bestCandidates.size()];
                pathRC.push_back(next);
                visited.insert(next);
                stack.push_back(next);
            } else {
                stack.pop_back();
                if (!stack.empty()) {
                    pathRC.push_back(stack.back());
                } else {
                    return {};
                }
            }
        }

        pathRC = twoOpt(pathRC);

        std::vector<Cell> result;
        result.reserve(pathRC.size());
        for (const auto& p : pathRC) {
            result.push_back(fromRC(p));
        }
        return result;
    }

    static std::vector<Cell> simplifyPath(const std::vector<Cell>& path)
    {
        if (path.size() <= 2) return path;

        std::vector<Cell> simplified;
        simplified.push_back(path.front());

        for (size_t i = 1; i + 1 < path.size(); ++i) {
            const auto& prev = path[i - 1];
            const auto& curr = path[i];
            const auto& next = path[i + 1];

            int dx1 = curr.first - prev.first;
            int dy1 = curr.second - prev.second;
            int dx2 = next.first - curr.first;
            int dy2 = next.second - curr.second;

            if (dx1 == dx2 && dy1 == dy2) {
                continue;
            }

            simplified.push_back(curr);
        }

        simplified.push_back(path.back());
        return simplified;
    }

    std::vector<Cell> generatePath(const std::set<CellRC>& blocked) const
    {
        bool visited[8][10] = {{false}};
        for (const auto& b : blocked) {
            visited[b.first][b.second] = true;
        }

        CellRC startRC = {7, 9}; // (row, col) => external (9,7)
        int totalSteps = 7 * 9 - static_cast<int>(blocked.size());

        std::vector<CellRC> pathRC;
        pathRC.push_back(startRC);
        visited[startRC.first][startRC.second] = true;

        auto startTime = std::chrono::steady_clock::now();
        bool timeout = false;

        std::vector<Cell> result;

        if (dfs(startRC, pathRC, visited, totalSteps - 1, startTime, timeout)) {
            result.reserve(pathRC.size());
            for (const auto& p : pathRC) {
                result.push_back(fromRC(p));
            }
        } else {
            std::vector<Cell> bestPath;
            int bestLength = INT_MAX;

            for (int i = 0; i < 100; ++i) {
                std::vector<Cell> path = findPathWithRepeat({9, 7}, blocked);
                if (!path.empty() && static_cast<int>(path.size()) < bestLength) {
                    bestLength = static_cast<int>(path.size());
                    bestPath = path;
                }
            }
            result = bestPath;
        }

        return simplifyPath(result);
    }

    gs_msgs::WaypointArray toWaypointArray(const std::vector<Cell>& path) const
    {
        gs_msgs::WaypointArray msg;
        for (const auto& cell : path) {
            msg.points.push_back(makeGridWaypoint(cell.first, cell.second));
        }
        return msg;
    }

    void noFlyCallback(const gs_msgs::NoFlyCells::ConstPtr& msg)
    {
        ROS_INFO("[planner_node] receive /ui/no_fly_cells, count=%lu",
                 static_cast<unsigned long>(msg->cells.size()));

        const std::set<CellRC> blocked = parseBlockedCellsRC(msg);
        const std::vector<Cell> path = generatePath(blocked);
        const gs_msgs::WaypointArray pathMsg = toWaypointArray(path);

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