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

    enum Direction
    {
        DIR_NONE  = 0,
        DIR_UP    = 1,
        DIR_DOWN  = 2,
        DIR_LEFT  = 3,
        DIR_RIGHT = 4
    };

    struct CandidatePath
    {
        std::vector<Cell> raw_path;
        std::vector<Cell> simplified_path;
        int corner_count = INT_MAX;
        int simplified_length = INT_MAX;
        int raw_length = INT_MAX;
        bool valid = false;
    };

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

    static int directionRC(const CellRC& from, const CellRC& to)
    {
        if (to.first < from.first) return DIR_UP;
        if (to.first > from.first) return DIR_DOWN;
        if (to.second < from.second) return DIR_LEFT;
        if (to.second > from.second) return DIR_RIGHT;
        return DIR_NONE;
    }

    static int modeRank(int mode, int dir)
    {
        static const int orders[4][4] = {
            {DIR_UP, DIR_LEFT, DIR_DOWN, DIR_RIGHT},
            {DIR_LEFT, DIR_UP, DIR_RIGHT, DIR_DOWN},
            {DIR_UP, DIR_RIGHT, DIR_DOWN, DIR_LEFT},
            {DIR_LEFT, DIR_DOWN, DIR_RIGHT, DIR_UP}
        };

        for (int i = 0; i < 4; ++i) {
            if (orders[mode % 4][i] == dir) {
                return i;
            }
        }
        return 99;
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

    int countUnvisitedNeighbors(const CellRC& p,
                                const std::set<CellRC>& blocked,
                                bool visited[8][10]) const
    {
        int cnt = 0;
        for (const auto& nb : getNeighborsRC(p)) {
            if (!blocked.count(nb) && !visited[nb.first][nb.second]) {
                ++cnt;
            }
        }
        return cnt;
    }

    int countSecondDegree(const CellRC& p,
                          const std::set<CellRC>& blocked,
                          bool visited[8][10]) const
    {
        int cnt = 0;
        for (const auto& nb : getNeighborsRC(p)) {
            if (!blocked.count(nb) && !visited[nb.first][nb.second]) {
                for (const auto& nb2 : getNeighborsRC(nb)) {
                    if (!blocked.count(nb2) &&
                        !visited[nb2.first][nb2.second] &&
                        nb2 != p) {
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

    std::vector<CellRC> sortedNeighborsForDfs(const CellRC& current,
                                              const std::set<CellRC>& blocked,
                                              bool visited[8][10],
                                              int prevDir,
                                              int mode) const
    {
        std::vector<CellRC> candidates;
        for (const auto& nb : getNeighborsRC(current)) {
            if (!blocked.count(nb) && !visited[nb.first][nb.second]) {
                candidates.push_back(nb);
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [&](const CellRC& a, const CellRC& b) {
                      int dirA = directionRC(current, a);
                      int dirB = directionRC(current, b);

                      int turnA = (prevDir == DIR_NONE) ? 0 : (dirA == prevDir ? 0 : 1);
                      int turnB = (prevDir == DIR_NONE) ? 0 : (dirB == prevDir ? 0 : 1);
                      if (turnA != turnB) return turnA < turnB;

                      int aCnt1 = countUnvisitedNeighbors(a, blocked, visited);
                      int bCnt1 = countUnvisitedNeighbors(b, blocked, visited);
                      if (aCnt1 != bCnt1) return aCnt1 < bCnt1;

                      int aCnt2 = countSecondDegree(a, blocked, visited);
                      int bCnt2 = countSecondDegree(b, blocked, visited);
                      if (aCnt2 != bCnt2) return aCnt2 < bCnt2;

                      return modeRank(mode, dirA) < modeRank(mode, dirB);
                  });

        return candidates;
    }

    bool dfsDirectional(const CellRC& current,
                        std::vector<CellRC>& path,
                        bool visited[8][10],
                        int remainSteps,
                        const std::set<CellRC>& blocked,
                        const std::chrono::steady_clock::time_point& startTime,
                        bool& timeout,
                        int prevDir,
                        int mode) const
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

        std::vector<CellRC> candidates =
            sortedNeighborsForDfs(current, blocked, visited, prevDir, mode);

        for (const auto& nb : candidates) {
            visited[nb.first][nb.second] = true;
            path.push_back(nb);

            int dir = directionRC(current, nb);
            if (dfsDirectional(nb, path, visited, remainSteps - 1,
                               blocked, startTime, timeout, dir, mode)) {
                return true;
            }

            path.pop_back();
            visited[nb.first][nb.second] = false;
        }

        return false;
    }

    std::vector<Cell> findPathWithRepeat(const Cell& start,
                                         const std::set<CellRC>& blocked,
                                         int mode) const
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
                int prevDir = DIR_NONE;
                if (pathRC.size() >= 2) {
                    prevDir = directionRC(pathRC[pathRC.size() - 2], pathRC[pathRC.size() - 1]);
                }

                std::sort(candidates.begin(), candidates.end(),
                          [&](const CellRC& a, const CellRC& b) {
                              int dirA = directionRC(current, a);
                              int dirB = directionRC(current, b);

                              int turnA = (prevDir == DIR_NONE) ? 0 : (dirA == prevDir ? 0 : 1);
                              int turnB = (prevDir == DIR_NONE) ? 0 : (dirB == prevDir ? 0 : 1);
                              if (turnA != turnB) return turnA < turnB;

                              int aCnt1 = countUnvisitedNeighbors(a, blocked, visited);
                              int bCnt1 = countUnvisitedNeighbors(b, blocked, visited);
                              if (aCnt1 != bCnt1) return aCnt1 < bCnt1;

                              int aCnt2 = countSecondDegree(a, blocked, visited);
                              int bCnt2 = countSecondDegree(b, blocked, visited);
                              if (aCnt2 != bCnt2) return aCnt2 < bCnt2;

                              return modeRank(mode, dirA) < modeRank(mode, dirB);
                          });

                int bestTurn = INT_MAX;
                int bestCnt1 = INT_MAX;
                int bestCnt2 = INT_MAX;
                int bestModeRank = INT_MAX;

                std::vector<CellRC> bestCandidates;

                for (const auto& cand : candidates) {
                    int dir = directionRC(current, cand);
                    int turn = (prevDir == DIR_NONE) ? 0 : (dir == prevDir ? 0 : 1);
                    int cnt1 = countUnvisitedNeighbors(cand, blocked, visited);
                    int cnt2 = countSecondDegree(cand, blocked, visited);
                    int rank = modeRank(mode, dir);

                    if (turn < bestTurn ||
                        (turn == bestTurn && cnt1 < bestCnt1) ||
                        (turn == bestTurn && cnt1 == bestCnt1 && cnt2 < bestCnt2) ||
                        (turn == bestTurn && cnt1 == bestCnt1 && cnt2 == bestCnt2 && rank < bestModeRank)) {
                        bestTurn = turn;
                        bestCnt1 = cnt1;
                        bestCnt2 = cnt2;
                        bestModeRank = rank;
                        bestCandidates.clear();
                        bestCandidates.push_back(cand);
                    } else if (turn == bestTurn &&
                               cnt1 == bestCnt1 &&
                               cnt2 == bestCnt2 &&
                               rank == bestModeRank) {
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

    static CandidatePath evaluateCandidate(const std::vector<Cell>& rawPath)
    {
        CandidatePath cand;
        if (rawPath.empty()) {
            return cand;
        }

        cand.raw_path = rawPath;
        cand.simplified_path = simplifyPath(rawPath);
        cand.corner_count = std::max(0, static_cast<int>(cand.simplified_path.size()) - 2);
        cand.simplified_length = static_cast<int>(cand.simplified_path.size());
        cand.raw_length = static_cast<int>(cand.raw_path.size());
        cand.valid = true;
        return cand;
    }

    static bool betterCandidate(const CandidatePath& lhs, const CandidatePath& rhs)
    {
        if (!lhs.valid) return false;
        if (!rhs.valid) return true;

        if (lhs.corner_count != rhs.corner_count) {
            return lhs.corner_count < rhs.corner_count;
        }

        if (lhs.simplified_length != rhs.simplified_length) {
            return lhs.simplified_length < rhs.simplified_length;
        }

        if (lhs.raw_length != rhs.raw_length) {
            return lhs.raw_length < rhs.raw_length;
        }

        return false;
    }

    std::vector<Cell> generatePath(const std::set<CellRC>& blocked) const
    {
        CandidatePath best;

        // 候选 1：多种方向偏好的 DFS
        for (int mode = 0; mode < 4; ++mode) {
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

            if (dfsDirectional(startRC, pathRC, visited, totalSteps - 1,
                               blocked, startTime, timeout, DIR_NONE, mode)) {
                std::vector<Cell> rawPath;
                rawPath.reserve(pathRC.size());
                for (const auto& p : pathRC) {
                    rawPath.push_back(fromRC(p));
                }

                CandidatePath dfsCandidate = evaluateCandidate(rawPath);
                if (betterCandidate(dfsCandidate, best)) {
                    best = dfsCandidate;
                }
            }
        }

        // 候选 2：多模式启发式搜索，方向连续优先
        const int trialPerMode = 80;
        for (int mode = 0; mode < 4; ++mode) {
            for (int i = 0; i < trialPerMode; ++i) {
                std::vector<Cell> rawPath = findPathWithRepeat({9, 7}, blocked, mode);
                CandidatePath repeatedCandidate = evaluateCandidate(rawPath);
                if (betterCandidate(repeatedCandidate, best)) {
                    best = repeatedCandidate;
                }
            }
        }

        if (!best.valid) {
            return {};
        }

        ROS_INFO("[planner_node] best candidate: corners=%d, simplified=%d, raw_len=%d",
                 best.corner_count,
                 best.simplified_length,
                 best.raw_length);

        return best.simplified_path;
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