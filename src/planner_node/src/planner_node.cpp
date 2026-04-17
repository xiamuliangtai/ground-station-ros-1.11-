#include <ros/ros.h>
#include <gs_msgs/NoFlyCells.h>
#include <gs_msgs/Waypoint.h>
#include <gs_msgs/WaypointArray.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstring>
#include <queue>
#include <string>
#include <utility>
#include <vector>

class PlannerNode
{
public:
    PlannerNode()
    {
        sub_ = nh_.subscribe("/ui/no_fly_cells", 1, &PlannerNode::noFlyCallback, this);
        pub_ = nh_.advertise<gs_msgs::WaypointArray>("/planner/path", 1, true);
    }

private:
    static const int kRows = 7;
    static const int kCols = 9;
    static const int kDirCount = 4;

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

    struct Grid
    {
        bool blocked[kRows + 1][kCols + 1];
        int total_free;

        Grid() : total_free(0)
        {
            std::memset(blocked, 0, sizeof(blocked));
        }
    };

    struct SimpleMove
    {
        CellRC next;
        bool disconnects_remainder;
        int turn_penalty;
        int corridor_len;
        int boundary_score;
        int future_degree;
        int mode_rank;
        int row;
        int col;
    };

    struct CoverMove
    {
        CellRC next;
        int edge_reuse_penalty;
        int turn_penalty;
        int uturn_penalty;
        int corridor_len;
        int boundary_score;
        int future_degree;
        int mode_rank;
        int row;
        int col;
    };

    struct DijkstraNode
    {
        int dist;
        bool used;

        DijkstraNode() : dist(INT_MAX), used(false) {}
    };

    struct DijkstraParent
    {
        int row;
        int col;
        int dir;
        bool has_parent;

        DijkstraParent() : row(0), col(0), dir(0), has_parent(false) {}
    };

    struct CandidatePath
    {
        std::vector<CellRC> raw_path_rc;
        std::vector<Cell> simplified_path;

        int segment_cross_count;
        int repeated_edge_count;
        int repeated_cell_visits;
        int immediate_backtracks;
        int corner_count;
        int raw_length;
        int first_segment_len;
        int planner_rank;   // 0=strict_simple, 1=low_overlap
        int total_score;
        std::string source;
        bool valid;

        CandidatePath()
            : segment_cross_count(INT_MAX),
              repeated_edge_count(INT_MAX),
              repeated_cell_visits(INT_MAX),
              immediate_backtracks(INT_MAX),
              corner_count(INT_MAX),
              raw_length(INT_MAX),
              first_segment_len(-1),
              planner_rank(INT_MAX),
              total_score(INT_MAX),
              valid(false)
        {
        }
    };

private:
    static bool inBoundsRC(int row, int col)
    {
        return row >= 1 && row <= kRows && col >= 1 && col <= kCols;
    }

    static Cell fromRC(const CellRC& p)
    {
        return Cell(p.second, p.first);
    }

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

    static int directionRC(const CellRC& from, const CellRC& to)
    {
        if (to.first < from.first) return DIR_UP;
        if (to.first > from.first) return DIR_DOWN;
        if (to.second < from.second) return DIR_LEFT;
        if (to.second > from.second) return DIR_RIGHT;
        return DIR_NONE;
    }

    static bool isOppositeDir(int d1, int d2)
    {
        return (d1 == DIR_UP    && d2 == DIR_DOWN) ||
               (d1 == DIR_DOWN  && d2 == DIR_UP)   ||
               (d1 == DIR_LEFT  && d2 == DIR_RIGHT)||
               (d1 == DIR_RIGHT && d2 == DIR_LEFT);
    }

    static void stepByDirection(const CellRC& p, int dir, CellRC& out)
    {
        out = p;
        if (dir == DIR_UP)    out.first -= 1;
        if (dir == DIR_DOWN)  out.first += 1;
        if (dir == DIR_LEFT)  out.second -= 1;
        if (dir == DIR_RIGHT) out.second += 1;
    }

    static int modeRank(int mode, int dir)
    {
        static const int orders[4][4] = {
            {DIR_UP,   DIR_LEFT,  DIR_DOWN, DIR_RIGHT},
            {DIR_LEFT, DIR_UP,    DIR_RIGHT, DIR_DOWN},
            {DIR_UP,   DIR_RIGHT, DIR_DOWN, DIR_LEFT},
            {DIR_LEFT, DIR_DOWN,  DIR_RIGHT, DIR_UP}
        };

        for (int i = 0; i < 4; ++i) {
            if (orders[mode % 4][i] == dir) {
                return i;
            }
        }
        return 99;
    }

    Grid parseBlockedCells(const gs_msgs::NoFlyCells::ConstPtr& msg) const
    {
        Grid grid;
        grid.total_free = kRows * kCols;

        for (std::size_t i = 0; i < msg->cells.size(); ++i) {
            int code = static_cast<int>(msg->cells[i]);
            int row = (code - 1) / kCols + 1;
            int col = (code - 1) % kCols + 1;

            if (inBoundsRC(row, col) && !grid.blocked[row][col]) {
                grid.blocked[row][col] = true;
                --grid.total_free;
            }
        }

        return grid;
    }

    static void getNeighborsRC(const CellRC& p, CellRC out[4], int& count)
    {
        count = 0;
        int r = p.first;
        int c = p.second;

        if (r > 1)     out[count++] = CellRC(r - 1, c);
        if (r < kRows) out[count++] = CellRC(r + 1, c);
        if (c > 1)     out[count++] = CellRC(r, c - 1);
        if (c < kCols) out[count++] = CellRC(r, c + 1);
    }

    static int countReachableFree(const Grid& grid, const CellRC& start)
    {
        if (grid.blocked[start.first][start.second]) {
            return 0;
        }

        bool seen[kRows + 1][kCols + 1];
        std::memset(seen, 0, sizeof(seen));

        std::queue<CellRC> q;
        q.push(start);
        seen[start.first][start.second] = true;

        int count = 0;
        while (!q.empty()) {
            CellRC cur = q.front();
            q.pop();
            ++count;

            CellRC neighbors[4];
            int n = 0;
            getNeighborsRC(cur, neighbors, n);

            for (int i = 0; i < n; ++i) {
                const CellRC& nb = neighbors[i];
                if (grid.blocked[nb.first][nb.second]) continue;
                if (seen[nb.first][nb.second]) continue;
                seen[nb.first][nb.second] = true;
                q.push(nb);
            }
        }

        return count;
    }

    static int countAvailableNeighbors(const CellRC& p,
                                       const Grid& grid,
                                       const bool occupied[kRows + 1][kCols + 1],
                                       const CellRC* extra_forbidden = NULL)
    {
        CellRC neighbors[4];
        int n = 0;
        getNeighborsRC(p, neighbors, n);

        int count = 0;
        for (int i = 0; i < n; ++i) {
            const CellRC& nb = neighbors[i];
            if (grid.blocked[nb.first][nb.second]) continue;
            if (occupied[nb.first][nb.second]) continue;
            if (extra_forbidden != NULL && nb == *extra_forbidden) continue;
            ++count;
        }
        return count;
    }

    static int corridorLength(const CellRC& current,
                              const CellRC& next,
                              const Grid& grid,
                              const bool occupied[kRows + 1][kCols + 1])
    {
        int dir = directionRC(current, next);
        CellRC p = next;
        int len = 1;

        while (true) {
            CellRC q;
            stepByDirection(p, dir, q);
            if (!inBoundsRC(q.first, q.second)) break;
            if (grid.blocked[q.first][q.second]) break;
            if (occupied[q.first][q.second]) break;
            ++len;
            p = q;
        }

        return len;
    }

    static int boundaryExposure(const CellRC& p,
                                const Grid& grid,
                                const bool occupied[kRows + 1][kCols + 1])
    {
        static const int dr[4] = {-1, 1, 0, 0};
        static const int dc[4] = {0, 0, -1, 1};

        int score = 0;
        for (int i = 0; i < 4; ++i) {
            int nr = p.first + dr[i];
            int nc = p.second + dc[i];
            if (!inBoundsRC(nr, nc) || grid.blocked[nr][nc] || occupied[nr][nc]) {
                ++score;
            }
        }
        return score;
    }

    static bool hasReachableFuture(const CellRC& candidate,
                                   int occupied_count,
                                   const Grid& grid,
                                   const bool occupied[kRows + 1][kCols + 1])
    {
        if (occupied_count + 1 >= grid.total_free) {
            return true;
        }
        return countAvailableNeighbors(candidate, grid, occupied, &candidate) > 0;
    }

    static bool remainderConnectedAfterMove(const CellRC& candidate,
                                            const Grid& grid,
                                            const bool occupied[kRows + 1][kCols + 1])
    {
        bool seen[kRows + 1][kCols + 1];
        std::memset(seen, 0, sizeof(seen));

        std::queue<CellRC> q;
        bool found_seed = false;
        int target = 0;

        for (int row = 1; row <= kRows; ++row) {
            for (int col = 1; col <= kCols; ++col) {
                if (grid.blocked[row][col]) continue;
                if (occupied[row][col]) continue;
                if (row == candidate.first && col == candidate.second) continue;
                ++target;
                if (!found_seed) {
                    q.push(CellRC(row, col));
                    seen[row][col] = true;
                    found_seed = true;
                }
            }
        }

        if (target <= 1) {
            return true;
        }

        int reached = 0;
        while (!q.empty()) {
            CellRC cur = q.front();
            q.pop();
            ++reached;

            CellRC neighbors[4];
            int n = 0;
            getNeighborsRC(cur, neighbors, n);

            for (int i = 0; i < n; ++i) {
                const CellRC& nb = neighbors[i];
                if (grid.blocked[nb.first][nb.second]) continue;
                if (occupied[nb.first][nb.second]) continue;
                if (nb == candidate) continue;
                if (seen[nb.first][nb.second]) continue;
                seen[nb.first][nb.second] = true;
                q.push(nb);
            }
        }

        return reached == target;
    }

    std::vector<SimpleMove> buildSimpleMoves(const CellRC& current,
                                             int prev_dir,
                                             int occupied_count,
                                             const Grid& grid,
                                             const bool occupied[kRows + 1][kCols + 1],
                                             int mode) const
    {
        CellRC neighbors[4];
        int n = 0;
        getNeighborsRC(current, neighbors, n);

        std::vector<SimpleMove> moves;
        moves.reserve(4);

        for (int i = 0; i < n; ++i) {
            const CellRC& nb = neighbors[i];
            if (grid.blocked[nb.first][nb.second]) continue;
            if (occupied[nb.first][nb.second]) continue;

            int dir = directionRC(current, nb);

            SimpleMove mv;
            mv.next = nb;
            mv.turn_penalty = (prev_dir == DIR_NONE || prev_dir == dir) ? 0 : 1;
            mv.corridor_len = corridorLength(current, nb, grid, occupied);
            mv.boundary_score = boundaryExposure(nb, grid, occupied);
            mv.future_degree = countAvailableNeighbors(nb, grid, occupied, &nb);
            mv.disconnects_remainder =
                !hasReachableFuture(nb, occupied_count, grid, occupied) ||
                !remainderConnectedAfterMove(nb, grid, occupied);
            mv.mode_rank = modeRank(mode, dir);
            mv.row = nb.first;
            mv.col = nb.second;
            moves.push_back(mv);
        }

        std::sort(moves.begin(), moves.end(),
                  [](const SimpleMove& a, const SimpleMove& b) {
                      if (a.disconnects_remainder != b.disconnects_remainder)
                          return !a.disconnects_remainder;
                      if (a.turn_penalty != b.turn_penalty)
                          return a.turn_penalty < b.turn_penalty;
                      if (a.corridor_len != b.corridor_len)
                          return a.corridor_len > b.corridor_len;
                      if (a.boundary_score != b.boundary_score)
                          return a.boundary_score > b.boundary_score;
                      if (a.future_degree != b.future_degree)
                          return a.future_degree < b.future_degree;
                      if (a.mode_rank != b.mode_rank)
                          return a.mode_rank < b.mode_rank;
                      if (a.row != b.row)
                          return a.row < b.row;
                      return a.col < b.col;
                  });

        return moves;
    }

    bool strictSimpleWalk(const Grid& grid,
                          const CellRC& start,
                          const CellRC& first,
                          int mode,
                          std::vector<CellRC>& out_path) const
    {
        bool occupied[kRows + 1][kCols + 1];
        std::memset(occupied, 0, sizeof(occupied));

        occupied[start.first][start.second] = true;
        occupied[first.first][first.second] = true;

        out_path.clear();
        out_path.reserve(grid.total_free);
        out_path.push_back(start);
        out_path.push_back(first);

        int occupied_count = 2;
        int prev_dir = directionRC(start, first);

        while (occupied_count < grid.total_free) {
            CellRC current = out_path.back();
            std::vector<SimpleMove> moves =
                buildSimpleMoves(current, prev_dir, occupied_count, grid, occupied, mode);

            if (moves.empty()) return false;
            if (moves.front().disconnects_remainder) return false;

            CellRC next = moves.front().next;
            occupied[next.first][next.second] = true;
            out_path.push_back(next);
            ++occupied_count;
            prev_dir = directionRC(current, next);
        }

        return true;
    }

    static int& edgeUseAt(const CellRC& a,
                          const CellRC& b,
                          int horiz[kRows + 1][kCols + 1],
                          int vert[kRows + 1][kCols + 1])
    {
        if (a.first == b.first) {
            int row = a.first;
            int col = std::min(a.second, b.second);
            return horiz[row][col];
        } else {
            int row = std::min(a.first, b.first);
            int col = a.second;
            return vert[row][col];
        }
    }

    static int edgeUseValue(const CellRC& a,
                            const CellRC& b,
                            const int horiz[kRows + 1][kCols + 1],
                            const int vert[kRows + 1][kCols + 1])
    {
        if (a.first == b.first) {
            int row = a.first;
            int col = std::min(a.second, b.second);
            return horiz[row][col];
        } else {
            int row = std::min(a.first, b.first);
            int col = a.second;
            return vert[row][col];
        }
    }

    std::vector<CoverMove> buildCoverMoves(const CellRC& current,
                                           int prev_dir,
                                           const Grid& grid,
                                           const bool covered[kRows + 1][kCols + 1],
                                           const int horiz[kRows + 1][kCols + 1],
                                           const int vert[kRows + 1][kCols + 1],
                                           int mode,
                                           const std::vector<CellRC>& path) const
    {
        CellRC neighbors[4];
        int n = 0;
        getNeighborsRC(current, neighbors, n);

        std::vector<CoverMove> moves;
        moves.reserve(4);

        CellRC prev_cell(-1, -1);
        if (path.size() >= 2) {
            prev_cell = path[path.size() - 2];
        }

        for (int i = 0; i < n; ++i) {
            const CellRC& nb = neighbors[i];
            if (grid.blocked[nb.first][nb.second]) continue;
            if (covered[nb.first][nb.second]) continue;

            int dir = directionRC(current, nb);

            CoverMove mv;
            mv.next = nb;
            mv.edge_reuse_penalty = edgeUseValue(current, nb, horiz, vert);
            mv.turn_penalty = (prev_dir == DIR_NONE || prev_dir == dir) ? 0 : 1;
            mv.uturn_penalty = (prev_cell == nb) ? 1 : 0;
            mv.corridor_len = corridorLength(current, nb, grid, covered);
            mv.boundary_score = boundaryExposure(nb, grid, covered);
            mv.future_degree = countAvailableNeighbors(nb, grid, covered, &nb);
            mv.mode_rank = modeRank(mode, dir);
            mv.row = nb.first;
            mv.col = nb.second;
            moves.push_back(mv);
        }

        std::sort(moves.begin(), moves.end(),
                  [](const CoverMove& a, const CoverMove& b) {
                      if (a.edge_reuse_penalty != b.edge_reuse_penalty)
                          return a.edge_reuse_penalty < b.edge_reuse_penalty;
                      if (a.uturn_penalty != b.uturn_penalty)
                          return a.uturn_penalty < b.uturn_penalty;
                      if (a.turn_penalty != b.turn_penalty)
                          return a.turn_penalty < b.turn_penalty;
                      if (a.corridor_len != b.corridor_len)
                          return a.corridor_len > b.corridor_len;
                      if (a.boundary_score != b.boundary_score)
                          return a.boundary_score > b.boundary_score;
                      if (a.future_degree != b.future_degree)
                          return a.future_degree < b.future_degree;
                      if (a.mode_rank != b.mode_rank)
                          return a.mode_rank < b.mode_rank;
                      if (a.row != b.row)
                          return a.row < b.row;
                      return a.col < b.col;
                  });

        return moves;
    }

    bool findConnectorPath(const Grid& grid,
                           const CellRC& start,
                           int prev_dir,
                           int mode,
                           const bool covered[kRows + 1][kCols + 1],
                           const int horiz[kRows + 1][kCols + 1],
                           const int vert[kRows + 1][kCols + 1],
                           std::vector<CellRC>& out_path) const
    {
        DijkstraNode dist[kRows + 1][kCols + 1][kDirCount + 1];
        DijkstraParent parent[kRows + 1][kCols + 1][kDirCount + 1];

        dist[start.first][start.second][prev_dir].dist = 0;

        while (true) {
            int best_r = -1;
            int best_c = -1;
            int best_d = -1;
            int best_dist = INT_MAX;

            for (int r = 1; r <= kRows; ++r) {
                for (int c = 1; c <= kCols; ++c) {
                    if (grid.blocked[r][c]) continue;
                    for (int d = 0; d <= kDirCount; ++d) {
                        if (dist[r][c][d].used) continue;
                        if (dist[r][c][d].dist >= best_dist) continue;
                        best_dist = dist[r][c][d].dist;
                        best_r = r;
                        best_c = c;
                        best_d = d;
                    }
                }
            }

            if (best_r == -1) {
                break;
            }

            dist[best_r][best_c][best_d].used = true;
            CellRC cur(best_r, best_c);

            if (!(cur == start) && !covered[cur.first][cur.second]) {
                std::vector<CellRC> rev;
                int rr = best_r;
                int cc = best_c;
                int dd = best_d;

                while (true) {
                    rev.push_back(CellRC(rr, cc));
                    if (rr == start.first && cc == start.second && dd == prev_dir) {
                        break;
                    }

                    DijkstraParent p = parent[rr][cc][dd];
                    if (!p.has_parent) break;

                    rr = p.row;
                    cc = p.col;
                    dd = p.dir;
                }

                std::reverse(rev.begin(), rev.end());
                out_path = rev;
                return !out_path.empty();
            }

            CellRC neighbors[4];
            int n = 0;
            getNeighborsRC(cur, neighbors, n);

            for (int i = 0; i < n; ++i) {
                const CellRC& nb = neighbors[i];
                if (grid.blocked[nb.first][nb.second]) continue;

                int move_dir = directionRC(cur, nb);

                int step_cost = 12;
                step_cost += 50 * edgeUseValue(cur, nb, horiz, vert);
                step_cost += covered[nb.first][nb.second] ? 8 : 0;
                step_cost += (best_d != DIR_NONE && move_dir != best_d) ? 4 : 0;
                step_cost += (best_d != DIR_NONE && isOppositeDir(best_d, move_dir)) ? 20 : 0;
                step_cost += modeRank(mode, move_dir);

                int nd = dist[best_r][best_c][best_d].dist + step_cost;
                if (nd < dist[nb.first][nb.second][move_dir].dist) {
                    dist[nb.first][nb.second][move_dir].dist = nd;
                    parent[nb.first][nb.second][move_dir].row = best_r;
                    parent[nb.first][nb.second][move_dir].col = best_c;
                    parent[nb.first][nb.second][move_dir].dir = best_d;
                    parent[nb.first][nb.second][move_dir].has_parent = true;
                }
            }
        }

        return false;
    }

    bool lowOverlapCoverWalk(const Grid& grid,
                             const CellRC& start,
                             int mode,
                             std::vector<CellRC>& out_path) const
    {
        bool covered[kRows + 1][kCols + 1];
        int horiz[kRows + 1][kCols + 1];
        int vert[kRows + 1][kCols + 1];

        std::memset(covered, 0, sizeof(covered));
        std::memset(horiz, 0, sizeof(horiz));
        std::memset(vert, 0, sizeof(vert));

        out_path.clear();
        out_path.reserve(grid.total_free * 2);
        out_path.push_back(start);

        covered[start.first][start.second] = true;
        int covered_count = 1;
        int prev_dir = DIR_NONE;

        while (covered_count < grid.total_free) {
            CellRC current = out_path.back();

            std::vector<CoverMove> direct_moves =
                buildCoverMoves(current, prev_dir, grid, covered, horiz, vert, mode, out_path);

            if (!direct_moves.empty()) {
                CellRC next = direct_moves.front().next;
                int& use_ref = edgeUseAt(current, next, horiz, vert);
                ++use_ref;

                out_path.push_back(next);
                if (!covered[next.first][next.second]) {
                    covered[next.first][next.second] = true;
                    ++covered_count;
                }
                prev_dir = directionRC(current, next);
                continue;
            }

            std::vector<CellRC> connector;
            if (!findConnectorPath(grid, current, prev_dir, mode, covered, horiz, vert, connector)) {
                return false;
            }

            if (connector.size() <= 1) {
                return false;
            }

            for (std::size_t i = 1; i < connector.size(); ++i) {
                CellRC a = out_path.back();
                CellRC b = connector[i];

                int& use_ref = edgeUseAt(a, b, horiz, vert);
                ++use_ref;

                out_path.push_back(b);
                if (!covered[b.first][b.second]) {
                    covered[b.first][b.second] = true;
                    ++covered_count;
                }
                prev_dir = directionRC(a, b);
            }
        }

        return true;
    }

    static std::vector<Cell> simplifyPathRC(const std::vector<CellRC>& raw_path_rc)
    {
        std::vector<Cell> raw_path;
        raw_path.reserve(raw_path_rc.size());

        for (std::size_t i = 0; i < raw_path_rc.size(); ++i) {
            raw_path.push_back(fromRC(raw_path_rc[i]));
        }

        if (raw_path.size() <= 2) {
            return raw_path;
        }

        std::vector<Cell> simplified;
        simplified.reserve(raw_path.size());
        simplified.push_back(raw_path.front());

        for (std::size_t i = 1; i + 1 < raw_path.size(); ++i) {
            const Cell& prev = raw_path[i - 1];
            const Cell& curr = raw_path[i];
            const Cell& next = raw_path[i + 1];

            int dx1 = curr.first - prev.first;
            int dy1 = curr.second - prev.second;
            int dx2 = next.first - curr.first;
            int dy2 = next.second - curr.second;

            if (dx1 == dx2 && dy1 == dy2) {
                continue;
            }

            simplified.push_back(curr);
        }

        simplified.push_back(raw_path.back());
        return simplified;
    }

    static bool rangeOverlap(int a1, int a2, int b1, int b2)
    {
        int l1 = std::min(a1, a2);
        int r1 = std::max(a1, a2);
        int l2 = std::min(b1, b2);
        int r2 = std::max(b1, b2);
        return !(r1 < l2 || r2 < l1);
    }

    static int countSegmentCrossings(const std::vector<Cell>& path)
    {
        if (path.size() < 4) return 0;

        int count = 0;
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
            Cell a1 = path[i];
            Cell a2 = path[i + 1];
            bool a_h = (a1.second == a2.second);
            bool a_v = (a1.first == a2.first);

            for (std::size_t j = i + 2; j + 1 < path.size(); ++j) {
                if (j == i + 1) continue;

                Cell b1 = path[j];
                Cell b2 = path[j + 1];
                bool b_h = (b1.second == b2.second);
                bool b_v = (b1.first == b2.first);

                if (a_h && b_v) {
                    int y = a1.second;
                    int x = b1.first;
                    if (rangeOverlap(a1.first, a2.first, x, x) &&
                        rangeOverlap(b1.second, b2.second, y, y)) {
                        if (!(a2 == b1) && !(a1 == b2)) {
                            ++count;
                        }
                    }
                } else if (a_v && b_h) {
                    int x = a1.first;
                    int y = b1.second;
                    if (rangeOverlap(a1.second, a2.second, y, y) &&
                        rangeOverlap(b1.first, b2.first, x, x)) {
                        if (!(a2 == b1) && !(a1 == b2)) {
                            ++count;
                        }
                    }
                }
            }
        }

        return count;
    }

    static CandidatePath evaluateCandidate(const std::vector<CellRC>& raw_path_rc,
                                           int planner_rank,
                                           const std::string& source)
    {
        CandidatePath cand;
        if (raw_path_rc.empty()) {
            return cand;
        }

        cand.raw_path_rc = raw_path_rc;
        cand.simplified_path = simplifyPathRC(raw_path_rc);
        cand.raw_length = static_cast<int>(raw_path_rc.size());
        cand.corner_count = std::max(0, static_cast<int>(cand.simplified_path.size()) - 2);
        cand.planner_rank = planner_rank;
        cand.source = source;
        cand.valid = true;

        if (cand.simplified_path.size() >= 2) {
            cand.first_segment_len =
                std::abs(cand.simplified_path[0].first - cand.simplified_path[1].first) +
                std::abs(cand.simplified_path[0].second - cand.simplified_path[1].second);
        }

        bool seen_cell[kRows + 1][kCols + 1];
        int horiz[kRows + 1][kCols + 1];
        int vert[kRows + 1][kCols + 1];
        std::memset(seen_cell, 0, sizeof(seen_cell));
        std::memset(horiz, 0, sizeof(horiz));
        std::memset(vert, 0, sizeof(vert));

        cand.repeated_edge_count = 0;
        cand.repeated_cell_visits = 0;
        cand.immediate_backtracks = 0;

        for (std::size_t i = 0; i < raw_path_rc.size(); ++i) {
            const CellRC& p = raw_path_rc[i];
            if (seen_cell[p.first][p.second]) {
                ++cand.repeated_cell_visits;
            } else {
                seen_cell[p.first][p.second] = true;
            }
        }

        for (std::size_t i = 1; i < raw_path_rc.size(); ++i) {
            const CellRC& a = raw_path_rc[i - 1];
            const CellRC& b = raw_path_rc[i];
            int& use_ref = edgeUseAt(a, b, horiz, vert);
            ++use_ref;
            if (use_ref > 1) {
                ++cand.repeated_edge_count;
            }
        }

        for (std::size_t i = 1; i + 1 < raw_path_rc.size(); ++i) {
            if (raw_path_rc[i - 1] == raw_path_rc[i + 1]) {
                ++cand.immediate_backtracks;
            }
        }

        cand.segment_cross_count = countSegmentCrossings(cand.simplified_path);

        cand.total_score =
            1200 * cand.segment_cross_count +
            450  * cand.immediate_backtracks +
            220  * cand.repeated_edge_count +
            40   * cand.corner_count +
            15   * cand.repeated_cell_visits +
            cand.raw_length;

        return cand;
    }

    static bool betterCandidate(const CandidatePath& lhs, const CandidatePath& rhs)
    {
        if (!lhs.valid) return false;
        if (!rhs.valid) return true;

        if (lhs.total_score != rhs.total_score)
            return lhs.total_score < rhs.total_score;
        if (lhs.segment_cross_count != rhs.segment_cross_count)
            return lhs.segment_cross_count < rhs.segment_cross_count;
        if (lhs.repeated_edge_count != rhs.repeated_edge_count)
            return lhs.repeated_edge_count < rhs.repeated_edge_count;
        if (lhs.immediate_backtracks != rhs.immediate_backtracks)
            return lhs.immediate_backtracks < rhs.immediate_backtracks;
        if (lhs.corner_count != rhs.corner_count)
            return lhs.corner_count < rhs.corner_count;
        if (lhs.first_segment_len != rhs.first_segment_len)
            return lhs.first_segment_len > rhs.first_segment_len;
        if (lhs.raw_length != rhs.raw_length)
            return lhs.raw_length < rhs.raw_length;
        if (lhs.planner_rank != rhs.planner_rank)
            return lhs.planner_rank < rhs.planner_rank;
        return lhs.source < rhs.source;
    }

    std::vector<CellRC> getInitialSteps(const Grid& grid, const CellRC& start) const
    {
        std::vector<CellRC> steps;
        steps.reserve(4);

        CellRC up(start.first - 1, start.second);
        CellRC left(start.first, start.second - 1);
        CellRC down(start.first + 1, start.second);
        CellRC right(start.first, start.second + 1);

        if (inBoundsRC(up.first, up.second) && !grid.blocked[up.first][up.second]) {
            steps.push_back(up);
        }
        if (inBoundsRC(left.first, left.second) && !grid.blocked[left.first][left.second]) {
            steps.push_back(left);
        }
        if (inBoundsRC(down.first, down.second) && !grid.blocked[down.first][down.second]) {
            steps.push_back(down);
        }
        if (inBoundsRC(right.first, right.second) && !grid.blocked[right.first][right.second]) {
            steps.push_back(right);
        }

        return steps;
    }

    std::vector<Cell> generatePath(const Grid& grid) const
    {
        const CellRC start(7, 9); // external (9,7)

        if (grid.total_free <= 0) {
            return std::vector<Cell>();
        }

        if (grid.blocked[start.first][start.second]) {
            ROS_WARN("[planner_node] start cell is blocked");
            return std::vector<Cell>();
        }

        int reachable = countReachableFree(grid, start);
        if (reachable != grid.total_free) {
            ROS_WARN("[planner_node] free cells are disconnected from start, reachable=%d total=%d",
                     reachable, grid.total_free);
            return std::vector<Cell>();
        }

        if (grid.total_free == 1) {
            return std::vector<Cell>(1, fromRC(start));
        }

        CandidatePath best;
        std::vector<CellRC> initial_steps = getInitialSteps(grid, start);

        for (std::size_t i = 0; i < initial_steps.size(); ++i) {
            const CellRC& first = initial_steps[i];
            for (int mode = 0; mode < 4; ++mode) {
                std::vector<CellRC> raw_path_rc;
                if (strictSimpleWalk(grid, start, first, mode, raw_path_rc)) {
                    CandidatePath cand = evaluateCandidate(raw_path_rc, 0, "strict_simple");
                    if (betterCandidate(cand, best)) {
                        best = cand;
                    }
                }
            }
        }

        for (int mode = 0; mode < 4; ++mode) {
            std::vector<CellRC> raw_path_rc;
            if (lowOverlapCoverWalk(grid, start, mode, raw_path_rc)) {
                CandidatePath cand = evaluateCandidate(raw_path_rc, 1, "low_overlap");
                if (betterCandidate(cand, best)) {
                    best = cand;
                }
            }
        }

        if (!best.valid) {
            ROS_WARN("[planner_node] no valid path found");
            return std::vector<Cell>();
        }

        ROS_INFO("[planner_node] best candidate: source=%s, score=%d, cross=%d, repeat_edge=%d, backtrack=%d, corners=%d, raw_len=%d",
                 best.source.c_str(),
                 best.total_score,
                 best.segment_cross_count,
                 best.repeated_edge_count,
                 best.immediate_backtracks,
                 best.corner_count,
                 best.raw_length);

        return best.simplified_path;
    }

    gs_msgs::WaypointArray toWaypointArray(const std::vector<Cell>& path) const
    {
        gs_msgs::WaypointArray msg;
        msg.points.reserve(path.size());
        for (std::size_t i = 0; i < path.size(); ++i) {
            msg.points.push_back(makeGridWaypoint(path[i].first, path[i].second));
        }
        return msg;
    }

    void noFlyCallback(const gs_msgs::NoFlyCells::ConstPtr& msg)
    {
        const auto t0 = std::chrono::steady_clock::now();

        ROS_INFO("[planner_node] receive /ui/no_fly_cells, count=%lu",
                 static_cast<unsigned long>(msg->cells.size()));

        const Grid grid = parseBlockedCells(msg);
        const std::vector<Cell> path = generatePath(grid);
        const gs_msgs::WaypointArray path_msg = toWaypointArray(path);

        const auto t1 = std::chrono::steady_clock::now();
        const double plan_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();

        ROS_INFO("[planner_node] publish /planner/path, size=%lu, plan_time_ms=%.3f",
                 static_cast<unsigned long>(path_msg.points.size()),
                 plan_ms);

        pub_.publish(path_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    PlannerNode node;
    ros::spin();
    return 0;
}