#ifndef SIMPLEPATHGENERATOR_H
#define SIMPLEPATHGENERATOR_H

#include <QObject>
#include <QList>
#include <QPoint>
#include <QSet>
#include <QMessageBox>
#include<QThread>
#include<QElapsedTimer>
#include<QStack>
#include <algorithm>
#include<QDebug>
#include<QHash>
#include <climits>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
inline uint qHash(const QPoint &point, uint seed = 0)
{
    return qHash(qMakePair(point.x(), point.y()), seed);
}
#endif

class SimplePathGenerator : public QObject
{
    Q_OBJECT
public:
    explicit SimplePathGenerator(QObject *parent = nullptr);

public:

    // 输入三个连续相邻的障碍点（列,行），返回从(9,7)出发的遍历路径（列,行），失败返回空列表
    QList<QPoint> generate(const QList<QPoint>& obstacles)
    {
        qDebug()<<QThread::currentThread();
        // 检查每个障碍点是否在网格范围内（列1-9，行1-7）
        for (const QPoint& p : obstacles) {
            int col = p.x();   // 列
            int row = p.y();   // 行
            qDebug()<<col<<","<<row;
            // if(col<1|| col > 9 || row < 1 || row > 7)
            //     return {};     // 非法坐标，直接返回空
        }
        // 2. 将障碍点从(列,行)转换为内部使用的(行,列)
        QList<QPoint> obstaclesRC; // (行,列)
        for (const QPoint& p : obstacles) {
            obstaclesRC.append(QPoint(p.y(), p.x())); // (列,行) -> (行,列)
        }
        // 4. 起点(9,7)转换为内部(行,列)为(7,9)
        QPoint startRC(7, 9); // (行,列)
        // 5. 将障碍转为快速查找集合（内部坐标）
        QSet<QPoint> blocked;
        for (int i=0;i<obstaclesRC.size();i++)
            blocked.insert(obstaclesRC[i]);
        // 6. 访问标记数组（行1~7，列1~9）
        bool visited[8][10] = {{false}};   // 多开一点，避免越界
        for (const QPoint& b : blocked)
            visited[b.x()][b.y()] = true; // b.x()是行，b.y()是列
        // 7. 需要走的格子总数 = 7*9 - 障碍数
        int totalSteps = 7 * 9 - blocked.size();

        // 8. 开始DFS搜索（内部坐标）
        QList<QPoint> pathRC; // (行,列)
        pathRC.append(startRC);
        visited[startRC.x()][startRC.y()] = true;
        timer.start();
        QList<QPoint> result;
        if (dfs(startRC, pathRC, visited, totalSteps - 1)) {
            // 将路径从(行,列)转换回(列,行)
            for (int i=0;i<pathRC.size();i++) {
                result.append(QPoint(pathRC[i].y(), pathRC[i].x()));
                // (行,列) -> (列,行)
            }
            emit sendblock(obstacles);
        } else {
            QList<QPoint> bestPath;
            int bestLength = INT_MAX;
            for (int i = 0; i < 100; ++i) {
                QList<QPoint> path = findPathWithRepeat(QPoint(9,7), obstacles);
                if (!path.isEmpty() && path.size() < bestLength) {
                    bestLength = path.size();
                    bestPath = path;
                }
            }
            result=bestPath;
            emit sendblock(obstacles);
        }
        result=simplifyPath(result);
        return result;
    }
    QElapsedTimer timer;
    bool timeout = false;
    // 获取当前格子的上下左右邻居（内部坐标(行,列)，需在网格范围内）
    QList<QPoint> getNeighbors(const QPoint& p)
    {
        QList<QPoint> neighbors;
        int r = p.x(), c = p.y(); // 行,列
        if (r > 1) neighbors.append(QPoint(r-1, c));
        if (r < 7) neighbors.append(QPoint(r+1, c));
        if (c > 1) neighbors.append(QPoint(r, c-1));
        if (c < 9) neighbors.append(QPoint(r, c+1));
        return neighbors;
    }

    // 深度优先搜索回溯（内部坐标(行,列)）
    bool dfs(const QPoint& current, QList<QPoint>& path,bool visited[8][10], int remainSteps)
    {
        if (timeout) return false;   // 已超时，放弃搜索
        if (timer.elapsed() > 800) {   // 检查是否超时
            timeout = true;
            return false;
        }
        if (remainSteps == 0)
            return true;   // 所有格子走完
        for (const QPoint& nb : getNeighbors(current)) {
            if (!visited[nb.x()][nb.y()]) {
                visited[nb.x()][nb.y()] = true;
                path.append(nb);
                if (dfs(nb, path, visited, remainSteps - 1))
                    return true;
                // 回溯
                path.removeLast();
                visited[nb.x()][nb.y()] = false;
            }
        }
        return false;
    }

    int manhattan(const QPoint& a, const QPoint& b)
    {
        return abs(a.x() - b.x()) + abs(a.y() - b.y());
    }

    QList<QPoint> twoOpt(const QList<QPoint>& path)
    {
        QList<QPoint> best = path;
        bool improved = true;
        while (improved) {
            improved = false;
            for (int i = 0; i < best.size() - 2; ++i) {
                for (int j = i + 2; j < best.size() - 1; ++j) {
                    int oldDist = manhattan(best[i], best[i+1]) + manhattan(best[j], best[j+1]);
                    int newDist = manhattan(best[i], best[j]) + manhattan(best[i+1], best[j+1]);
                    if (newDist < oldDist) {
                        std::reverse(best.begin() + i + 1, best.begin() + j + 1);
                        improved = true;
                    }
                }
            }
        }
        return best;
    }
    QList<QPoint> findPathWithRepeat(const QPoint& start, const QList<QPoint>& obstacles)
    {
        // 转换为内部坐标 (行,列)
        QPoint startRC(start.y(), start.x());
        QSet<QPoint> blocked;
        for (const QPoint& p : obstacles) {
            blocked.insert(QPoint(p.y(), p.x())); // (列,行) -> (行,列)
        }

        QList<QPoint> pathRC; // 内部路径
        QSet<QPoint> visited;
        QStack<QPoint> stack;

        int totalCells = 7 * 9 - blocked.size();
        if (totalCells == 0) return {};

        pathRC.append(startRC);
        visited.insert(startRC);
        stack.push(startRC);

        while (visited.size() < totalCells) {
            QPoint current = stack.top();
            QList<QPoint> neighbors = getNeighbors(current);
            QList<QPoint> candidates;

            // 收集未访问的邻居
            for (const QPoint& nb : neighbors) {
                if (!blocked.contains(nb) && !visited.contains(nb))
                    candidates.append(nb);
            }

            if (!candidates.isEmpty()) {
                // 二级启发式排序：先按一级未访问邻居数，再按二级未访问邻居数
                std::sort(candidates.begin(), candidates.end(),
                          [&](const QPoint& a, const QPoint& b) {
                              // 一级度数
                              int aCnt1 = 0, bCnt1 = 0;
                              for (const QPoint& aa : getNeighbors(a))
                                  if (!blocked.contains(aa) && !visited.contains(aa)) aCnt1++;
                              for (const QPoint& bb : getNeighbors(b))
                                  if (!blocked.contains(bb) && !visited.contains(bb)) bCnt1++;
                              if (aCnt1 != bCnt1) return aCnt1 < bCnt1;

                              // 二级度数（排除自身）
                              int aCnt2 = 0, bCnt2 = 0;
                              for (const QPoint& aa : getNeighbors(a)) {
                                  if (!blocked.contains(aa) && !visited.contains(aa)) {
                                      for (const QPoint& aaa : getNeighbors(aa))
                                          if (!blocked.contains(aaa) && !visited.contains(aaa) && aaa != a)
                                              aCnt2++;
                                  }
                              }
                              for (const QPoint& bb : getNeighbors(b)) {
                                  if (!blocked.contains(bb) && !visited.contains(bb)) {
                                      for (const QPoint& bbb : getNeighbors(bb))
                                          if (!blocked.contains(bbb) && !visited.contains(bbb) && bbb != b)
                                              bCnt2++;
                                  }
                              }
                              return aCnt2 < bCnt2;
                          });

                // 找出所有与第一个候选具有相同启发式值（cnt1, cnt2）的候选（即最佳候选集）
                // 计算第一个候选的 cnt1, cnt2
                int bestCnt1 = 0, bestCnt2 = 0;
                for (const QPoint& aa : getNeighbors(candidates.first()))
                    if (!blocked.contains(aa) && !visited.contains(aa)) bestCnt1++;
                for (const QPoint& aa : getNeighbors(candidates.first())) {
                    if (!blocked.contains(aa) && !visited.contains(aa)) {
                        for (const QPoint& aaa : getNeighbors(aa))
                            if (!blocked.contains(aaa) && !visited.contains(aaa) && aaa != candidates.first())
                                bestCnt2++;
                    }
                }

                QList<QPoint> bestCandidates;
                for (const QPoint& cand : candidates) {
                    int cnt1 = 0, cnt2 = 0;
                    for (const QPoint& aa : getNeighbors(cand))
                        if (!blocked.contains(aa) && !visited.contains(aa)) cnt1++;
                    for (const QPoint& aa : getNeighbors(cand)) {
                        if (!blocked.contains(aa) && !visited.contains(aa)) {
                            for (const QPoint& aaa : getNeighbors(aa))
                                if (!blocked.contains(aaa) && !visited.contains(aaa) && aaa != cand)
                                    cnt2++;
                        }
                    }
                    if (cnt1 == bestCnt1 && cnt2 == bestCnt2)
                        bestCandidates.append(cand);
                }

                // 从最佳候选中随机选择一个
                QPoint next = bestCandidates[rand() % bestCandidates.size()];
                pathRC.append(next);
                visited.insert(next);
                stack.push(next);
            } else {
                // 没有未访问邻居，回溯
                stack.pop();
                if (!stack.isEmpty()) {
                    QPoint back = stack.top();
                    pathRC.append(back);
                } else {
                    // 栈空但未访问完，说明图不连通
                    return {};
                }
            }
        }

        // 2-opt 优化
        pathRC = twoOpt(pathRC);

        // 转换回外部坐标 (列,行)
        QList<QPoint> result;
        for (const QPoint& p : pathRC) {
            result.append(QPoint(p.y(), p.x()));
        }
        return result;
    }
    QList<QPoint> simplifyPath(const QList<QPoint>& path)
    {
        if (path.size() <= 2)
            return path;
        QList<QPoint> simplified;
        simplified.append(path.first());  // 起点保留

        for (int i = 1; i < path.size() - 1; ++i) {
            const QPoint& prev = path[i-1];
            const QPoint& curr = path[i];
            const QPoint& next = path[i+1];

            // 计算方向向量
            int dx1 = curr.x() - prev.x();
            int dy1 = curr.y() - prev.y();
            int dx2 = next.x() - curr.x();
            int dy2 = next.y() - curr.y();

            // 如果前后两段方向相同，则当前点是直线上的中间点，可以跳过
            if (dx1 == dx2 && dy1 == dy2)
                continue;

            simplified.append(curr);
        }

        simplified.append(path.last());   // 终点保留
        return simplified;
    }
public slots:
    void run(QList<QPoint> block);
signals:
    void sendPath(QList<QPoint> path);
    void sendblock(QList<QPoint> block);
};

#endif // SIMPLEPATHGENERATOR_H
