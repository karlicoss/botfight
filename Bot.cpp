#include <QtGui>

#include <queue>
#include <cmath>
#include <algorithm>

#include "Bot.h"
#include "tools.h"

Bot::Bot(qreal moveSpeed__, qreal rotSpeed__, // in degrees
         qreal fovDist__, qreal fovAngle__,// in degrees
         QPointF startPos__, qreal startAngle__,// in degrees
         int width__, int height__,
         QVector<QVector<QPointF> > map__,
         int id__):
    fieldWidth_(width__), fieldHeight_(height__),
    moveSpeed(moveSpeed__), rotSpeed(degr2rad(rotSpeed__)),
    fovDist_(fovDist__), fovAngle_(degr2rad(fovAngle__)),
    curPos_(startPos__), curAngle_(degr2rad(startAngle__)),
    id_(id__),
    pivotOffset_(8.0), cellSize_(8.0),
    control_(AIControl),
    state_(NoState),
    role_(Runner),
    map_(map__),
    targetPos_(curPos_)
{
    qsrand(10);

    int cellsx = width__ / cellSize_ + 1;
    int cellsy = height__ / cellSize_ + 1;
    isDiscovered_ = QVector<QVector<bool> > (cellsx, QVector<bool> (cellsy, false));
    int curcellx = curPos_.x() / cellSize_;
    int curcelly = curPos_.y() / cellSize_;
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            if (curcellx + i >= 0 && curcellx + i < cellsx &&
                curcelly + j >= 0 && curcelly + j < cellsy)
            {
                isDiscovered_[curcellx + i][curcelly + j] = true;
            }
        }
    }

    visits_ = QVector<QVector<int> > (cellsx, QVector<int> (cellsy, 0));
    potential_ = QVector<QVector<qreal> > (cellsx, QVector<qreal> (cellsy, 0.0));

    QPointF p00 = QPointF(0, 0), p10 = QPointF(width__ - 1, 0), p01 = QPointF(0, height__ - 1), p11 = QPointF(width__ - 1, height__ - 1);
    QVector<QPointF> edge;
    edge.append(p00);
    edge.append(p01);
    edge.append(p11);
    edge.append(p10);
    edge.append(p00);
    map_.append(edge);

    for (int i = 0; i < map_.size(); i++)
        mapPivots_ += getPivots(map_[i], true);
}

QPair<QPointF, QPointF> Bot::getVertexPivots(const QPointF &a, const QPointF &b, const QPointF &c) const
{
    QLineF dir1 = QLineF(b, a).unitVector(),
           dir2 = QLineF(b, c).unitVector();
    QLineF helper = QLineF(a, c).normalVector();

    QPointF med = (dir1.p2() + dir2.p2()) / 2.0;
    QLineF dir = QLineF(b, med);
    dir.setLength(pivotOffset_);
    QPointF d1 = dir.p2(), d2 = dir.p1() - (dir.p2() - dir.p1());

    int angle = helper.angleTo(dir);
    if (90 <= angle && angle < 270) //First point will always be in the inner side
        return qMakePair(d2, d1);
    else
        return qMakePair(d1, d2);
}


QVector<QPointF> Bot::getPivots(const QVector<QPointF> &v, bool mapPivots) const
{
    bool innerZone = isDiscovered_[v[0].x() / cellSize_][v[0].y() / cellSize_];
    QVector<QPointF> ans;
    if (v.size() == 1)
    {
        ans.append(v.front() - QPointF(pivotOffset_, 0));
        ans.append(v.front() - QPointF(0, pivotOffset_));
        ans.append(v.front() + QPointF(pivotOffset_, 0));
        ans.append(v.front() + QPointF(0, pivotOffset_));
        return ans;
    }
    if (v.front() == v.back() && v.size() > 2) // the line is enclosed
    {
        QPair<QPointF, QPointF> pv = getVertexPivots(v[v.size() - 2], v[0], v[1]);
        ans.append(pv.first);
        ans.append(pv.second);
    }
    else
    {
        // The line is not enclosed, so we'll use two points on its extension to the first and the last vertices.
        QLineF dir1 = QLineF(v[0], v[1]),
               dir2 = QLineF(v[v.size() - 1], v[v.size() - 2]);
        dir1.setLength(pivotOffset_);
        dir2.setLength(pivotOffset_);
        ans.push_back(dir1.p1() - (dir1.p2() - dir1.p1()));
        ans.push_back(dir2.p1() - (dir2.p2() - dir2.p1()));

        // And four on the perpendiculars to the first and the last segment(extended to both sides).
        QLineF norm1 = QLineF(v[0], v[1]).normalVector(),
               norm2 = QLineF(v[v.size() - 1], v[v.size() - 2]).normalVector();
        norm1.setLength(pivotOffset_);
        norm2.setLength(pivotOffset_);
        ans.push_back(norm1.p2());
        ans.push_back(norm1.p1() - (norm1.p2() - norm1.p1()));
        ans.push_back(norm2.p2());
        ans.push_back(norm2.p1() - (norm2.p2() - norm2.p1()));
    }

    for (int i = 1; i < v.size() - 1; i++)
    {
        QPair<QPointF, QPointF> pv = getVertexPivots(v[i - 1], v[i], v[i + 1]);
        if (mapPivots)
        {
            ans.append(pv.first);
            ans.append(pv.second);
        }
        else
        {
            if (innerZone)
                ans.append(pv.first);
            else
                ans.append(pv.second);
        }
    }

    return ans;
}

namespace
{

void bfs(QVector<QVector<int> > *v, int stx, int sty, int cid) // Helper function for connectivity components determining. cid = component id.
{
    QVector<QVector<int> > &lv = *v;

    QQueue<QPair<int, int> > q;
    q.enqueue(qMakePair(stx, sty));
    lv[stx][sty] = cid;
    while (!q.isEmpty())
    {
        QPair<int, int> top = q.head();
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if ((top.first + i >= 0) && (top.first + i < lv.size()) &&
                    (top.second + j >= 0) && (top.second + j < lv[0].size()) &&
                    lv[top.first + i][top.second + j] == 0)
                {
                    q.enqueue(qMakePair(top.first + i, top.second + j));
                    lv[top.first + i][top.second + j] = cid;
                }
            }
        }
        q.dequeue();
    }
}
}

QVector<QVector<int> > Bot::determineConnComp() const
{
    QVector<QVector<int> > isVisited(isDiscovered_.size(), QVector<int>(isDiscovered_[0].size(), 0));
    for (int i = 0; i < isVisited.size(); i++)// First, we marks _discovered_ nodes as "walls"
    {
        for (int j = 0; j < isVisited[0].size(); j++)
        {
            if (isDiscovered_[i][j])
                isVisited[i][j] = 1;
        }
    }
    int compCount = 1;
    for (int i = 0; i < isVisited.size(); i++)// Searching for the connected components
    {
        for (int j = 0; j < isVisited[0].size(); j++)
        {
            if (isVisited[i][j] == 0)
            {
                bfs(&isVisited, i, j, compCount + 1);// components ids are 1-indexed
                compCount += 1;
            }
        }
    }
#ifdef DEBUG
    //qDebug() << isVisited << endl;
    dbgCompNumber = isVisited;
#endif
    return isVisited;
}

namespace
{

QVector<QPointF> optimizeWall(const QVector<QPointF> &w)// Optimising walls is merging some its consequent segments into one. For example:
                                                        // {..(20, 100),(20, 200),(20, 300)..} might be merged into {..(20, 100); (20, 300)..}
                                                        // {..(20, 100),(20, 200),(21, 300)..} can't be merged because the second segment is not on the first segment's line.
{
    QVector<QPointF> ans;
    if (w.size() >= 2)
    {
        ans.append(w[0]);
        ans.append(w[1]);
    }
    else
    {
        return w;
    }
    qreal eps = 0.001;
    for (int i = 2; i < w.size(); i++)
    {
        QLineF l1(ans[ans.size() - 2], ans[ans.size() - 1]);
        QLineF l2(ans[ans.size() - 1], w[i]);
        if (qAbs(l1.angle() - l2.angle()) < eps)
        {
            ans.pop_back();
        }
        ans.append(w[i]);
    }
    return ans;
}

}

void Bot::updateVirtualWalls()
{
    virtualWalls_.clear();

    QVector<QVector<int> > connComp = determineConnComp();

#ifdef DEBUG
    dbgCompNumber = connComp;
#endif

    int curComp = 1;
    while (true)
    {
        QVector<QPointF> result;
        int startx = -1, starty = -1;// Searching for the leftmost topmost point.
        for (int j = 0; j < connComp[0].size() && starty == -1; j++)
        {
            for (int i = 0; i < connComp.size() && startx == -1; i++)
            {
                if (connComp[i][j] == curComp)
                {
                    startx = i;
                    starty = j;
                    break;
                }
            }
        }
        if (startx == -1 && starty == -1)
            break;
        int curx = startx, cury = starty;

        int dx[] = {-1, -1, 0, 1, 1,  1,  0, -1};
        int dy[] = {0 ,  1, 1, 1, 0, -1, -1, -1};
        int curDir = 4;

        bool startVisited = false;
        while (!(startVisited && curx == startx && cury == starty))
        {
            if (curx == startx && cury == starty)
                startVisited = true;
            if (result.size() > 400)
            {
//                qDebug() << "oops";
            }
            result.append(cellSize_ * QPointF(curx, cury));

            int c = (curDir + 4) % 8;
            bool foundNew = false;
            for (int i = c + 1; i < 8 && !foundNew; i++)
            {
                int nx = curx + dx[i];
                int ny = cury + dy[i];
                if (nx >= 0 && nx < connComp.size() &&
                    ny >= 0 && ny < connComp[0].size() &&
                    (connComp[nx][ny] == curComp))
                {
                    curx = nx;
                    cury = ny;
                    foundNew = true;
                    curDir = i;
                }
            }
            for (int i = 0; i <= c && !foundNew; i++)
            {
                int nx = curx + dx[i];
                int ny = cury + dy[i];

                if (nx >= 0 && nx < connComp.size() &&
                    ny >= 0 && ny < connComp[0].size() &&
                    (connComp[nx][ny] == curComp))
                {
                    curx = nx;
                    cury = ny;
                    foundNew = true;
                    curDir = i;
                }
            }
            if (!foundNew)
            {
                break;
            }
        }
        if (result.size() >= 2)
        {
            result.append(result.front());
        }

        QVector<QPointF> optResult = optimizeWall(result);
#ifdef DEBUG
//        qDebug() << "Result was " << result.size() << " optimized to " << optResult.size() << endl;
#endif
        virtualWalls_.append(optResult);
        curComp++;
    }
}

QHash<QPointF, QVector<QPointF> > Bot::getGraph(const QPointF &startPos, const QPointF &targetPos) const
{
    QVector<QPointF> points;

    points += mapPivots_;

    for (int i = 0; i < virtualWalls_.size(); i++)
        points += getPivots(virtualWalls_[i]);

    points.push_back(startPos);
    points.push_back(targetPos);

#ifdef DEBUG
dbgPivots = points;
#endif

    QVector<QLineF> lines;
    for (int i = 0; i < map_.size(); i++)
    {
        for (int j = 0; j < map_[i].size() - 1; j++)
        {
            lines.push_back(QLineF(map_[i][j], map_[i][j + 1]));
        }
    }
#ifdef DEBUG
//    qDebug() << "There are " << lines.size() << " solid and " << virtualWalls.size() << " virtual walls ,and " << points.size() << "points" << endl;
#endif
    QHash<QPointF, QVector<QPointF> > graph;//visibility graph
    QPointF trash;
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = i + 1; j < points.size(); j++)
        {
            QLineF line(points[i], points[j]);
            bool wasIntersection = false;
            for (int l = 0; l < lines.size() && !wasIntersection; l++)
            {
                if (line.p1() != lines[l].p1() && line.p2() != lines[l].p1() && line.p1() != lines[l].p2() && line.p2() != lines[l].p2())//то есть линии не смежные
                    if (line.intersect(lines[l], &trash) == QLineF::BoundedIntersection)
                        wasIntersection = true;
            }
            for (int l = 0; l < virtualWalls_.size() && !wasIntersection; l++)
            {
                for (int e = 0; e < virtualWalls_[l].size() - 1 && !wasIntersection; e++)
                {
                    QLineF tline(virtualWalls_[l][e], virtualWalls_[l][e + 1]);
                    if (line.intersect(tline, &trash) == QLineF::BoundedIntersection)
                        wasIntersection = true;
                }
            }
            if (!wasIntersection)
            {
                graph[points[i]].push_back(points[j]);
                graph[points[j]].push_back(points[i]);
            }
        }
    }
    return graph;
}

QVector<QPointF> Bot::getPath(const QPointF &startPos, const QPointF &targetPos) const
{
    QHash<QPointF, QVector<QPointF> > graph = getGraph(startPos, targetPos);

    QVector<QPointF> ans;

    QSet<QPointF> closed, open;
    QHash<QPointF, double> g_h, h_h;
    QHash<QPointF, QPointF> parent;

    open.insert(startPos);
    g_h[startPos] = 0;
    h_h[startPos] = distance(startPos, targetPos);

    while (!open.isEmpty())
    {
        QPointF top = *open.begin();
        for (QSet<QPointF>::ConstIterator it = open.begin(); it != open.end(); ++it)
        {
            if (g_h[*it] + h_h[*it] < g_h[top] + h_h[top])
                top = *it;
        }
        if (top == targetPos)
        {
            QPointF cur = targetPos;
            while (cur != startPos)
            {
                ans.append(cur);
                cur = parent[cur];
            }
            ans.append(startPos);
            std::reverse(ans.begin(), ans.end());
            break;
        }

        open.remove(top);
        closed.insert(top);
        QVector<QPointF> l = graph[top];

        for (int i = 0; i < l.size(); i++)
        {
            if (closed.contains(l[i]))
                continue;

            double new_g = g_h[top] + distance(top, l[i]);
            bool upd;
            if (!open.contains(l[i]))
            {
                open.insert(l[i]);
                upd = true;
            }
            else
            {
                upd = new_g < g_h[l[i]];
            }
            if (upd)
            {
                parent[l[i]] = top;
                g_h[l[i]] = new_g;
                h_h[l[i]] = distance(l[i], targetPos);
            }
        }
    }
    return ans;
}

bool Bot::exploreMap()
{
    bool discovered = false;

    qreal stx = curPos_.x() - fovDist_, fnx = curPos_.x() + fovDist_;
    qreal sty = curPos_.y() - fovDist_, fny = curPos_.y() + fovDist_;
    int stxp = qMax(0.0, stx / cellSize_), fnxp = qMin(qreal(isDiscovered_.size() - 1), fnx / cellSize_);
    int styp = qMax(0.0, sty / cellSize_), fnyp = qMin(qreal(isDiscovered_[0].size() - 1), fny / cellSize_);
    for (int i = stxp; i <= fnxp; i++)
    {
        for (int j = styp; j <= fnyp; j++)
        {
            if (!isDiscovered_[i][j] && fitsInPie(cellSize_ * QPointF(i, j), curPos_, fovDist_, curAngle_, fovAngle_))
            {
                if (!wallBetween(curPos_, cellSize_ * QPointF(i, j)))
                {
                    isDiscovered_[i][j] = true;
                    discovered = true;
                }
            }
        }
    }
    updateVirtualWalls();
    return discovered;
}

void Bot::makeManualMove()
{
    QHash<int, bool> pressed;
    queryKeys(&pressed);
    if (pressed[Qt::Key_Left])
    {
        curAngle_ += rotSpeed;
    }
    if (pressed[Qt::Key_Right])
    {
        curAngle_ -= rotSpeed;
    }
    if (pressed[Qt::Key_Up])
    {
        QLineF dir = QLineF::fromPolar(moveSpeed, rad2degr(curAngle_)).translated(curPos_);
        QPointF trash;
        bool intersects = false;
        for (int i = 0; i < map_.size() && !intersects; i++)
        {
            for (int j = 0; j < map_[i].size() - 1 && !intersects; j++)
            {
                QLineF tmp(map_[i][j], map_[i][j + 1]);
                if (dir.intersect(tmp, &trash) == QLineF::BoundedIntersection)
                {
                    intersects = true;

                    qreal prod = dir.dx() * tmp.dx() + dir.dy() * tmp.dy();
                    int angle = dir.angleTo(tmp);
                    if ((0 <= angle && angle <= 180) ^ (prod > 0))
                        curAngle_ -= rotSpeed;
                    else
                        curAngle_ += rotSpeed;
                }
            }
        }
        if (!intersects)
        {
            curPos_ += QLineF::fromPolar(moveSpeed, rad2degr(curAngle_)).p2();
        }
    }
}

void Bot::updatePotential()
{
    double minPotential = -5000.0; // potential for undiscovered points
    potential_.fill(QVector<double> (potential_[0].size(), minPotential));

    int randomAffectionRadius = fovDist_ / 2.0; // Some points will be chosen randomly
    int affectionRadius = fovDist_ / 5.0; // And some will be chosen definetely
    int tries = 20;
    for (int i = 0; i < potential_.size(); i++)
    {
        for (int j = 0; j < potential_[0].size(); j++)
        {
            if (!isDiscovered_[i][j])
                continue;
            if (!(isDiscovered_[i][j] &&
                i > 0 && i < potential_.size() - 1 &&
                j > 0 && j < potential_[0].size() - 1 &&
                isDiscovered_[i - 1][j] && isDiscovered_[i + 1][j] && //
                isDiscovered_[i][j - 1] && isDiscovered_[i][j + 1]))
            {
                continue;
            }
            potential_[i][j] = 0.0;
            int potRadius = affectionRadius / cellSize_;
            for (int q = - potRadius; q < potRadius; q++)
            {
                if (i + q < 0 || i + q >= potential_.size())
                    continue;
                for (int w = - potRadius; w < potRadius; w++)
                {
                    if (j + w < 0 || j + w >= potential_[0].size())
                        continue;
                    if (!isDiscovered_[i + q][j + w])
                    {
                        if (!wallBetween(cellSize_ * QPointF(q + i, w + j), cellSize_ * QPointF(i, j)))
                            potential_[i][j] += 10.0 / (qAbs(q) + 0.01) + 10.0 / (qAbs(w) + 0.01);
                    }
                }
            }

            int potRandRadius = randomAffectionRadius / cellSize_;
            for (int t = 0; t < tries; t++)
            {
                int tx = rand() % (2 * potRandRadius);
                int ty = rand() % (2 * potRandRadius);
                int cx = i - potRandRadius + tx;
                int cy = j - potRandRadius + ty;
                if (cx >= 0 && cx < potential_.size() &&
                    cy >= 0 && cy < potential_[0].size() &&
                    !isDiscovered_[cx][cy])
                {
                    if (!wallBetween(cellSize_ * QPointF(cx, cy), cellSize_ * QPointF(i, j)))
                        potential_[i][j] += 10.0 / (qAbs(i - cx) + 0.01) + 10.0 / (qAbs(j - cy) + 0.01);
                }

            }
            potential_[i][j] -= visits_[i][j];
        }
    }
}

QPointF Bot::getAITarget() const
{
    int mi = -1, mj = -1;
    // Step 1. Searching for the point with the hightst potential in the given radius.
    int searchRadius = fovDist_;
    for (int curRadius = searchRadius; curRadius <= fieldWidth_; curRadius += searchRadius)
    {
        int sx = qMax(0, int((curPos_.x() - curRadius) / cellSize_));
        int ex = qMin(potential_.size(), int((curPos_.x() + curRadius) / cellSize_));
        int sy = qMax(0, int((curPos_.y() - curRadius) / cellSize_));
        int ey = qMin(potential_[0].size(), int((curPos_.y() + curRadius) / cellSize_));

        for (int i = sx; i < ex; i++)
        {
            for (int j = sy; j < ey; j++)
            {
                if (isDiscovered_[i][j] && (mi == -1 || mj == -1 || potential_[i][j] > potential_[mi][mj]))
                {
                    if (!wallBetween(curPos_, cellSize_ * QPointF(i, j)))
                    {
                        mi = i;
                        mj = j;
                    }
                }
            }
        }
        if (potential_[mi][mj] > 0.0) // if point's potential is less than zero, this point has already been visited.
            return cellSize_ * QPointF(mi, mj);
    }
    Q_ASSERT(mi != -1);
    // Step 2. Searching for the point with the higwst potential on the full map
    for (int i = 0; i < potential_.size(); i++)
    {
        for (int j = 0; j < potential_[0].size(); j++)
        {
            if (isDiscovered_[i][j] && (mi == -1 || mj == -1 || potential_[i][j] > potential_[mi][mj]))
            {
                mi = i;
                mj = j;
            }
        }
    }

    return cellSize_ * QPointF(mi, mj);
}

void Bot::makeAIMove()
{
    if (role_ == Hunter)
    {
        for (QHash<Bot *, int>::iterator it = enemiesVisible_.begin(); it != enemiesVisible_.end(); ++it)
        {
            state_ = FollowPathState;
            if (it.key()->role() == Hunter)
            {
                path_.clear();
                path_.append(curPos_);
                path_.append(it.key()->pos());
                targetPos_ = path_[1];
                break; // Hunters have higher kill priority than Runners
            }
            else
            {
                path_.clear();
                path_.append(curPos_);
                path_.append(it.key()->pos());
                targetPos_ = path_[1];
            }
        }
    }

    if (state_ == FollowPathState)
    {
        if (path_.size() == 0) // We haven't found the path(or it is incorrect)
        {
            addVisitsCount(targetPos_, 50);// We lower the target point's potential, and its probality to be chosen again
            state_ = NoState;// Another chance
        }
        else
        {
            addVisitsCount(curPos_);
            if (path_.size() <= 1)
            {
                if (rand() % 2 == 0)
                    state_ = PointExploreStateCW;
                else
                    state_ = PointExploreStateCCW;
                return;
            }
            QPointF a = path_[0];
            QPointF b = path_[1];
            bool came = makeMoveByLine(a, b);
            if (came) // we have passed a, so we can remove it from the path
                path_.pop_front();
        }
    }
    else if (state_ == NoState)
    {
        targetPos_ = getAITarget();
        path_ = getPath(curPos_, targetPos_);
        state_ = FollowPathState;
    }
    else if (state_ == PointExploreStateCW || state_ == PointExploreStateCCW)
    {
        int stopProbality = 90;// The parameter to control rotation time.
        if (qrand() % 100 > stopProbality)
        {
            state_ = NoState;
        }
        else
        {
            if (state_ == PointExploreStateCW)
                curAngle_ -= rotSpeed;
            else
                curAngle_ += rotSpeed;
        }
    }
}

namespace
{

bool isOnSegment(QPointF p, QPointF a, QPointF b) //returns true if p belongs to segment [a, b]
{
    //segment = [t * a + (1 - t) * b | 0 <= t <= 1]
    qreal eps = 0.001;
    if (qAbs(a.x() - b.x()) < eps) // to exclude division by zero
    {
        qSwap(a.rx(), a.ry());
        qSwap(b.rx(), b.ry());
        qSwap(p.rx(), p.ry());
    }
    qreal t = (p.x() - b.x()) / (a.x() - b.x());
    if (t >= 0 && t <= 1)
    {
        if (qAbs(t * a.y() + (1 - t) * b.y() - p.y()) < 0.01)
            return true;
    }
    return false;
}

}

bool Bot::makeMoveByLine(const QPointF &a, const QPointF &b)
{
    QLineF dir(a, b);
    qreal angle = degr2rad(dir.angle());

    while (curAngle_ + 2 * PI() < 0)
        curAngle_ += 2 * PI();
    while (curAngle_ - 2 * PI() >= 0)
        curAngle_ -= 2 * PI();

    if (curAngle_ == angle)
    {
        QLineF delta = QLineF::fromPolar(moveSpeed, rad2degr(curAngle_));
        QPointF newPos = curPos_ + delta.p2();
        if (isOnSegment(newPos, a, b))
            curPos_ = newPos;
        else
        {
            curPos_ = b;
            return true;
        }
    }
    else
    {
        bool ccw;//counter-clockwise
        if (angle > curAngle_)
        {
            if (curAngle_ + PI() > angle)
                ccw = true;
            else
                ccw = false;
        }
        else
        {
            if (angle + PI() > curAngle_)
                ccw = false;
            else
                ccw = true;
        }
        if (ccw)
        {
            if (angle < curAngle_)
                angle += 2 * PI();
            if (curAngle_ + rotSpeed >= angle)
                curAngle_ = angle;
            else
                curAngle_ += rotSpeed;
        }
        else
        {
            if (curAngle_ < angle)
                curAngle_ += 2 * PI();
            if (curAngle_ - rotSpeed <= angle)
                curAngle_ = angle;
            else
                curAngle_ -= rotSpeed;
        }
    }
    return false;
}

void Bot::addVisitsCount(const QPointF &p, qreal value)
{
    int affectionRadius = 4; // in grid nodes
    int cx = p.x() / cellSize_, cy = p.y() / cellSize_;
    for (int q = -affectionRadius; q <= affectionRadius; q++)
    {
        if (cx + q < 0 || cx + q >= potential_.size())
            continue;
        for (int w = -affectionRadius; w <= affectionRadius; w++)
        {
            if (cy + w < 0 || cy + w >= potential_[0].size())
                continue;
            if (!wallBetween(p, cellSize_ * QPointF(cx + q, cy + w)))
                visits_[cx + q][cy + w] += value / (qAbs(q) + 0.1) + value / (qAbs(w) + 0.1);
        }
    }
    visits_[cx][cy] += value;
}

void Bot::toggleRole()
{
    if (role_ == Hunter)
        role_ = Runner;
    else
        role_ = Hunter;
}

void Bot::toggleControl()
{
    state_ = NoState;
    if (control_ == ManualContol)
        control_ = AIControl;
    else
        control_ = ManualContol;
}

void Bot::makeMove()
{
    exploreMap();
    updateEnemies();
    updatePotential();
    if (role_ == Hunter)
        killkillkill();
    if (control_ == ManualContol)
        makeManualMove();
    else
        makeAIMove();
}

QPointF Bot::pos() const
{
    return curPos_;
}

qreal Bot::angle() const
{
    return curAngle_;
}

void Bot::killkillkill()
{
    int killMoves = 50;// If target in the field of killMoves moves, it's dead
    qreal instantKillRadius = 2 * cellSize_;
    for (QHash<Bot *, int>::iterator it = enemiesVisible_.begin(); it != enemiesVisible_.end(); ++it)
    {
        if (distance(curPos_, it.key()->pos()) < instantKillRadius)
            emit botKilled(it.key()->id());
        if (it.value() > killMoves)
            emit botKilled(it.key()->id());
    }
}

QImage Bot::image() const
{
    QImage ans(fieldWidth_, fieldHeight_, QImage::Format_ARGB32);
    ans.fill(0);
    QPainter p(&ans);

    QColor botColor = QColor::fromHsv(id_ * 20, 200, 200);
    QColor botColorT = botColor;
    botColorT.setAlpha(40);
    QPen circlePen;
    if (role_ == Hunter)
        circlePen = QPen(Qt::red, 3);
    else
        circlePen = QPen(Qt::green, 3);


    QPainterPath posMark;
    qreal markWidth = 20, markHeight = 10;
    QPolygonF triangle;
    triangle << QPointF(-markWidth / 2, markHeight / 2) << QPointF(-markWidth / 2, -markHeight / 2) << QPointF(markWidth / 2, 0);
    triangle << triangle.front();
    QMatrix rotM;
    rotM.translate(curPos_.x(), curPos_.y());
    rotM.rotate(-rad2degr(curAngle_));
    triangle = rotM.map(triangle);
    posMark.addPolygon(triangle);

    p.setPen(Qt::transparent);
    p.setBrush(Qt::transparent);
    p.drawRect(0, 0, fieldWidth_, fieldHeight_);

    p.setPen(botColorT);
    p.setBrush(botColorT);
    for (int i = 0; i < isDiscovered_.size(); i++)
    {
        for (int j = 0; j < isDiscovered_[i].size(); j++)
        {
            if (isDiscovered_[i][j])
            {
                p.drawEllipse(cellSize_ * QPointF(i, j), cellSize_, cellSize_);
            }
        }
    }

    p.setPen(botColor);
    p.setBrush(botColor);
    p.drawPath(posMark);

    p.setBrush(Qt::transparent);
    p.setPen(circlePen);
    p.drawEllipse(curPos_, 20, 20);

    p.setPen(Qt::black);
    p.setPen(Qt::black);
    p.drawText(curPos_ + QPointF(5, 5), QString::number(id_));


    p.setPen(Qt::magenta);// Drawing the FOV
    p.setBrush(Qt::transparent);
    p.drawPie(QRectF(curPos_ - QPointF(fovDist_, fovDist_), curPos_ + QPointF(fovDist_, fovDist_)), rad2degr(curAngle_ - fovAngle_ / 2) * 16, rad2degr(fovAngle_) * 16);

    p.setPen(botColor);
    p.setBrush(botColor);
    p.drawEllipse(targetPos_, 5, 5);
#ifdef DEBUGsfdf
    p.setPen(Qt::black);
    for (int i = 0; i < potential.size(); i += 4)
    {
        for (int j = 0; j < potential[i].size(); j += 2)
        {
            double mp = -10000.0;
            for (int q = i; q < i + 4 && q < potential.size(); q++)
            {
                for (int w = j; w < j + 2 && w < potential[0].size(); w++)
                {
                    mp = qMax(mp, potential[q][w]);
                }
            }
            p.drawText(cellSize * i, cellSize * j, QString::number(int(mp)));
        }
    }
#endif
    return ans;
}

qreal Bot::fovAngle() const
{
    return fovAngle_;
}

qreal Bot::fovDistance() const
{
    return fovDist_;
}

int Bot::id() const
{
    return id_;
}

int Bot::role() const
{
    return role_;
}

bool Bot::wallBetween(const QPointF &a, const QPointF &b) const
{
    QLineF line(a, b);
    QPointF trash;
    for (int i = 0; i < map_.size(); i++)
    {
        for (int j = 0; j < map_[i].size() - 1; j++)
        {
            if (QLineF(map_[i][j], map_[i][j + 1]).intersect(line, &trash) == QLineF::BoundedIntersection)
            {
                return true;
            }
        }
    }
    return false;
}

void Bot::updateEnemies()
{
    QVector<Bot *> enemies;
    QHash<Bot *, int> ev;
    emit queryBots(this, &enemies);
    for (int i = 0; i < enemies.size(); i++)
        ev.insert(enemies[i], 1);
    for (QHash<Bot *, int>::iterator it = ev.begin(); it != ev.end(); ++it)
    {
        it.value() += enemiesVisible_[it.key()];
    }
    enemiesVisible_ = ev;
}





