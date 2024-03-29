#ifndef BOT_H
#define BOT_H

#include <QtGui>

class Bot: public QObject
{
    Q_OBJECT

public:
    Bot(qreal moveSpeed__, qreal rotSpeed__, // in degrees
        qreal fovDist__, qreal fovAngle__,// in degrees
        QPointF startPos__, qreal startAngle__,// in degrees
        int width__, int height__,
        QVector<QVector<QPointF> > map__,
        int id__);
    int id() const;
    QPointF pos() const;
    qreal angle() const;
    qreal fovAngle() const;
    qreal fovDistance() const;
    int role() const;
    QImage image() const;
    bool wallBetween(const QPointF &a, const QPointF &b) const;

signals:
    void queryBots(Bot *bot, QVector<Bot *> *result);
    void queryKeys(QHash<int, bool> *result);
    void botKilled(int id__);

public slots:
    void makeMove();
    void toggleControl();
    void toggleRole();

private:

#ifdef DEBUG
public:
#endif
    void updateEnemies();

    void makeAIMove();// Follows the path in the "path" variable
    void makeManualMove();// Queries the pressed keys, and tries to move. If collides a wall, tries to rotate.

    bool makeMoveByLine(const QPointF &a, const QPointF &b);// helper method for makeAIMove. Rotates while curAngle isn't equal to
                                                            // Line(a, b).angle, then follows this line.
    QPointF getAITarget() const;// Finds the point with the hightest potential.

    bool exploreMap();// Updates the "isExplored" variable. Returns true if finds a new point
    QVector<QVector<int> > determineConnComp() const;// A helper function for updateVirtualWalls

    void updateVirtualWalls();
    void updatePotential();
    void killkillkill();

    QPair<QPointF, QPointF> getVertexPivots(const QPointF &a, const QPointF &b, const QPointF &c) const;// Calculates the pivots for line [a, b][b, c]. The first element in the return value will always be the "inner" point
    QVector<QPointF> getPivots(const QVector<QPointF> &, bool mapPivots_ = false) const;// Calculates pivots for the polyline(might be enclosed). Additional parameter is for correct handling of map pivots generating.
    QHash<QPointF, QVector<QPointF> > getGraph(const QPointF &startPos, const QPointF &targetPos_) const;// Calculates the visibility graph and returns it
    QVector<QPointF> getPath(const QPointF &startPos, const QPointF &targetPos_) const;// Calculates the path and returns it

    void addVisitsCount(const QPointF &p, qreal value = 20.0);//Adds visits count to the point and its neighbours(affection radius is set in the method).

    int fieldWidth_, fieldHeight_;
    qreal moveSpeed, rotSpeed;// rotSpeed is in radians
    qreal fovDist_, fovAngle_;// FOV(field of view) is a circle sector with the radius fovDist and the central angle fovAngle(in radians)
    QPointF curPos_;
    qreal curAngle_;
    int id_;
    qreal pivotOffset_; // A parameter for conflicts exclusion. Path should be binded not to polygonal chains' vertices, but to the nearby located point, soThis parameter sets there points' offset from the vertices.
    qreal cellSize_;// The discovered zones edges are being drawn as circles, so this parameter affects "smoothing". Also, it significantly affects the perfomance.

    enum Control
    {
        ManualContol,
        AIControl
    };
    Control control_;

    enum ExplorationState
    {
        NoState,
        FollowPathState,
        PointExploreStateCW,// starts rotating and tries to stop with the given probality each moment(I consider it as a dirty hack, but don't see another sufficient method to handle it)
        PointExploreStateCCW //the same, but counter-clockwise
    };
    ExplorationState state_;
    enum Role
    {
        Hunter,
        Runner
    };
    Role role_;


    QHash<Bot *, int> enemiesVisible_;

    QVector<QVector<qreal> > potential_;// The potential heuristic is formed by the nearby located undiscovered point(they increase it) and by the nearby located points' visits(they decrease it).
    QVector<QVector<int> > visits_;// Not exactly the visits count, but comparatively to other points, it's the time the bot was close to the point.

    QVector<QVector<QPointF > > map_;// Contains just the map, shoudn't be changed during the visualisation. Changed once in the constructor to add the screen edges.
    QVector<QPointF> mapPivots_;// Initialized at the startup, for the better perfomance.
    QVector<QVector<bool> > isDiscovered_;// The Visualisation field is a grid, so some points of this grid are already discovered, some not
                                         // To convert grid nodes into real coordinates, you'll just multiply it by cellSize.
    QVector<QPointF> path_;// Contains the path to targetPos

    QPointF targetPos_;// Program will follow the path to this point

    QVector<QVector<QPointF> > virtualWalls_;// These walls are formed by the edges of the undiscovered zone.
                                            // They are "virtual" because they don't physically exist(unlike the walls formed by the "map" variable, their purpose is limitation for the path search algorithm
#ifdef DEBUG
    mutable QVector<QPointF> dbgPivots;
    mutable QVector<QPointF> dbgConnComp;
    mutable QVector<QVector<int> > dbgCompNumber;
#endif

};


#endif // BOT_H
