#include "tools.h"


#include <QPair>
#include <QHash>
#include <QtCore/qmath.h>
#include <QLineF>

uint qHash(const QPointF &p)
{
    return qHash(QPair<qint64, qint64>(p.x(), p.y()));
}

qreal distance(const QPointF &a, const QPointF &b)
{
    QPointF c = b - a;
    return qSqrt(c.x() * c.x() + c.y() * c.y());
}

qreal PI()
{
    return 4 * qAtan(1);
}

qreal rad2degr(qreal rad)
{
    return rad / PI() * 180;
}

qreal degr2rad(qreal degr)
{
    return degr / 180 * PI();
}

bool fitsInPie(const QPointF &p, const QPointF &centre, qreal radius, qreal dirAngle, qreal spanAngle)
{
    dirAngle = fmod(dirAngle, PI() * 2);

    qreal st = dirAngle - spanAngle / 2;
    qreal fn = dirAngle + spanAngle / 2;

    st = fmod(st, PI() * 2);
    if (st < 0)
        st += 2 * PI();
    fn = fmod(fn, PI() * 2);
    if (fn < 0)
        fn += 2 * PI();
    if (st > fn)
        fn += 2 * PI();
    qreal angle = degr2rad(QLineF(centre, p).angle());

    if (angle < 0)
        angle += 2 * PI();
    if (angle < st)
        angle += 2 * PI();
#ifdef DEBUG
//    qDebug() << "Start angle " << rad2degr(st) << " Finish angle " << rad2degr(fn) << " Anlge " << rad2degr(angle) << endl;
#endif

    return (distance(p, centre) < radius && st <= angle && angle <= fn);
}
