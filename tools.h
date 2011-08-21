#ifndef TOOLS_H
#define TOOLS_H

#include <QtGlobal>
#include <QPointF>

uint qHash(const QPointF &p);

qreal distance(const QPointF &a, const QPointF &b);

qreal PI();

qreal rad2degr(qreal);
qreal degr2rad(qreal);

bool fitsInPie(const QPointF &p, const QPointF &centre, qreal radius, qreal dirAngle, qreal spanAngle);// checks if p fits in a pie sector.

#endif // TOOLS_H
