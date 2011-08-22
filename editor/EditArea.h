#ifndef EDITAREA_H
#define EDITAREA_H

#include <QtGui>
#include <QtGlobal>

class EditArea: public QWidget
{
    Q_OBJECT

public: 
    EditArea(QWidget *parent = NULL);

    void setMap(const QVector<QVector<QPointF> > &);
    QVector<QVector<QPointF> > getMap() const;

public slots:
    void setSnapRadius(int);

signals:
    void mapChanged();

private:
    void paintEvent(QPaintEvent *);
    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);

    void snapPoints(const QPointF &, const QPointF &);
    bool canSnap(const QPointF &, const QPointF &) const;

    QVector<QVector<QPointF> > map;
    QPointF startPos, lastPos;// Set the "edit line".
    qreal snapRadius;// Consider we're drawing a polyline and already have some its part drawn.
                     // To continue drawing it we would need to start it _exactly_ form the last point,
                     // and that's quite hard. However, we have a "snap zone" - so you only need to make
                     // sure the end point is in the snap zone, and the line will be continued.
                     // To enclose the polyline to get a polygone, you need to ensure both
                     // start and finish points of the "edit line" are in the polyline ends' snap zone.
    enum EditMode
    {
        noMode,
        drawMode,
        deleteMode
    };
    EditMode mode;
};

#endif
