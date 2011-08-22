#include "EditArea.h"

EditArea::EditArea(QWidget *parent):
    QWidget(parent)
{
}

void EditArea::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton)
        mode = drawMode;
    else if (e->button() == Qt::RightButton)
        mode = deleteMode;

    startPos = e->pos();
}

void EditArea::mouseReleaseEvent(QMouseEvent *)
{
    if (mode == drawMode) //drew "add line" line
    {
        snapPoints(startPos, lastPos);
    }
    else if (mode == deleteMode) //drew "delete" line
    {
        QLineF line(startPos, lastPos);
        QVector<QVector<QPointF> > newdata;
        QVector<QPointF> st;
        QPointF trash;
        for (int i = 0; i < map.size(); i++)
        {
            if (map[i].size() >= 1)
                st.push_back(map[i][0]);
            for (int j = 0; j < map[i].size() - 1; j++)
            {
                QLineF tmpline(map[i][j], map[i][j + 1]);
                if (line.intersect(tmpline, &trash) == QLineF::BoundedIntersection)
                {
                    emit mapChanged();
                    if (st.size() >= 2)
                        newdata.push_back(st);
                    st.clear();
                }
                st.push_back(map[i][j + 1]);
            }
            if (st.size() >= 2)
                newdata.push_back(st);
            st.clear();
        }
        map = newdata;
    }
    mode = noMode;
    update();
}

void EditArea::mouseMoveEvent(QMouseEvent *e)
{
    lastPos = e->pos();
    update();
}


void EditArea::paintEvent(QPaintEvent * )
{
    QPainter p(this);

    QPen mapPen;
    mapPen.setColor(QColor(45, 0, 179));
    mapPen.setWidth(3);

    p.setPen(Qt::black);
    p.setBrush(Qt::black);
    p.drawRect(QRectF(0, 0, width() - 1, height() - 1));

    p.setPen(mapPen);
    for (int i = 0; i < map.size(); i++)
        for (int j = 0; j < map[i].size() - 1; j++)
            p.drawLine(QLineF(map[i][j], map[i][j + 1]));

    p.setBrush(Qt::blue);
    for (int i = 0; i < map.size(); i++)
        for (int j = 0; j < map[i].size(); j++)
            p.drawEllipse(map[i][j], 1, 1);

    if (mode == noMode || mode == drawMode)
    {
        p.setPen(Qt::red);
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(lastPos, snapRadius, snapRadius);
    }

    if (mode == drawMode || mode == deleteMode)
    {
        if (mode == drawMode)
            p.setPen(Qt::green);
        else if (mode == deleteMode)
            p.setPen(Qt::red);
        p.drawLine(QLineF(startPos, lastPos));
    }
}


void EditArea::setSnapRadius(int r)
{
    snapRadius = r;
}

void EditArea::setMap(const QVector<QVector<QPointF> > &m)
{
    emit mapChanged();
    map = m;
    update();
}

QVector<QVector<QPointF> > EditArea::getMap() const
{
    return map;
}

bool EditArea::canSnap(const QPointF &a, const QPointF &b) const
{
    QPointF c = a - b;
    return c.x() * c.x() + c.y() * c.y() < snapRadius * snapRadius;
}

void EditArea::snapPoints(const QPointF &a, const QPointF &b)
{
    if (canSnap(a, b))
        return;

    //1. Can we enclose an existing polyline?
    for (int i = 0; i < map.size(); i++)
    {
        if (map[i].size() > 2 && // Enlosing two points looks odd.
            map[i].front() != map[i].back() && // Line must not be enclosed already
            ((canSnap(map[i].front(), a) && canSnap(map[i].back(), b)) ||
            (canSnap(map[i].front(), b) && canSnap(map[i].back(), a))))
        {
            map[i].append(map[i].front());
            return;
        }
    }

    //2. Can we continue an existing polyline?
    for (int i = 0; i < map.size(); i++)
    {
        if (map[i].front() != map[i].back())
        {
            if (canSnap(map[i].front(), a))
            {
                map[i].prepend(b);
                return;
            }
            if (canSnap(map[i].back(), a))
            {
                map[i].append(b);
                return;
            }
            if (canSnap(map[i].front(), b))
            {
                map[i].prepend(a);
                return;
            }
            if (canSnap(map[i].back(), b))
            {
                map[i].append(a);
                return;
            }
        }

    }
    //3. If not, just start a new polyline
    QVector<QPointF> v;
    v.append(a);
    v.append(b);
    map.push_back(v);
    emit mapChanged();
}
