#include <QtGui>

#include <queue>
#include <cmath>
#include <algorithm>

#include "Visualisation.h"
#include "Bot.h"
#include "tools.h"

FightArena::FightArena(int width_, int height_, QVector<QVector<QPointF> > map_, QWidget *parent):
    QWidget(parent),
    map(map_),
    timer(new QTimer(this))
{
   qsrand(10);
   setFixedSize(width_, height_);
   setFocusPolicy(Qt::StrongFocus);

   connect(timer, SIGNAL(timeout()), this, SLOT(moveBots()));

   timer->start(50);// let it begin!
}

void FightArena::keyPressEvent(QKeyEvent *e)
{
    pressedKeys[e->key()] = true;
}

void FightArena::keyReleaseEvent(QKeyEvent *e)
{
    pressedKeys[e->key()] = false;
}

void FightArena::paintEvent(QPaintEvent *)
{
    QVector<QColor> botColors;
    QVector<QPainterPath> marks;

    for (int i = 0; i < bots.size(); i++)
    {

        QPointF botPos = bots[i]->getPos();
        qreal botAngle = bots[i]->getAngle();
        QPainterPath posMark;
        qreal markWidth = 20, markHeight = 10;
        QPolygonF triangle;
        triangle << QPointF(-markWidth / 2, markHeight / 2) << QPointF(-markWidth / 2, -markHeight / 2) << QPointF(markWidth / 2, 0);
        triangle << triangle.front();
        QMatrix rotM;
        rotM.translate(botPos.x(), botPos.y());
        rotM.rotate(-rad2degr(botAngle));
        triangle = rotM.map(triangle);
        posMark.addPolygon(triangle);
        posMark.addText(botPos + QPointF(5, 5), QFont(), QString::number(bots[i]->id));
        marks.push_back(posMark);

        botColors.push_back(QColor::fromHsv(30 * i, 200, 200));
    }


    QPen bkgPen(Qt::white);
    QPen posMarkPen(Qt::green);
    QPen mapPen(QColor(45, 0, 179), 3);

    QBrush bkgBrush(Qt::white);
    QBrush posMarkBrush(Qt::green);

    QImage img(width(), height(), QImage::Format_RGB32);
    QPainter p(&img);// Drawing on the widget directly causes perfomance loss.


    p.setPen(bkgPen);
    p.setBrush(bkgBrush);
    p.drawRect(QRectF(0, 0, 799, 599));

    p.setPen(mapPen);
    for (int i = 0; i < map.size(); i++)
    {
        for (int j = 0; j < map[i].size() - 1; j++)
        {
            p.drawLine(QLineF(map[i][j], map[i][j + 1]));
        }
    }
    for (int i = 0; i < bots.size(); i++)
    {
        p.setPen(botColors[i]);
        p.setBrush(botColors[i]);
        p.drawPath(marks[i]);
#ifdef DEBUG
        p.setPen(Qt::magenta);// Drawing the FOV
        p.setBrush(Qt::transparent);
        p.drawPie(QRectF(bots[i]->getPos() - QPointF(bots[i]->fovDist, bots[i]->fovDist), bots[i]->curPos + QPointF(bots[i]->fovDist, bots[i]->fovDist)), rad2degr(bots[i]->curAngle - bots[i]->fovAngle / 2) * 16, rad2degr(bots[i]->fovAngle) * 16);
        QColor curCol = botColors[i];
        curCol.setAlpha(30);
        p.setPen(curCol);
        p.setBrush(curCol);
        for (int q = 0; q < bots[i]->isDiscovered.size(); q++)
        {
            for (int w = 0; w < bots[i]->isDiscovered[q].size(); w++)
            {
                if (bots[i]->isDiscovered[q][w])
                {
                    p.drawEllipse(bots[i]->cellSize * QPointF(q, w), bots[i]->cellSize, bots[i]->cellSize);
                }
            }
        }
#endif
    }

    QPainter q(this);
    q.drawImage(QPointF(0, 0), img);// And finally..Drawing the whole image on the widget.
}

void FightArena::handleKeys()
{
    bool needsDiscover = false;
    if (pressedKeys[Qt::Key_Left])
    {
        //curAngle += rotSpeed;
        needsDiscover = true;
    }
    if (pressedKeys[Qt::Key_Right])
    {
        //curAngle -= rotSpeed;
        needsDiscover = true;
    }
    if (pressedKeys[Qt::Key_Up])
    {
        //makeManualMove();
        needsDiscover = true;
    }
    if (needsDiscover)
    {
        //exploreMap();
    }
}

void FightArena::togglePause()
{
    if (timer->isActive())
        timer->stop();
    else
        timer->start(50);
}

void FightArena::moveBots()
{
    for (int i = 0; i < bots.size(); i++)
    {
        bots[i]->makeMove();
    }
    update();
}

void FightArena::addBot(int id)
{
    int rx = qrand() % width();
    int ry = qrand() % height();
    QPointF pos(rx, ry);
    Bot *nb = new Bot(10.0, 20.0,
                      200, 75.0,
                      pos, -45.0,
                      width(), height(),
                      map,
                      id);
    connect(nb, SIGNAL(queryBots(Bot*,QVector<Bot*>*)), this, SLOT(giveBots(Bot*,QVector<Bot*>*)));
    connect(nb, SIGNAL(queryKeys(QHash<int,bool>*)), this, SLOT(giveKeys(QHash<int,bool>*)));
    connect(nb, SIGNAL(botKilled(int)), this, SLOT(deleteBot(int)));
    bots.append(nb);
    showBot.append(true);
}

void FightArena::deleteBot(int id)
{
    for (int i = 0; i < bots.size(); i++)
    {
        if (bots[i]->id == id)
        {
            delete bots[i];
            bots.remove(i);
            showBot.remove(i);
            emit botDead(id);
            break;
        }
    }
}

void FightArena::toggleBotControl(int id)
{
    for (int i = 0; i < bots.size(); i++)
    {
        if (bots[i]->id == id)
        {
            bots[i]->toggleControl();
        }
    }
}

void FightArena::toggleBotRole(int id)
{
    for (int i = 0; i < bots.size(); i++)
    {
        if (bots[i]->id == id)
        {
            bots[i]->toggleRole();
        }
    }
}

void FightArena::giveBots(Bot* bot, QVector<Bot *> *result)
{
    for (int i = 0; i < bots.size(); i++)
    {
        if (bots[i] == bot)
            continue;
        if (fitsInPie(bots[i]->getPos(), bot->getPos(), bot->fovDist, bot->getAngle(), bot->fovAngle) && !bot->wallOnPathTo(bots[i]->getPos()))
        {
            result->push_back(bots[i]);
        }
    }
}

void FightArena::giveKeys(QHash<int, bool> *result)
{
    *result = pressedKeys;
}

