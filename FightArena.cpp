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
    QImage img(width(), height(), QImage::Format_ARGB32);
    QPainter p(&img);

    QPen bkgPen(Qt::white);
    QPen mapPen(QColor(45, 0, 179), 3);

    QBrush bkgBrush(Qt::white);

    p.setPen(bkgPen);
    p.setBrush(bkgBrush);
    p.drawRect(0, 0, 800, 600);

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
        p.drawImage(QPointF(0, 0), bots[i]->getImage());
    }
    QPainter q(this);
    q.drawImage(QPointF(0, 0), img);// And finally..Drawing the whole image on the widget.
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
        if (bots[i]->getID() == id)
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
        if (bots[i]->getID() == id)
        {
            bots[i]->toggleControl();
        }
    }
}

void FightArena::toggleBotRole(int id)
{
    for (int i = 0; i < bots.size(); i++)
    {
        if (bots[i]->getID() == id)
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
        if (fitsInPie(bots[i]->getPos(), bot->getPos(), bot->getFOVDistance(), bot->getAngle(), bot->getFOVAngle()) &&
            !bot->wallBetween(bot->getPos(), bots[i]->getPos()))
        {
            result->push_back(bots[i]);
        }
    }
}

void FightArena::giveKeys(QHash<int, bool> *result)
{
    *result = pressedKeys;
}



