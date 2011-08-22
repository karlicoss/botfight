#ifndef FIGHTARENA_H
#define FIGHTARENA_H

#include <QtGui>
#include "Bot.h"

class FightArena: public QWidget
{
    Q_OBJECT

public:
    FightArena(int width_, int height_, QVector<QVector<QPointF> > map_,  QWidget *parent = NULL);

signals:
    void botDead(int id);

public slots:
    void togglePause();
    void toggleBotRole(int id);
    void toggleBotControl(int id);
    void addBot(int id);
    void deleteBot(int id);
    void giveBots(Bot* bot, QVector<Bot*> *result);
    void giveKeys(QHash<int, bool> *result);

private slots:
    void moveBots();

private:
    void paintEvent(QPaintEvent *);
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);

    QVector<QVector<QPointF > > map;// Contains just the map, shoudn't be changed during the visualisation. Changed once in the constructor to add the screen edges.
    QHash<int, bool> pressedKeys;// This map contains the states of the keys, to support key combinations(e.g. to rotate and move simultaneously)

    QTimer* timer;// Calls makeMove
    QVector<Bot*> bots;
    QVector<bool> showBot;// Show bot and it's discovered area on the map.

};

#endif // FIGHTARENA_H
