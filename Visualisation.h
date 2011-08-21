#ifndef VISUALISATION_H
#define VISUALISATION_H

#include <QtGui>
#include "FightArena.h"

struct ControlLine
{
    int id;
    QLabel *botName;

    QHBoxLayout *botControlLayout;
    QButtonGroup *botControl;
    QRadioButton *botControlAI, *botControlManual;

    QCheckBox *botShow;

    QHBoxLayout *botRoleLayout;
    QButtonGroup *botRole;
    QRadioButton *botRoleHunter, *botRoleRunner;

    QPushButton *botDelete;
    QLabel* botState;

    ControlLine(int id_ = 0);
    void setDead();
    void clean();
};

class Visualisation: public QWidget
{
    Q_OBJECT

public:
    Visualisation(QVector<QVector<QPointF> > map_,  QWidget *parent = NULL);

private slots:
    void addBot();
    void deleteBot(int id);

private:
    FightArena* fightArena;
    QVector<QVector<QPointF> > map;
    QPushButton *togglePauseBtn;
    int maxid;

    QVector<ControlLine> controlLines;
    QVector<int> botids;
    QPushButton *addBotBtn;
    QGridLayout *botControlPanel;
    QVBoxLayout *botNames, *botRole, *botShow, *botControl, *botDelete, *botState;
    QSignalMapper *botShowCol;
    QSignalMapper *botRoleHunterCol, *botRoleRunnerCol;
    QSignalMapper *botControlManualCol, *botControlAICol;
    QSignalMapper *botDeleteCol;
};

#endif //VISUALISATION_H
