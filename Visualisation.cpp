#include <QtGui>

#include "Visualisation.h"
#include "FightArena.h"

Visualisation::Visualisation(QVector<QVector<QPointF> > map_, QWidget *parent):
    QWidget(parent),
    map(map_),
    maxid(0)
{
    fightArena = new FightArena(800, 600, map);

    addBotBtn = new QPushButton("Add bot");
    togglePauseBtn = new QPushButton("Pause visualisation");
    QHBoxLayout *visControls = new QHBoxLayout();
    visControls->addWidget(addBotBtn);
    connect(addBotBtn, SIGNAL(clicked()), this, SLOT(addBot()));
    visControls->addWidget(togglePauseBtn);

    botControlPanel = new QGridLayout();
    botControlPanel->addWidget(new QLabel("Name"), 0, 0);
    botControlPanel->addWidget(new QLabel("Control"), 0, 1);
    botControlPanel->addWidget(new QLabel("Show"), 0, 2);
    botControlPanel->addWidget(new QLabel("Role"), 0, 3);
    botControlPanel->addWidget(new QLabel("Delete"), 0, 4);
    botControlPanel->addWidget(new QLabel("State"), 0, 5);

    botNames = new QVBoxLayout();
    botControl = new QVBoxLayout();
    botShow = new QVBoxLayout();
    botRole = new QVBoxLayout();
    botDelete = new QVBoxLayout();
    botState = new QVBoxLayout();

    botControlAICol = new QSignalMapper();
    botControlManualCol = new QSignalMapper();
    connect(botControlAICol, SIGNAL(mapped(int)), fightArena, SLOT(toggleBotControl(int)));
    connect(botControlManualCol, SIGNAL(mapped(int)), fightArena, SLOT(toggleBotControl(int)));

    botShowCol = new QSignalMapper();// It's unuseful?


    botRoleHunterCol = new QSignalMapper();
    botRoleRunnerCol = new QSignalMapper();
    connect(botRoleHunterCol, SIGNAL(mapped(int)), fightArena, SLOT(toggleBotRole(int)));
    connect(botRoleRunnerCol, SIGNAL(mapped(int)), fightArena, SLOT(toggleBotRole(int)));

    botDeleteCol = new QSignalMapper();
    connect(botDeleteCol, SIGNAL(mapped(int)), fightArena, SLOT(deleteBot(int)));
    connect(fightArena, SIGNAL(botDead(int)), this, SLOT(deleteBot(int)));

    botControlPanel->addLayout(botNames, 1, 0);
    botControlPanel->addLayout(botControl, 1, 1);
    botControlPanel->addLayout(botShow, 1, 2);
    botControlPanel->addLayout(botRole, 1, 3);
    botControlPanel->addLayout(botDelete, 1, 4);
    botControlPanel->addLayout(botState, 1, 5);

    QGridLayout *mainLayout = new QGridLayout();
    mainLayout->addWidget(fightArena, 0, 0);
    mainLayout->addLayout(botControlPanel, 0, 1);
    mainLayout->addLayout(visControls, 1, 1);
    setLayout(mainLayout);
}

ControlLine::ControlLine(int id_):
    id(id_),
    botName(new QLabel(QString("bot %1").arg(id))),
    botControlAI(new QRadioButton()), botControlManual(new QRadioButton()),
    botShow(new QCheckBox()),
    botRoleHunter(new QRadioButton()), botRoleRunner(new QRadioButton()),
    botDelete(new QPushButton("delete")),
    botState(new QLabel("Alive"))
{
    botShow->setChecked(true);

    botControl = new QButtonGroup();
    botControlLayout = new QHBoxLayout();
    botControlAI->setChecked(true);
    botControl->addButton(botControlAI);
    botControl->addButton(botControlManual);
    botControlLayout->addWidget(botControlAI);
    botControlLayout->addWidget(botControlManual);

    botRole = new QButtonGroup();
    botRoleLayout = new QHBoxLayout();
    botRoleHunter->setChecked(true);
    botRole->addButton(botRoleHunter);
    botRole->addButton(botRoleRunner);
    botRoleLayout->addWidget(botRoleHunter);
    botRoleLayout->addWidget(botRoleRunner);
}
void ControlLine::setDead()
{
    botDelete->setDisabled(true);
    botState->setText("Dead");
}
void ControlLine::clean()
{
    delete botName;

    delete botControl;
    delete botControlLayout;
    delete botControlAI;
    delete botControlManual;
    delete botShow;

    delete botRoleHunter;
    delete botRoleRunner;
    delete botRole;
    delete botRoleLayout;

    delete botDelete;
    delete botState;
}

void Visualisation::addBot()
{
    int id = maxid;
    for (int i = 0; i < botids.size(); i++)
        id = qMax(id, botids[i]);
    id++;
    maxid = id;
    botids.push_back(id);
    fightArena->addBot(id);

    ControlLine cl(id);
    controlLines.push_back(cl);

    botNames->addWidget(cl.botName);

    botControl->addLayout(cl.botControlLayout);
    connect(cl.botControlAI, SIGNAL(clicked()), botControlAICol, SLOT(map()));
    connect(cl.botControlManual, SIGNAL(clicked()), botControlManualCol, SLOT(map()));
    botControlAICol->setMapping(cl.botControlAI, id);
    botControlManualCol->setMapping(cl.botControlManual, id);

    botShow->addWidget(cl.botShow);


    botRole->addLayout(cl.botRoleLayout);
    connect(cl.botRoleHunter, SIGNAL(clicked()), botRoleHunterCol, SLOT(map()));
    connect(cl.botRoleRunner, SIGNAL(clicked()), botRoleRunnerCol, SLOT(map()));
    botRoleHunterCol->setMapping(cl.botRoleHunter, id);
    botRoleRunnerCol->setMapping(cl.botRoleRunner, id);

    botDelete->addWidget(cl.botDelete);
    connect(cl.botDelete, SIGNAL(clicked()), botDeleteCol, SLOT(map()));
    botDeleteCol->setMapping(cl.botDelete, id);

    botState->addWidget(cl.botState);
    dumpObjectTree();
}

void Visualisation::deleteBot(int id)
{
    for (int i = 0; i < botids.size(); i++)
    {
        if (botids[i] == id)
        {
            botids.remove(i);
            controlLines[i].clean();
            controlLines.remove(i);
            break;
        }
    }
    dumpObjectTree();
}
