#include <QtGui>
#include "BotFight.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
   
    BotFight *p = new BotFight();
    p->show();

    return app.exec();
}
