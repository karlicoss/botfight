#ifndef MAPEXPLORATION_H
#define MAPEXPLORATION_H

#include <QtGui>

#include "Visualisation.h"
#include "editor/MapEditor.h"

class BotFight: public QWidget
{
    Q_OBJECT

public:
    BotFight(QWidget *parent = NULL);

private slots:
    void loadFromFile();
    void reloadMap();
    void editMap(); // Creates a map editor instanse. If there is one already, does nothing.
    void unblockEditMap();// To prevent creating multiple map editor windows, "Edit Map" button is blocked while editing map. This slot handles unblocking it after map editor close.

private:
    void closeEvent(QCloseEvent *);
    void setVisualisation(Visualisation *newvis);// Handles the signals and layouting too

    QString name;
    MapEditor *mapEditor;
    Visualisation *visualisation;
    QPushButton *loadMapBtn, *reloadMapBtn, *startMapEditorBtn;
    QString curMap;
    QGridLayout *mainLayout;
};

#endif //MAPEXPLORATION_H
