#ifndef MAPEDITOR_H
#define MAPEDITOR_H

#include <QtGui>
#include "EditArea.h"

class MapEditor: public QWidget
{
    Q_OBJECT

public:
    MapEditor(int mapwidth, int mapheight, const QString &, QWidget *parent = NULL);

signals:
    void gonnaDie();

private slots:
    void createNewMap();
    void saveToFile();
    void loadFromFile();
    void generateRandom();// The generated map looks lame, should be improved.
    void mapChanged(); // To add (unsaved) to the title.

private:
    void closeEvent(QCloseEvent *);

    QString name;
    QPushButton *newBtn, *loadBtn, *saveBtn, *randomBtn;
    EditArea *editArea; //NOTE: might be easier to delete the old widget and create new.
    QSlider *snapRadiusSlider;

    bool fileSaved;
};

#endif
