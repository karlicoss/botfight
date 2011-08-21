#include <QtGui>
#include "BotFight.h"
#include "Visualisation.h"
#include "editor/MapEditor.h"
#include "tools.h"

BotFight::BotFight(QWidget *parent):
    QWidget(parent),
    name("Bot fight"),
    mapEditor(NULL),
    visualisation(NULL),
    loadMapBtn(new QPushButton("Load...")),
    reloadMapBtn(new QPushButton("Reload map")),
    startMapEditorBtn(new QPushButton("Edit map"))
{
    setWindowTitle(name + " - " + "Empty map");

    connect(loadMapBtn, SIGNAL(clicked()), this, SLOT(loadFromFile()));
    connect(reloadMapBtn, SIGNAL(clicked()), this, SLOT(reloadMap()));
    connect(startMapEditorBtn, SIGNAL(clicked()), this, SLOT(editMap()));

    QHBoxLayout *mapControls = new QHBoxLayout();
    mapControls->addWidget(loadMapBtn);
    mapControls->addWidget(reloadMapBtn);
    mapControls->addWidget(startMapEditorBtn);
    mapControls->addStretch(1);



    mainLayout = new QGridLayout();
    setVisualisation(new Visualisation(QVector<QVector<QPointF> > ()));
    mainLayout->addLayout(mapControls, 1, 0);
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);

    setLayout(mainLayout);
}

void BotFight::closeEvent(QCloseEvent *)
{
    delete mapEditor;
}

void BotFight::unblockEditMap()
{
    startMapEditorBtn->setEnabled(true);
    mapEditor = NULL;
}

void BotFight::editMap()
{
    if (mapEditor != NULL)
    {
        return;
    }
#ifdef DEBUG
    qDebug() << "Creating a MapEditor window..." << endl;
#endif
    mapEditor = new MapEditor(800, 600, curMap);
    startMapEditorBtn->setDisabled(true);
    connect(mapEditor, SIGNAL(gonnaDie()), this, SLOT(unblockEditMap()));
    mapEditor->show();
}

namespace{

QVector<QVector<QPointF> > getMapFromFile(const QString &fileName)
{
    QFile file(fileName);
    file.open(QIODevice::ReadOnly);
    QDataStream in(&file);
    QVector<QVector<QPointF> > m;
    in >> m;
    file.close();
    return m;
}

}

void BotFight::loadFromFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    "Open map",
                                                    "",
                                                    "Map files (*.map);;All files (*)");

    if (fileName.isEmpty())
        return;
    else
    {
        curMap = fileName;
        setWindowTitle(name + " - " + fileName);
        QVector<QVector<QPointF> > m = getMapFromFile(fileName);
#ifdef DEBUG
        qDebug() << "File " + fileName + " has been loaded: " << endl << m << endl;
#endif
        setVisualisation(new Visualisation(m));
    }
}

void BotFight::reloadMap()
{
    if (curMap.isEmpty())
        return;
    QVector<QVector<QPointF> > m = getMapFromFile(curMap);
    setVisualisation(new Visualisation(m));
#ifdef DEBUG
    qDebug() << "File " + curMap + " has been reloaded: " << endl << m << endl;
#endif
}

void BotFight::setVisualisation(Visualisation *newvis)
{
    if (visualisation != NULL)
        delete visualisation;
    visualisation = newvis;
    mainLayout->addWidget(visualisation, 0, 0);
}
