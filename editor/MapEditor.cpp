#include "MapEditor.h"
#include "EditArea.h"

namespace
{

QVector<QVector<QPointF> > getMapFromFile(const QString &fileName)// assuming the map exist
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

MapEditor::MapEditor(int mapwidth, int mapheight, const QString &fileName, QWidget *parent):
    QWidget(parent),
    name("Simple map editor"),
    newBtn(new QPushButton("New")),
    loadBtn(new QPushButton("Load...")),
    saveBtn(new QPushButton("Save...")),
    randomBtn(new QPushButton("Random")),
    editArea(new EditArea(this)),
    snapRadiusSlider(new QSlider(Qt::Horizontal))
{
    connect(newBtn, SIGNAL(clicked()), this, SLOT(createNewMap()));
    connect(loadBtn, SIGNAL(clicked()), this, SLOT(loadFromFile()));
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveToFile()));
    connect(randomBtn, SIGNAL(clicked()), this, SLOT(generateRandom()));
    
    QHBoxLayout *btnLayout = new QHBoxLayout();
    btnLayout->addWidget(newBtn);
    btnLayout->addWidget(loadBtn);
    btnLayout->addWidget(saveBtn);
    btnLayout->addWidget(randomBtn);

    editArea->move(10, 10);
    editArea->setFixedSize(mapwidth, mapheight);
    editArea->setMouseTracking(true);
    editArea->setSnapRadius(25);
    if (!fileName.isEmpty())
    {
        editArea->setMap(getMapFromFile(fileName));
        setWindowTitle(name + " - " + fileName);
        fileSaved = true;
    }
    else
    {
        createNewMap();
    }
    connect(editArea, SIGNAL(mapChanged()), this, SLOT(mapChanged()));

    snapRadiusSlider->setMaximum(50);
    snapRadiusSlider->setSliderPosition(25);
    connect(snapRadiusSlider, SIGNAL(sliderMoved(int)), editArea, SLOT(setSnapRadius(int)));

    QVBoxLayout *snapLayout = new QVBoxLayout();
    QLabel *lbl = new QLabel("Snap precision:");
    snapLayout->addWidget(lbl);
    snapLayout->addWidget(snapRadiusSlider);
    snapLayout->addStretch(1);

    QGridLayout *mainLayout = new QGridLayout();
    mainLayout->addWidget(editArea, 0, 0);
    mainLayout->addLayout(btnLayout, 1, 0);
    mainLayout->addLayout(snapLayout, 0, 1);
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);

    setLayout(mainLayout);
}

void MapEditor::closeEvent(QCloseEvent *)
{
    emit gonnaDie();
}

void MapEditor::generateRandom()
{
    setWindowTitle(name + " - " + "random map(unsaved)");
    fileSaved = false;
    QVector<QVector<QPointF> > m;
    for (int i = 0; i < 15; i++)
    {
        QVector<QPointF> l;
        int w = qMax(qrand() % 400, 10);
        int h = qMax(qrand() % 300, 10);
        int px = qrand() % (editArea->width() - w);
        int py = qrand() % (editArea->height() - h);
        for (int j = 0; j < 5; j++)
        {
            l.append(QPointF(px + qrand() % w, py + qrand() % h));
        }
        if (qrand() % 5 == 0)
            l.append(l.front());
        m.append(l);
    }
    editArea->setMap(m);
}

void MapEditor::loadFromFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    "Open map",
                                                    "",
                                                    "Map files (*.map);;All files (*)");

    if (fileName.isEmpty())
        return;
    else
    {
        setWindowTitle(name + " - " + fileName);
        QVector<QVector<QPointF> > m = getMapFromFile(fileName);
        editArea->setMap(m);
        fileSaved = true;
    }
    editArea->update();
}


void MapEditor::saveToFile()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                   "Save map",
                                                   "untitled.map",
                                                   "Map files (*.map);;All files (*)");
    if (fileName.isEmpty())
        return;
    else
    {
        setWindowTitle(name + " - " + fileName);
        QFile file(fileName);
        file.open(QIODevice::WriteOnly);
        QDataStream out(&file);
        out << editArea->getMap();
        file.close();
        fileSaved = true;
    }
}

void MapEditor::mapChanged()
{
    setWindowTitle(name + "(unsaved)");
    fileSaved = false;
}

void MapEditor::createNewMap()
{
    setWindowTitle(name + " - " + "Empty map");
    fileSaved = false;
    QVector<QVector<QPointF> > m;
    editArea->setMap(m);
}

