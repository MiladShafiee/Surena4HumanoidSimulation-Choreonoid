#ifndef CHARTFORM_H
#define CHARTFORM_H
#include "Headers/Robot.h"
#include<QForeachContainer>


#include <QMainWindow>
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <Headers/TaskSpace.h>
#include<Headers/taskspaceoffline.h>
#include <Headers/qcustomplot.h>
#include"Headers/taskspaceonline1.h"
#include"Headers/taskspaceonline2.h"
namespace Ui {
class ChartForm;
}

class ChartForm : public QMainWindow
{
    Q_OBJECT

public:
    explicit ChartForm(QWidget *parent, TaskSpaceOnline2 pelvis);
    ~ChartForm();

    void Plot(TaskSpaceOnline1 Pelvis);

    void PlotFoot(TaskSpace Pelvis);
    void PlotFootAccel(TaskSpace Pelvis);
    void PlotFootTime(TaskSpace Pelvis);
    void PlotFootVel(TaskSpace Pelvis);
    void PlotFootOffline(TaskSpaceOffline Pelvis);
    void PlotOfflinePelvis(TaskSpaceOffline Pelvis);
    void milad(TaskSpace Pelvis);
    void Plotchoreonoidtest(QCustomPlot *customPlot, TaskSpaceOnline2 Pelvis);
private:
    Ui::ChartForm *ui;
    QCustomPlot *figure1;
};

#endif // CHARTFORM_H
