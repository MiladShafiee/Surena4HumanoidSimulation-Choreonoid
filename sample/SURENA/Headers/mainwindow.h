#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "Headers/TaskSpace.h"
#include <QMainWindow>
#include "Headers/TaskSpace.h"
#include <Headers/qcustomplot.h>
#include"Headers/chartform.h"
#include "Headers/taskspaceoffline.h"
#include"Headers/taskspaceonline1.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    ChartForm *Charts;
     ChartForm *Charts1;
     ChartForm *Chartsalaki;
          ChartForm *Chartsalaki2;
ChartForm *Chartsfoot;
ChartForm *ChartsPelvisOffline;
// QCPGraph *plot2;
// QCPLayoutGrid *subLayout1;
//  QCPAxisRect *AxisRect2;
ChartForm *ChartsfootTime;

ChartForm *ChartsfootVel;
ChartForm  *ChartsfootAccel;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
QString demoName;
 QCustomPlot *customPlot;
 void Plot(TaskSpaceOnline1 Pelvis);
 void milad(QCustomPlot *customPlot);
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H




