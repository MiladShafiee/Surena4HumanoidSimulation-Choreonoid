#include "Headers/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//milad(ui->widget);

//    Charts=new ChartForm();

//   Charts->show();
   //Charts.Plot( Pelvis);

    //setGeometry(400, 250, 542, 390);
}





void MainWindow::Plot(TaskSpaceOnline1 Pelvis){
//milad(ui->widget);
//    Charts=new ChartForm();
//    Charts->show();
//    Charts->Plot( Pelvis);


//    Chartsfoot=new ChartForm();
//    Chartsfoot->show();
//    Chartsfoot->PlotFootOffline(Pelvis);

//    ChartsPelvisOffline=new ChartForm();
//    ChartsPelvisOffline->show();
//    ChartsPelvisOffline->PlotOfflinePelvis(Pelvis);

//-------------uncomment following for plotting foot trajectories------------------

//    Chartsfoot=new ChartForm();
//    Chartsfoot->show();
//    Chartsfoot->PlotFoot(Pelvis);

//    ChartsfootTime=new ChartForm();
//    ChartsfootTime->show();
//    ChartsfootTime->PlotFootTime(Pelvis);

//--------------uncomment above for plotting foot trajectories----------------------

//    ChartsfootVel=new ChartForm();
//    ChartsfootVel->show();
//    ChartsfootVel->PlotFootVel(Pelvis);


//    ChartsfootAccel=new ChartForm();
//    ChartsfootAccel->show();
//    ChartsfootAccel->PlotFootAccel(Pelvis);


//    Charts1=new ChartForm();
//    Charts1->show();
//    Charts1->Plot( Pelvis);

//        Chartsalaki2=new ChartForm();
//       Chartsalaki2->show();
//        Chartsalaki2->milad(Pelvis/*ui->widget*/);


      // subLayout1 = new QCPLayoutGrid;
      // QWidget *parent = 0;
        Chartsalaki=new ChartForm(0,Pelvis);
       Chartsalaki->show();
      //  Chartsalaki->Plotchoreonoidtest(Pelvis,plot2,subLayout1,AxisRect2/*ui->widget*/);

}


//void MainWindow::milad(QCustomPlot *customPlot){

//    // add two new graphs and set their look:
//    //QCustomPlot *customPlot;


//    demoName = "Simple Demo";

//    demoName = "Quadratic Demo";
//    // generate some data:
//    QVector<double> x(101), y(101); // initialize with entries 0..100
//    for (int i=0; i<101; ++i)
//    {
//      x[i] = i/50.0 - 1; // x goes from -1 to 1
//      y[i] = x[i]*x[i];  // let's plot a quadratic function
//    }
//    // create graph and assign data to it:
//    customPlot->addGraph();
//    customPlot->graph(0)->setData(x, y);
//    // give the axes some labels:
//    customPlot->xAxis->setLabel("x");
//    customPlot->yAxis->setLabel("y");
//    // set axes ranges, so we see all data:
//    customPlot->xAxis->setRange(-1, 1);
//    customPlot->yAxis->setRange(0, 1);
//customPlot->replot();

//}


MainWindow::~MainWindow()
{
    delete ui;
}
