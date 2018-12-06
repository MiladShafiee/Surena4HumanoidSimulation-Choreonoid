/********************************************************************************
** Form generated from reading UI file 'chartform.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CHARTFORM_H
#define UI_CHARTFORM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "Headers/qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_ChartForm
{
public:
    QWidget *centralwidget;
    QCustomPlot *widget1;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ChartForm)
    {
        if (ChartForm->objectName().isEmpty())
            ChartForm->setObjectName(QStringLiteral("ChartForm"));
        ChartForm->resize(579, 674);
        centralwidget = new QWidget(ChartForm);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        widget1 = new QCustomPlot(centralwidget);
        widget1->setObjectName(QStringLiteral("widget1"));
        widget1->setGeometry(QRect(0, 60, 381, 561));
        ChartForm->setCentralWidget(centralwidget);
        menubar = new QMenuBar(ChartForm);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 579, 25));
        ChartForm->setMenuBar(menubar);
        statusbar = new QStatusBar(ChartForm);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        ChartForm->setStatusBar(statusbar);

        retranslateUi(ChartForm);

        QMetaObject::connectSlotsByName(ChartForm);
    } // setupUi

    void retranslateUi(QMainWindow *ChartForm)
    {
        ChartForm->setWindowTitle(QApplication::translate("ChartForm", "MainWindow", 0));
    } // retranslateUi

};

namespace Ui {
    class ChartForm: public Ui_ChartForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CHARTFORM_H
