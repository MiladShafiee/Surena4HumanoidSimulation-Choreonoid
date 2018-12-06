/**
   @author Shin'ichiro Nakaoka
*/

#include "GraphBar.h"
#include "GraphWidget.h"
#include "MainWindow.h"
#include "ExtensionManager.h"
#include "CheckBox.h"
#include "SpinBox.h"
#include "ComboBox.h"
#include "Dialog.h"
#include <QFileDialog>
#include <cnoid/ConnectionSet>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace std::placeholders;

namespace {

class ConfigDialog : public Dialog
{
public:
    ConnectionSet& connections;
    GraphWidget*& focusedGraphWidget;

    DoubleSpinBox verticalValueRangeSpin;
    CheckBox showLimitCheck;
    SpinBox lineWidthSpin;
    CheckBox gridCheck;
    DoubleSpinBox gridSizeSpin;
    CheckBox rulerCheck;
    CheckBox editModeCheck; // This should be toggle button on the tool bar.
    QButtonGroup radioGroup;
    RadioButton freeLineModeRadio;
    RadioButton lineModeRadio;
    CheckBox highlightingControlPointCheck;
    SpinBox controlPointStepSpin;
    SpinBox controlPointOffsetSpin;
    CheckBox timeBarSyncToggle;
    ComboBox autoScrollModeCombo;

    ConfigDialog(GraphBarImpl* barImpl);
    void focus(GraphWidget* graph);
        
    void onVerticalValueRangeChanged(double value);
    void onLineWidthChanged(int value);
    void onShowLimitToggled(bool on);
    void onGridToggled(bool on);
    void onGridSizeChanged(double size);
    void onRulerToggled(bool on);
    void onTimeBarSyncToggled(bool on);
    void onAutoScrollModeChanged(int index);
    void onEditModeToggled(bool on);
    void onFreeLineModeToggled(bool on);
    void onLineModeToggled(bool on);
    void onControlPointStepOrOffsetChanged();
    void onHighlightingControlPointToggled(bool on);
};

}

namespace cnoid {

class GraphBarImpl
{
public:
    GraphBarImpl(GraphBar* self);

    GraphBar* self;

    ToolButton* orgRenderingToggle;
    ToolButton* velRenderingToggle;
    ToolButton* accRenderingToggle;

    ConnectionSet connections;
    GraphWidget* focusedGraphWidget;

    ConfigDialog config;
        
    void focus(GraphWidget* graphWidget, bool forceUpdate);
    void onRenderingTypesToggled();
};

}


void GraphBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(GraphBar::instance());
        initialized = true;
    }
}

GraphBar* GraphBar::instance()
{
    static GraphBar* graphBar = new GraphBar();
    return graphBar;
}
       

GraphBar::GraphBar() : ToolBar(N_("GraphBar"))
{
    impl = new GraphBarImpl(this);
}


GraphBarImpl::GraphBarImpl(GraphBar* self)
    : self(self),
      config(this)
{
    self->setVisibleByDefault(true);    
    
    orgRenderingToggle = self->addToggleButton(QIcon(":/Base/icons/graph.png"),
                                               _("Plot trajectories of the target data on the graph view"));
    orgRenderingToggle->setChecked(true);
    connections.add(
        orgRenderingToggle->sigToggled().connect(
            std::bind(&GraphBarImpl::onRenderingTypesToggled, this)));

    velRenderingToggle = self->addToggleButton(QIcon(":/Base/icons/velocitygraph.png"),
                                               _("Plot velocity trajectories"));
    connections.add(
        velRenderingToggle->sigToggled().connect(
            std::bind(&GraphBarImpl::onRenderingTypesToggled, this)));

    accRenderingToggle = self->addToggleButton(QIcon(":/Base/icons/accgraph.png"),
                                               _("Plot acceleration trajectories"));
    // Hide this button because the acc trajectory is currently not supported by the graph wieget
    accRenderingToggle->hide();
    
    connections.add(
        accRenderingToggle->sigToggled().connect(
            std::bind(&GraphBarImpl::onRenderingTypesToggled, this)));

    self->addButton(QIcon(":/Base/icons/setup.png"), _("Show the config dialog"))
        ->sigClicked().connect(std::bind(&QDialog::show, &config));

    self->setEnabled(false);
    
    focusedGraphWidget = 0;
}


ConfigDialog::ConfigDialog(GraphBarImpl* barImpl)
    : connections(barImpl->connections),
      focusedGraphWidget(barImpl->focusedGraphWidget)
{
    setWindowTitle(_("Graph Config"));

    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout();        
    hbox->addWidget(new QLabel(_("Line width")));
    
    lineWidthSpin.setRange(1, 9);
    lineWidthSpin.setValue(1);
    connections.add(
        lineWidthSpin.sigValueChanged().connect(
            std::bind(&ConfigDialog::onLineWidthChanged, this, _1)));
    hbox->addWidget(&lineWidthSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("y-range, 10^")));
    
    verticalValueRangeSpin.setDecimals(1);
    verticalValueRangeSpin.setRange(-99.8, 99.8);
    verticalValueRangeSpin.setSingleStep(0.1);
    verticalValueRangeSpin.setValue(1.0);
    connections.add(
        verticalValueRangeSpin.sigValueChanged().connect(
            std::bind(&ConfigDialog::onVerticalValueRangeChanged, this, _1)));
    hbox->addWidget(&verticalValueRangeSpin);

    showLimitCheck.setText(_("Show limit values"));
    showLimitCheck.setChecked(true);
    connections.add(
        showLimitCheck.sigToggled().connect(
            std::bind(&ConfigDialog::onShowLimitToggled, this, _1)));
    hbox->addWidget(&showLimitCheck);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    gridCheck.setText(_("Show grid"));
    connections.add(
        gridCheck.sigToggled().connect(
            std::bind(&ConfigDialog::onGridToggled, this, _1)));
    hbox->addWidget(&gridCheck);
    
    gridSizeSpin.setDecimals(3);
    gridSizeSpin.setRange(-999.999, 999.999);
    gridSizeSpin.setSingleStep(0.001);
    gridSizeSpin.setValue(0.2);
    connections.add(
        gridSizeSpin.sigValueChanged().connect(
            std::bind(&ConfigDialog::onGridSizeChanged, this, _1)));
    hbox->addWidget(&gridSizeSpin);
    
    rulerCheck.setText(_("Show rulers"));
    rulerCheck.setEnabled(false);
    connections.add(
        rulerCheck.sigToggled().connect(
            std::bind(&ConfigDialog::onRulerToggled, this, _1)));
    hbox->addWidget(&rulerCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    editModeCheck.setText(_("Edit mode"));
    connections.add(
        editModeCheck.sigToggled().connect(
            std::bind(&ConfigDialog::onEditModeToggled, this, _1)));
    hbox->addWidget(&editModeCheck);

    radioGroup.addButton(&freeLineModeRadio);
    radioGroup.addButton(&lineModeRadio);
    
    freeLineModeRadio.setText(_("Free line"));
    freeLineModeRadio.setChecked(true);
    connections.add(
        freeLineModeRadio.sigToggled().connect(
            std::bind(&ConfigDialog::onFreeLineModeToggled, this, _1)));
    hbox->addWidget(&freeLineModeRadio);
    
    lineModeRadio.setText(_("Line edit"));
    connections.add(
        lineModeRadio.sigToggled().connect(
            std::bind(&ConfigDialog::onLineModeToggled, this, _1)));
    hbox->addWidget(&lineModeRadio);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    highlightingControlPointCheck.setText(_("Control points"));
    connections.add(
        highlightingControlPointCheck.sigToggled().connect(
            std::bind(&ConfigDialog::onHighlightingControlPointToggled, this, _1)));
    hbox->addWidget(&highlightingControlPointCheck);
    
    hbox->addWidget(new QLabel(_("Step")));
    controlPointStepSpin.setRange(1, 999);
    controlPointStepSpin.setValue(1);
    connections.add(
        controlPointStepSpin.sigValueChanged().connect(
            std::bind(&ConfigDialog::onControlPointStepOrOffsetChanged, this)));
    hbox->addWidget(&controlPointStepSpin);
    
    hbox->addWidget(new QLabel(_("Offset")));
    controlPointOffsetSpin.setRange(0, 999);
    controlPointOffsetSpin.setValue(0);
    connections.add(
        controlPointOffsetSpin.sigValueChanged().connect(
            std::bind(&ConfigDialog::onControlPointStepOrOffsetChanged, this)));
    hbox->addWidget(&controlPointOffsetSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    timeBarSyncToggle.setText(_("Time bar sync"));
    timeBarSyncToggle.setChecked(true);
    connections.add(
        timeBarSyncToggle.sigToggled().connect(
            std::bind(&ConfigDialog::onTimeBarSyncToggled, this, _1)));
    hbox->addWidget(&timeBarSyncToggle);

    autoScrollModeCombo.addItem(_("off"));
    autoScrollModeCombo.addItem(_("cont."));
    autoScrollModeCombo.addItem(_("page"));
    autoScrollModeCombo.setCurrentIndex(1);
    connections.add(
        autoScrollModeCombo.sigCurrentIndexChanged().connect(
            std::bind(&ConfigDialog::onAutoScrollModeChanged, this, _1)));
    hbox->addWidget(&autoScrollModeCombo);
    hbox->addStretch();
    vbox->addLayout(hbox);
}


GraphBar::~GraphBar()
{
    delete impl;
}


GraphWidget* GraphBar::focusedGraphWidget()
{
    return impl->focusedGraphWidget;
}


void GraphBar::focus(GraphWidget* graph, bool forceUpdate)
{
    impl->focus(graph, forceUpdate);
}


void GraphBarImpl::focus(GraphWidget* graph, bool forceUpdate)
{
    if(graph && (forceUpdate || (graph != focusedGraphWidget))){

        focusedGraphWidget = graph;
        self->setEnabled(true);

        connections.block();
        
        bool org, vel, acc;
        graph->getRenderingTypes(org, vel, acc);
        orgRenderingToggle->setChecked(org);
        velRenderingToggle->setChecked(vel);
        accRenderingToggle->setChecked(acc);

        config.focus(graph);

        connections.unblock();
    }
}


void ConfigDialog::focus(GraphWidget* graph)
{
    editModeCheck.setChecked(graph->mode() == GraphWidget::EDIT_MODE);

    GraphWidget::EditMode editMode = graph->editMode();
    if(editMode == GraphWidget::FREE_LINE_MODE){
        freeLineModeRadio.setChecked(true);
    } else if(editMode == GraphWidget::LINE_MODE){
        lineModeRadio.setChecked(true);
    }

    double lower, upper;
    graph->getVerticalValueRange(lower, upper);
    verticalValueRangeSpin.setValue(upper);
    
    lineWidthSpin.setValue(graph->getLineWidth());
    
    timeBarSyncToggle.setChecked(graph->isTimeBarSyncMode());
    autoScrollModeCombo.setCurrentIndex(graph->autoScrollMode());
    showLimitCheck.setChecked(graph->showsLimits());
    rulerCheck.setChecked(graph->showsRulers());
    gridCheck.setChecked(graph->showsGrid());
    
    double width, height;
    graph->getGridSize(width, height);
    gridSizeSpin.setValue(width);
    
    int step, offset;
    graph->getControlPointStep(step, offset);
    controlPointStepSpin.setValue(step);
    controlPointOffsetSpin.setValue(offset);
    highlightingControlPointCheck.setChecked(graph->highlightsControlPoints());
}


void GraphBar::releaseFocus(GraphWidget* graphWidget)
{
    if(impl->focusedGraphWidget == graphWidget){
        impl->focusedGraphWidget = 0;
        setEnabled(false);
    }
}


void GraphBarImpl::onRenderingTypesToggled()
{
    focusedGraphWidget->setRenderingTypes
        (orgRenderingToggle->isChecked(), velRenderingToggle->isChecked(), accRenderingToggle->isChecked());
}


void ConfigDialog::onVerticalValueRangeChanged(double value)
{
    double upper = pow(10.0, value);
    double lower = -upper;
    focusedGraphWidget->setVerticalValueRange(lower, upper);
}


void ConfigDialog::onLineWidthChanged(int value)
{
    focusedGraphWidget->setLineWidth(value);
}
        

void ConfigDialog::onShowLimitToggled(bool on)
{
    focusedGraphWidget->showLimits(on);
}


void ConfigDialog::onGridToggled(bool on)
{
    focusedGraphWidget->showGrid(on);
}


void ConfigDialog::onGridSizeChanged(double size)
{
    focusedGraphWidget->setGridSize(size, size);
}


void ConfigDialog::onRulerToggled(bool on)
{
    focusedGraphWidget->showRulers(on);
}


void ConfigDialog::onTimeBarSyncToggled(bool on)
{
    focusedGraphWidget->setTimeBarSyncMode(on);
}


void ConfigDialog::onAutoScrollModeChanged(int index)
{
    focusedGraphWidget->setAutoScrollMode((GraphWidget::ScrollMode)index);
}


void ConfigDialog::onEditModeToggled(bool on)
{
    GraphWidget::Mode mode = on ? GraphWidget::EDIT_MODE : GraphWidget::VIEW_MODE;
    focusedGraphWidget->changeMode(mode);
}


void ConfigDialog::onFreeLineModeToggled(bool on)
{
    if(on){
        focusedGraphWidget->changeEditMode(GraphWidget::FREE_LINE_MODE);
    }
}


void ConfigDialog::onLineModeToggled(bool on)
{
    if(on){
        focusedGraphWidget->changeEditMode(GraphWidget::LINE_MODE);
    }
}


void ConfigDialog::onControlPointStepOrOffsetChanged()
{
    focusedGraphWidget->setControlPointStep(
        controlPointStepSpin.value(), controlPointOffsetSpin.value());
}


void ConfigDialog::onHighlightingControlPointToggled(bool on)
{
    focusedGraphWidget->highlightControlPoints(on);
}
