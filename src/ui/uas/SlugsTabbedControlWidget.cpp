#include "SlugsTabbedControlWidget.h"

SlugsTabbedControlWidget::SlugsTabbedControlWidget(QWidget *parent) : QWidget(parent)
{
    ui.setupUi(this);
    controlWidget = new SlugsControlWidget(this);
    statusWidget = new SlugsStatusWidget(this);

    //ui.tabWidget->addTab(messageView,"Messages");
    ui.tabWidget->addTab(controlWidget,"Control");

    ui.verticalLayout_2->insertWidget(0, statusWidget);
    //ui.statusWidget = new SlugsStatusWidget(this);

}

SlugsTabbedControlWidget::~SlugsTabbedControlWidget()
{
}
