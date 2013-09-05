#include "SlugsTabbedControlWidget.h"

SlugsTabbedControlWidget::SlugsTabbedControlWidget(QWidget *parent) : QWidget(parent)
{
    ui.setupUi(this);
    //messageView = new QGCMessageView(this);
    controlWidget = new SlugsControlWidget(this);
    //ui.tabWidget->addTab(messageView,"Messages");
    ui.tabWidget->addTab(controlWidget,"Control");

}

SlugsTabbedControlWidget::~SlugsTabbedControlWidget()
{
}
