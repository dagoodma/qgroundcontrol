#ifndef SLUGSTABBEDCONTROLWIDGET_H
#define SLUGSTABBEDCONTROLWIDGET_H

#include <QWidget>
#include "ui_SlugsTabbedControl.h"
#include "MAVLinkDecoder.h"
#include "SlugsControlWidget.h"
#include "SlugsStatusWidget.h"
class SlugsTabbedControlWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit SlugsTabbedControlWidget(QWidget *parent = 0);
    ~SlugsTabbedControlWidget();
private:
    MAVLinkDecoder *m_decoder;
    Ui::SlugsTabbedControl ui;
    //QGCMessageView *messageView;
    SlugsControlWidget *controlWidget;
    SlugsStatusWidget *statusWidget;
};

#endif // SLUGSTABBEDCONTROLWIDGET_H
