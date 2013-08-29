#ifndef SLUGSTABBEDCONTROLWIDGET_H
#define SLUGSTABBEDCONTROLWIDGET_H

#include <QWidget>
#include "ui_SlugsTabbedControl.h"
#include "MAVLinkDecoder.h"
#include "QGCMessageView.h"
#include "SlugsControlWidget.h"
//#include "UASActionsWidget.h"
//#include "UASQuickView.h"
//#include "UASRawStatusView.h"
class SlugsTabbedControlWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit SlugsTabbedControlWidget(QWidget *parent = 0);
    ~SlugsTabbedControlWidget();
private:
    MAVLinkDecoder *m_decoder;
    Ui::SlugsTabbedControl ui;
    QGCMessageView *messageView;
    SlugsControlWidget *controlWidget;
};

#endif // SLUGSTABBEDCONTROLWIDGET_H
