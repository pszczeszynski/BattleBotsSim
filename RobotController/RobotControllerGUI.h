#pragma once

#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QVBoxLayout>  // Include the QVBoxLayout header
#include <QSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <opencv2/opencv.hpp>
#include "RobotConfig.h"
#include <QMouseEvent>
#include <QKeyEvent>
#include <QProgressBar>
#include <QPainter>
#include "Globals.h"
#include "Input/Input.h"
#include "UIWidgets/IMUWidget.h"
#include "Clock.h"

class RobotControllerGUI : public QMainWindow
{
    Q_OBJECT

public slots:
    // needs to be a slot so it can be called from the robot controller thread
    // qt will schedule the call to this function so that it is thread safe
    void RefreshFieldImage();
public:
    void ShowGUI();
    void SetApp(QApplication& app);
    static RobotControllerGUI& GetInstance();
    QLabel *GetImageLabel();
    QLabel *GetVescInfo(int row, int column);
    
protected:
    // qt event handlers
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    // private ctor since this is a singleton
    RobotControllerGUI();

    QLabel *_imageLabel;
    QApplication *app;
    IMUWidget *_imuWidget;
    Clock _displayImageClock;
    QLabel *vescInfo[4][4];
};

class TargetRpmProgressBar : public QProgressBar
{
    Q_OBJECT

public:
    TargetRpmProgressBar(QWidget *parent = nullptr) : QProgressBar(parent), targetRpm(0) {}

    void SetTargetRpm(float value)
    {
        targetRpm = value;
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QProgressBar::paintEvent(event);
        int targetX = (targetRpm / maximum()) * width();
        QPainter painter(this);
        QPen pen(Qt::magenta, 3); // Adjust the color and width as needed
        painter.setPen(pen);
        painter.drawLine(targetX, 0, targetX, height());
    }

private:
    float targetRpm;
};
