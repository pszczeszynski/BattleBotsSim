#pragma once

#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <opencv2/opencv.hpp>
#include "RobotConfig.h"
#include <QMouseEvent>

class RobotConfigWindow : public QMainWindow
{
    Q_OBJECT

public:
    RobotConfigWindow();
    void RefreshFieldImage();
    void ShowGUI();
    void SetApp(QApplication& app);

protected:
    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            std::cout << "PRESSED AT: " << event->x() << ", " << event->y() << std::endl;
            std::cout << "image label pos: " << imageLabel->pos().x() << ", " << imageLabel->pos().y() << std::endl;
            // Handle left button press
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            // Handle left button release
        }
    }

    void mouseDoubleClickEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            // Handle left button double click
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        // Handle mouse move
    }

private:

    class MouseEvents : public QObject
    {
    public:
        MouseEvents(RobotConfigWindow& window);
        bool eventFilter(QObject *obj, QEvent *event);
    };

    QApplication *app;
    QLabel *imageLabel;
};
