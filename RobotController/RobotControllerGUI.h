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
#include <QKeyEvent>
#include "Globals.h"

class RobotConfigWindow : public QMainWindow
{
    Q_OBJECT

public:
    void RefreshFieldImage();
    void ShowGUI();
    void SetApp(QApplication& app);
    static RobotConfigWindow& GetInstance();

protected:
    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
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

    bool eventFilter(QObject *watched, QEvent *event) override
    {
        // We're interested in KeyPress and KeyRelease events
        if(event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease)
        {
            // Cast the QEvent pointer to a QKeyEvent pointer
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

            ///// WASD

            // w
            if(keyEvent->key() == Qt::Key_W)
            {
                if(event->type() == QEvent::KeyPress) {
                    wDown = true;
                } else {
                    wDown = false;
                }
            }

            // a
            if(keyEvent->key() == Qt::Key_A)
            {
                if(event->type() == QEvent::KeyPress) {
                    aDown = true;
                } else {
                    aDown = false;
                }
            }

            // s
            if(keyEvent->key() == Qt::Key_S)
            {
                if(event->type() == QEvent::KeyPress) {
                    sDown = true;
                } else {
                    sDown = false;
                }
            }

            // d
            if(keyEvent->key() == Qt::Key_D)
            {
                if(event->type() == QEvent::KeyPress) {
                    dDown = true;
                } else {
                    dDown = false;
                }
            }
        }

        // Call the base class implementation (important!)
        return QMainWindow::eventFilter(watched, event);
    }

private:
    // private ctor since this is a singleton
    RobotConfigWindow();

    class MouseEvents : public QObject
    {
    public:
        MouseEvents(RobotConfigWindow& window);
        bool eventFilter(QObject *obj, QEvent *event);
    };

    QApplication *app;
    QLabel *imageLabel;
};
