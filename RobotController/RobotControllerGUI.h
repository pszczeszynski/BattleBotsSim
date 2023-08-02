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
#include "Globals.h"
#include "Mouse.h"

class RobotConfigWindow : public QMainWindow
{
    Q_OBJECT

public slots:
    // needs to be a slot so it can be called from the robot controller thread
    // qt will schedule the call to this function so that it is thread safe
    void RefreshFieldImage();
public:
    void ShowGUI();
    void SetApp(QApplication& app);
    static RobotConfigWindow& GetInstance();

protected:
    QLabel *imageLabel;

    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            // Handle left button press
            Mouse::GetInstance().SetLeftDown(true);
        }

        if (event->button() == Qt::RightButton)
        {
            // Handle right button press
            Mouse::GetInstance().SetRightDown(true);
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            // Handle left button release
            Mouse::GetInstance().SetLeftDown(false);
        }

        if (event->button() == Qt::RightButton)
        {
            // Handle right button release
            Mouse::GetInstance().SetRightDown(false);
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        cv::Point2f imageLabelPos = cv::Point2f(imageLabel->x(), imageLabel->y());
        cv::Point2f absolutePos = cv::Point2f(event->x(), event->y());

        cv::Point2f relativePos;
        SAFE_DRAW
        relativePos = (absolutePos - imageLabelPos) * (float)drawingImage.rows / imageLabel->height();
        END_SAFE_DRAW

        // Set position relative to image label
        Mouse::GetInstance().SetPos(relativePos);
    }

    bool eventFilter(QObject *watched, QEvent *event) override
    {
        // We're interested in KeyPress and KeyRelease events
        if(event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease)
        {
            // Cast the QEvent pointer to a QKeyEvent pointer
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

            // Shift
            if(keyEvent->key() == Qt::Key_Shift)
            {
                if(event->type() == QEvent::KeyPress) {
                    shiftDown = true;
                } else {
                    shiftDown = false;
                }
            }

            // A
            if(keyEvent->key() == Qt::Key_A)
            {
                if(event->type() == QEvent::KeyPress) {
                    aDown = true;
                } else {
                    aDown = false;
                }
            }

            // P
            if(keyEvent->key() == Qt::Key_P)
            {
                if(event->type() == QEvent::KeyPress) {
                    pDown = true;
                } else {
                    pDown = false;
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
};
