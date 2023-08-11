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
#include "Input/Input.h"

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
    QLabel *GetImageLabel();
    
protected:
    QLabel *imageLabel;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    // private ctor since this is a singleton
    RobotConfigWindow();

    QApplication *app;
};
