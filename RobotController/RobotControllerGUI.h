#pragma once

#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <opencv2/opencv.hpp>
#include "RobotConfig.h"

class RobotConfigWindow 
{
public:
    RobotConfigWindow();
    void RefreshFieldImage();
    void ShowGUI();
    void SetApp(QApplication& app);

private:
    QApplication *app;
    QLabel *imageLabel;
    QMainWindow window;
};
