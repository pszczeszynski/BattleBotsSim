#pragma once
#include <QMutex>
#include <QMouseEvent>
#include <QEvent>
#include <QPoint>
#include <QMap>
#include <opencv2/core/core.hpp>
#include <QLabel>

class Input
{
public:
    static Input &GetInstance();

    bool IsKeyPressed(int keyCode) const;
    cv::Point2f GetMousePosition(bool imageRelative = true) const;
    bool IsMouseOverImage() const;
    bool IsLeftMousePressed() const;
    bool IsRightMousePressed() const;

    void UpdateMousePress(QMouseEvent *event);
    void UpdateMouseRelease(QMouseEvent *event);
    void UpdateMouseMove(QMouseEvent *event);
    void UpdateEventFilter(QObject *watched, QEvent *event);

    void SetImageLabel(QLabel *imageLabel);

private:
    Input();
    Input(const Input &) = delete;
    Input &operator=(const Input &) = delete;

    mutable QMutex _mutex;
    cv::Point2f _mousePos;
    cv::Point2f _imagePos;
    cv::Point2f _imageActualSize;
    bool _leftMousePressed;
    bool _rightMousePressed;
    QMap<int, bool> _keysState;
};
