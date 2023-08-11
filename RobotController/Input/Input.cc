#include "Input.h"
#include <QKeyEvent>
#include "../RobotControllerGUI.h"

// Singleton pattern implementation
Input &Input::GetInstance()
{
    static Input instance;
    return instance;
}

bool Input::IsKeyPressed(int keyCode) const
{
    QMutexLocker locker(&_mutex);
    return _keysState[keyCode];
}

void Input::SetImageLabel(QLabel *imageLabel)
{
    QMutexLocker locker(&_mutex);

    // set image pos
    _imagePos = cv::Point2f(imageLabel->x(), imageLabel->y());
    _imageActualSize = cv::Point2f(imageLabel->width(), imageLabel->height());
}

/**
 * Gets the mouse position relative to either the image label or the screen
 */
cv::Point2f Input::GetMousePosition(bool imageRelative) const
{
    QMutexLocker locker(&_mutex);

    // if not relative to image, return raw mouse position
    if (!imageRelative)
    {
        return _mousePos;
    }

    // if image size is 0, throw exception
    if (_imageActualSize.x == 0 || _imageActualSize.y == 0)
    {
        throw std::runtime_error("Image label size is 0");
    }

    // Convert mouse position to image coordinates
    cv::Point2f relativePos = (_mousePos - _imagePos) * (float)HEIGHT / _imageActualSize.y;

    return relativePos;
}

/**
 * @brief Input::IsMouseOverImage
 * @return true if mouse is over the field image, false otherwise
*/
bool Input::IsMouseOverImage() const
{
    // get mouse position in image coordinates
    cv::Point2f mousePos = GetMousePosition(true);

    // check if mouse is over image
    return mousePos.x >= 0 && mousePos.x <= WIDTH &&
           mousePos.y >= 0 && mousePos.y <= HEIGHT;
}

bool Input::IsLeftMousePressed() const
{
    QMutexLocker locker(&_mutex);
    return _leftMousePressed;
}

bool Input::IsRightMousePressed() const
{
    QMutexLocker locker(&_mutex);
    return _rightMousePressed;
}

void Input::UpdateMousePress(QMouseEvent *event)
{
    QMutexLocker locker(&_mutex);
    if (event->button() == Qt::LeftButton)
    {
        _leftMousePressed = true;
    }
    else if (event->button() == Qt::RightButton)
    {
        _rightMousePressed = true;
    }
    _mousePos = cv::Point2f(event->pos().x(), event->pos().y());
}

void Input::UpdateMouseRelease(QMouseEvent *event)
{
    QMutexLocker locker(&_mutex);
    if (event->button() == Qt::LeftButton)
    {
        _leftMousePressed = false;
    }
    else if (event->button() == Qt::RightButton)
    {
        _rightMousePressed = false;
    }
    _mousePos = cv::Point2f(event->pos().x(), event->pos().y());
}

void Input::UpdateMouseMove(QMouseEvent *event)
{
    QMutexLocker locker(&_mutex);
    _mousePos = cv::Point2f(event->pos().x(), event->pos().y());
}

void Input::UpdateEventFilter(QObject *watched, QEvent *event)
{
    QMutexLocker locker(&_mutex);
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        _keysState[keyEvent->key()] = true;
    }
    else if (event->type() == QEvent::KeyRelease)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        _keysState[keyEvent->key()] = false;
    }
}

Input::Input() : _leftMousePressed(false), _rightMousePressed(false)
{

}