#include "RobotControllerGUI.h"
#include "RobotClassifier.h"
#include "RobotController.h"
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include <QLineEdit>
#include <QProgressBar>
#include <QLabel>
#include <QPalette>
#include <QTimer>

// Constants for window layout
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1015;
const int COLUMN_WIDTH = WINDOW_WIDTH / 5;
const int COLUMN_SPACING = 20;
const int LABEL_HEIGHT = 30;
const int SPINBOX_HEIGHT = 30;
const int SLIDER_HEIGHT = 30;

const int widgetVerticalMargin = 10; // Margin between the label and the spin box or slider
const int widgetHorizontalMargin = 10; // Margin between the window border and the widgets
int nextWidgetYLeft = widgetVerticalMargin; // Vertical position of the next widget on the left side
int nextWidgetYRight = widgetVerticalMargin; // Vertical position of the next widget on the right side

const int rightSideX = WINDOW_WIDTH - COLUMN_WIDTH - widgetHorizontalMargin - 50;

/**
 * @brief addLabeledSpinBox
 * @param window The window to add the spin box to
 * @param label The label to display above to the spin box
 * @param value The value to display in the spin box
 * @param left Whether to place the spin box on the left or right side of the window
*/
void addLabeledSpinBox(QMainWindow* window, const QString& label, int& value, bool left = true)
{
    int x = left ? widgetHorizontalMargin : rightSideX;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;

    QLabel* spinBoxLabel = new QLabel(label, window);
    spinBoxLabel->setGeometry(x, y, COLUMN_WIDTH, LABEL_HEIGHT);

    QSpinBox* spinBox = new QSpinBox(window);
    spinBox->setGeometry(x, y + LABEL_HEIGHT, COLUMN_WIDTH, SPINBOX_HEIGHT);
    spinBox->setRange(0, 10000);
    spinBox->setValue(value);  // Set the initial value
    // When the spin box value changes, update the value variable
    QObject::connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [&](int newValue)
                     { value = newValue; });

    int shiftAmount = LABEL_HEIGHT + SPINBOX_HEIGHT + widgetVerticalMargin;

    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += shiftAmount;
    }
    else
    {
        nextWidgetYRight += shiftAmount;
    }
}

// Helper function to add a labeled slider
void addLabeledSlider(QMainWindow* window, const QString& label, int& value, int minValue, int maxValue, bool left = true)
{
    int x = left ? widgetHorizontalMargin : rightSideX;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;

    QLabel* sliderLabel = new QLabel(label, window);
    sliderLabel->setGeometry(x, y, COLUMN_WIDTH, LABEL_HEIGHT);

    QSlider* slider = new QSlider(Qt::Horizontal, window);
    slider->setGeometry(x, y + LABEL_HEIGHT, COLUMN_WIDTH, SLIDER_HEIGHT);
    slider->setRange(minValue, maxValue);
    slider->setValue(value);  // Set the initial value
    QObject::connect(slider, &QSlider::valueChanged, [&](int newValue)
    {
        value = newValue;
    });

    int shiftAmount = LABEL_HEIGHT + SLIDER_HEIGHT + widgetVerticalMargin;

    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += shiftAmount;
    }
    else
    {
        nextWidgetYRight += shiftAmount;
    }
}

const int BUTTON_HEIGHT = 30;
// Helper function to add a toggle button
void addToggleButton(QMainWindow* window, const QString& topLabel, const QString& labelDisabled, const QString& labelEnabled, bool& value, bool left = true)
{
    int x = left ? widgetHorizontalMargin : rightSideX;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;

    QLabel* topLabelWidget = new QLabel(topLabel, window);
    topLabelWidget->setGeometry(x, y, COLUMN_WIDTH, LABEL_HEIGHT);

    QPushButton* toggleButton = new QPushButton(labelDisabled, window);
    toggleButton->setGeometry(x, y + LABEL_HEIGHT, COLUMN_WIDTH, BUTTON_HEIGHT);
    toggleButton->setText(value ? labelEnabled : labelDisabled);  // Set the initial text

    // Create dynamic memory for the captured variables
    QString *enabledText = new QString(labelEnabled);
    QString *disabledText = new QString(labelDisabled);

    // Function to toggle the button state and update the button text
    auto toggleButtonState = [toggleButton, enabledText, disabledText, &value]()
    {
        value = !value;
        if (value)
        {
            toggleButton->setText(*enabledText);
        }
        else
        {
            toggleButton->setText(*disabledText);
        }
    };

    // Connect the button clicked signal to the toggleButtonState function
    QObject::connect(toggleButton, &QPushButton::clicked, toggleButtonState);

    int shiftAmount = LABEL_HEIGHT + BUTTON_HEIGHT + widgetVerticalMargin;

    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += shiftAmount;
    }
    else
    {
        nextWidgetYRight += shiftAmount;
    }
}

void addPushButton(QMainWindow* window, const QString& label, std::function<void()> callback, bool left = true)
{
    int x = left ? widgetHorizontalMargin : rightSideX;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;

    QPushButton* pushButton = new QPushButton(label, window);
    pushButton->setGeometry(x, y, COLUMN_WIDTH, BUTTON_HEIGHT);

    // Connect the button clicked signal to the callback function
    QObject::connect(pushButton, &QPushButton::clicked, callback);


    int shiftAmount = BUTTON_HEIGHT + widgetVerticalMargin;

    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += shiftAmount;
    }
    else
    {
        nextWidgetYRight += shiftAmount;
    }
}

void addTextInput(QMainWindow* window, const QString& label, QString& value, bool left = true)
{
    int x = left ? widgetHorizontalMargin : rightSideX;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;

    QLabel* textInputLabel = new QLabel(label, window);
    textInputLabel->setGeometry(x, y, COLUMN_WIDTH, LABEL_HEIGHT);

    QLineEdit* textInput = new QLineEdit(window);
    textInput->setGeometry(x, y + LABEL_HEIGHT, COLUMN_WIDTH, SPINBOX_HEIGHT);
    textInput->setText(value);  // Set the initial value
    QObject::connect(textInput, &QLineEdit::textChanged, [&](const QString& newValue)
    {
        value = newValue;
    });

    int shiftAmount = LABEL_HEIGHT + SPINBOX_HEIGHT + widgetVerticalMargin * 2;

    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += shiftAmount;
    }
    else
    {
        nextWidgetYRight = shiftAmount;
    }
}

const int RPM_WIDGET_HEIGHT = LABEL_HEIGHT * 2;
const int RPM_WIDGET_WIDTH = COLUMN_WIDTH * 0.9;
const int MAX_RPM = 4000;
void addRpmWidget(QMainWindow *window, QString labelString, float &targetRpm, float &currentRpm, bool left = true)
{
    int x = left ? widgetHorizontalMargin : rightSideX;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;

    QLabel *label = new QLabel(labelString, window);
    label->setGeometry(x, y, RPM_WIDGET_WIDTH, LABEL_HEIGHT);

    TargetRpmProgressBar  *rpmProgressBar = new TargetRpmProgressBar(window);
    rpmProgressBar->setGeometry(x, y + LABEL_HEIGHT, RPM_WIDGET_WIDTH, RPM_WIDGET_HEIGHT - LABEL_HEIGHT);
    rpmProgressBar->setRange(0, MAX_RPM);
    rpmProgressBar->setValue(currentRpm * MAX_RPM);
    rpmProgressBar->SetTargetRpm(targetRpm);
    // make it show the format RPM: value instead of just the value
    rpmProgressBar->setFormat(QString("RPM: %1").arg(currentRpm));

    QPalette palette = rpmProgressBar->palette();
    if (currentRpm < targetRpm * 0.9)
    {
        palette.setColor(QPalette::Highlight, Qt::red);
    }
    else
    {
        palette.setColor(QPalette::Highlight, Qt::green);
    }
    rpmProgressBar->setPalette(palette);

    auto updateRpmWidget = [label, rpmProgressBar, &targetRpm, &currentRpm]()
    {
        rpmProgressBar->setValue(currentRpm * MAX_RPM);
        rpmProgressBar->SetTargetRpm(targetRpm * MAX_RPM);

        QPalette palette = rpmProgressBar->palette();
        if (currentRpm < targetRpm)
        {
            palette.setColor(QPalette::Highlight, Qt::red);
        }
        else
        {
            palette.setColor(QPalette::Highlight, Qt::green);
        }
        rpmProgressBar->setPalette(palette);
    };

    int shiftAmount = RPM_WIDGET_HEIGHT + widgetVerticalMargin * 2;

    // make it update every 100ms
    QTimer *timer = new QTimer(window);
    QObject::connect(timer, &QTimer::timeout, updateRpmWidget);
    timer->start(15);

    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += shiftAmount;
    }
    else
    {
        nextWidgetYRight += shiftAmount;
    }
}

RobotConfigWindow::RobotConfigWindow()
{
    setWindowTitle("Orbitron Hub");
    setWindowFlags(windowFlags() | Qt::WindowMinimizeButtonHint);
    setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT);

    const int LEFT_COLUMN_X = COLUMN_SPACING;
    const int RIGHT_COLUMN_X = LEFT_COLUMN_X + COLUMN_WIDTH + COLUMN_SPACING;

    loadGlobalVariablesFromFile(SAVE_FILE_NAME.toStdString());

    addLabeledSpinBox(this, "DriveToPos Turn Thresh 1 Deg: ", TURN_THRESH_1_DEG);
    addLabeledSpinBox(this, "DriveToPos Turn Thresh 2 Deg: ", TURN_THRESH_2_DEG);
    addLabeledSpinBox(this, "DriveToPos Max Turn Power (%): ", MAX_TURN_POWER_PERCENT);
    addLabeledSpinBox(this, "DriveToPos Min Turn Power (%): ", MIN_TURN_POWER_PERCENT);
    addLabeledSpinBox(this, "DriveToPos Scale Down Movement (%): ", SCALE_DOWN_MOVEMENT_PERCENT);
    addLabeledSlider(this, "DriveToPos Angle Extrapolate MS:", ANGLE_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(this, "DriveToPos Position Extrapolate MS:", POSITION_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(this, "Master: Orbit Radius:", ORBIT_RADIUS, 0, 300);
    addLabeledSlider(this, "Master: PP Radius:", PURE_PURSUIT_RADIUS, 0, 300);
    addLabeledSlider(this, "Master: Orbit dTheta Degrees:", ORBIT_DTHETA_DEG, 0, 300);
    addLabeledSlider(this, "Opponent Position Extrapolate MS:", OPPONENT_POSITION_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(this, "Master Speed Scale:", MASTER_SPEED_SCALE_PERCENT, 0, 100);

    addTextInput(this, "Save File Name: ", SAVE_FILE_NAME, true);

    // add save button
    addPushButton(this, "Save", []()
    {
        saveGlobalVariablesToFile(SAVE_FILE_NAME.toStdString());
    }, true);


    // add switch robots button
    addPushButton(this, "Switch Robots", []()
    {
        RobotClassifier::instance->SwitchRobots();
    });


    // Middle column: Display OpenCV Mat (passed by reference)
    _imageLabel = new QLabel(this);
    _imageLabel->setGeometry(RIGHT_COLUMN_X, 10, WINDOW_HEIGHT - 20, WINDOW_HEIGHT - 20);


    // right column buttons
    addPushButton(this, "Angle Invert", []()
    {
        RobotOdometry::Robot().InvertAngle();
    }, false);

    addRpmWidget(this, QString("Front Weapon"), RobotController::GetInstance().GetFrontWeaponTargetPowerRef(), RobotController::GetInstance().GetFrontWeaponTargetPowerRef(), false);
    addRpmWidget(this, QString("Rear Weapon"), RobotController::GetInstance().GetBackWeaponTargetPowerRef(), RobotController::GetInstance().GetBackWeaponTargetPowerRef(), false);

    SAFE_DRAW
    // init drawing image
    drawingImage = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    // Load an OpenCV Mat and set it as the image in QLabel
    QImage image(drawingImage.data, WIDTH, HEIGHT, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(image);
    _imageLabel->setPixmap(pixmap.scaled(_imageLabel->size(), Qt::KeepAspectRatio));
    END_SAFE_DRAW

    // Install this object as an event filter on the application object
    qApp->installEventFilter(this);


    // enable tracking the mouse even when it isn't pressed
    setMouseTracking(true);
    _imageLabel->setMouseTracking(true);

    Input::GetInstance().SetImageLabel(_imageLabel);
}

/**
 * @brief RobotConfigWindow::RefreshFieldImage
 * Refreshes the field image in the GUI
 * This function is called from the robot controller thread
 * It is scheduled by the RobotController::RefreshFieldImageSignal signal
*/
void RobotConfigWindow::RefreshFieldImage()
{
    cv::Mat drawingImage;
    DRAWING_IMAGE_MUTEX.lock();
    drawingImage = P_DRAWING_IMAGE.clone();
    CAN_DRAW = true;
    DRAWING_IMAGE_MUTEX.unlock();


    // check if drawing image is empty and return if it is
    bool isEmpty = false;
    isEmpty = drawingImage.empty();
    if (isEmpty)
    {
        return;
    }

    // otherwise, convert drawing image to QImage and set it as the image in QLabel
    QImage imageQt(drawingImage.data, drawingImage.cols, drawingImage.rows, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(imageQt);
    // Update the imageLabel pixmap whenever drawingImage is updated
    _imageLabel->setPixmap(pixmap.scaled(_imageLabel->size(), Qt::KeepAspectRatio));
}

void RobotConfigWindow::ShowGUI()
{
    show();
}

void RobotConfigWindow::SetApp(QApplication& app)
{
    // Set the application palette to a dark color scheme
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    app.setPalette(darkPalette);

    app.setStyle("Fusion");

    this->app = &app;
}

RobotConfigWindow& RobotConfigWindow::GetInstance()
{
    static RobotConfigWindow instance;
    return instance;
}

QLabel* RobotConfigWindow::GetImageLabel()
{
    return _imageLabel;
}

// events
void RobotConfigWindow::mousePressEvent(QMouseEvent *event)
{
    Input::GetInstance().UpdateMousePress(event);
}

void RobotConfigWindow::mouseReleaseEvent(QMouseEvent *event)
{
    Input::GetInstance().UpdateMouseRelease(event);
}

void RobotConfigWindow::mouseMoveEvent(QMouseEvent *event)
{
    Input::GetInstance().UpdateMouseMove(event);
}

bool RobotConfigWindow::eventFilter(QObject *watched, QEvent *event)
{
    Input::GetInstance().UpdateEventFilter(watched, event);
    return QMainWindow::eventFilter(watched, event);
}


