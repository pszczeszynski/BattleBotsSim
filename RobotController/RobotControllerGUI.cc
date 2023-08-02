#include "RobotControllerGUI.h"
#include "RobotClassifier.h"
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include <QLineEdit>
#include "RobotController.h"

// Constants for window layout
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;
const int COLUMN_WIDTH = WINDOW_WIDTH / 4;
const int COLUMN_SPACING = 20;
const int LABEL_HEIGHT = 30;
const int SPINBOX_HEIGHT = 30;
const int SLIDER_HEIGHT = 30;

const int widgetVerticalMargin = 10; // Margin between the label and the spin box or slider
const int widgetHorizontalMargin = 10; // Margin between the window border and the widgets
int nextWidgetY = widgetVerticalMargin; // Vertical position of the next widget


// Helper function to add a labeled spin box
void addLabeledSpinBox(QMainWindow* window, const QString& label, int& value)
{
    QLabel* spinBoxLabel = new QLabel(label, window);
    spinBoxLabel->setGeometry(widgetHorizontalMargin, nextWidgetY, COLUMN_WIDTH, LABEL_HEIGHT);

    QSpinBox* spinBox = new QSpinBox(window);
    spinBox->setGeometry(widgetHorizontalMargin, nextWidgetY + LABEL_HEIGHT, COLUMN_WIDTH, SPINBOX_HEIGHT);
    spinBox->setRange(0, 10000);
    spinBox->setValue(value);  // Set the initial value
    QObject::connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [&](int newValue)
    {
        value = newValue;
    });

    nextWidgetY += LABEL_HEIGHT + SPINBOX_HEIGHT + widgetVerticalMargin; // Increase the vertical position for the next widget
}

// Helper function to add a labeled slider
void addLabeledSlider(QMainWindow* window, const QString& label, int& value, int minValue, int maxValue)
{
    QLabel* sliderLabel = new QLabel(label, window);
    sliderLabel->setGeometry(widgetHorizontalMargin, nextWidgetY, COLUMN_WIDTH, LABEL_HEIGHT);

    QSlider* slider = new QSlider(Qt::Horizontal, window);
    slider->setGeometry(widgetHorizontalMargin, nextWidgetY + LABEL_HEIGHT, COLUMN_WIDTH, SLIDER_HEIGHT);
    slider->setRange(minValue, maxValue);
    slider->setValue(value);  // Set the initial value
    QObject::connect(slider, &QSlider::valueChanged, [&](int newValue)
    {
        value = newValue;
    });

    nextWidgetY += LABEL_HEIGHT + SLIDER_HEIGHT + widgetVerticalMargin; // Increase the vertical position for the next widget
}

const int BUTTON_HEIGHT = 30;
// Helper function to add a toggle button
void addToggleButton(QMainWindow* window, const QString& topLabel, const QString& labelDisabled, const QString& labelEnabled, bool& value)
{
    QLabel* topLabelWidget = new QLabel(topLabel, window);
    topLabelWidget->setGeometry(widgetHorizontalMargin, nextWidgetY, COLUMN_WIDTH, LABEL_HEIGHT);

    QPushButton* toggleButton = new QPushButton(labelDisabled, window);
    toggleButton->setGeometry(widgetHorizontalMargin, nextWidgetY + LABEL_HEIGHT, COLUMN_WIDTH, BUTTON_HEIGHT);
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

    nextWidgetY += LABEL_HEIGHT + BUTTON_HEIGHT + widgetVerticalMargin; // Increase the vertical position for the next widget
}

void addPushButton(QMainWindow* window, const QString& label, std::function<void()> callback)
{
    QPushButton* pushButton = new QPushButton(label, window);
    pushButton->setGeometry(widgetHorizontalMargin, nextWidgetY, COLUMN_WIDTH, BUTTON_HEIGHT);

    // Connect the button clicked signal to the callback function
    QObject::connect(pushButton, &QPushButton::clicked, callback);

    nextWidgetY += BUTTON_HEIGHT + widgetVerticalMargin; // Increase the vertical position for the next widget
}

void addTextInput(QMainWindow* window, const QString& label, QString& value)
{
    QLabel* textInputLabel = new QLabel(label, window);
    textInputLabel->setGeometry(widgetHorizontalMargin, nextWidgetY, COLUMN_WIDTH, LABEL_HEIGHT);

    QLineEdit* textInput = new QLineEdit(window);
    textInput->setGeometry(widgetHorizontalMargin, nextWidgetY + LABEL_HEIGHT, COLUMN_WIDTH, SPINBOX_HEIGHT);
    textInput->setText(value);  // Set the initial value
    QObject::connect(textInput, &QLineEdit::textChanged, [&](const QString& newValue)
    {
        value = newValue;
    });

    nextWidgetY += LABEL_HEIGHT + SPINBOX_HEIGHT + widgetVerticalMargin; // Increase the vertical position for the next widget
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
    addLabeledSlider(this, "Master: Orbit dTheta Degrees:", ORBIT_DTHETA_DEG, 0, 300);
    addLabeledSlider(this, "Opponent Position Extrapolate MS:", OPPONENT_POSITION_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(this, "Master Speed Scale:", MASTER_SPEED_SCALE_PERCENT, 0, 100);

    addToggleButton(this, "Orbitron Starter", "Start", "Stop", IS_RUNNING);

    addTextInput(this, "Save File Name: ", SAVE_FILE_NAME);

    // add save button
    addPushButton(this, "Save", []()
    {
        saveGlobalVariablesToFile(SAVE_FILE_NAME.toStdString());
    });


    // add switch robots button
    addPushButton(this, "Switch Robots", []()
    {
        RobotClassifier::instance->SwitchRobots();
    });


    // Right column: Display OpenCV Mat (passed by reference)
    imageLabel = new QLabel(this);
    imageLabel->setGeometry(RIGHT_COLUMN_X, 10, WINDOW_WIDTH - RIGHT_COLUMN_X - COLUMN_SPACING, WINDOW_HEIGHT - 20);

    SAFE_DRAW
    // init drawing image
    drawingImage = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    // Load an OpenCV Mat and set it as the image in QLabel
    QImage image(drawingImage.data, drawingImage.cols, drawingImage.rows, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(image);
    imageLabel->setPixmap(pixmap.scaled(imageLabel->size(), Qt::KeepAspectRatio));
    END_SAFE_DRAW

    // Install this object as an event filter on the application object
    qApp->installEventFilter(this);


    // enable tracking the mouse even when it isn't pressed
    setMouseTracking(true);
    imageLabel->setMouseTracking(true);

}

/**
 * @brief RobotConfigWindow::RefreshFieldImage
 * Refreshes the field image in the GUI
 * This function is called from the robot controller thread
 * It is scheduled by the RobotController::RefreshFieldImageSignal signal
*/
void RobotConfigWindow::RefreshFieldImage()
{
    // check if drawing image is empty and return if it is
    bool isEmpty = false;
    SAFE_DRAW
    isEmpty = drawingImage.empty();
    END_SAFE_DRAW
    if (isEmpty)
    {
        return;
    }

    SAFE_DRAW
    // otherwise, convert drawing image to QImage and set it as the image in QLabel
    QImage imageQt(drawingImage.data, drawingImage.cols, drawingImage.rows, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(imageQt);
    // Update the imageLabel pixmap whenever drawingImage is updated
    imageLabel->setPixmap(pixmap.scaled(imageLabel->size(), Qt::KeepAspectRatio));
    // finally release lock
    END_SAFE_DRAW
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