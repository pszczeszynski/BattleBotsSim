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
const int CHART_HEIGHT = 250;
const int CHART_HISTORY = 60;

const int widgetVerticalMargin = 5; // Margin between the label and the spin box or slider
const int widgetHorizontalMargin = 10; // Margin between the window border and the widgets
int nextWidgetYLeft = widgetVerticalMargin; // Vertical position of the next widget on the left side
int nextWidgetYRight = widgetVerticalMargin; // Vertical position of the next widget on the right side
int radioCount = 1;

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

/**
 * @brief addAutoUpdatingLabel
 * @param window The window to add the label to
 * @param initialText The initial text to display in the label
 * @param updateTextLambda A lambda function that returns the new text to display in the label
 * @param left Whether to place the label on the left or right side of the window
 * @param refreshIntervalMs The interval in milliseconds to refresh the label text, default is 50 ms
 */
QLabel* addAutoUpdatingLabel(QMainWindow *window, std::function<QString()> updateTextLambda, bool left = true,
                          int refreshIntervalMs = 50, int width = COLUMN_WIDTH, int x_offset = 0)
{
    int shiftAmount = LABEL_HEIGHT + widgetVerticalMargin;

    // if there is an x_offset, assume it's at the last y position
    if (x_offset != 0)
    {
        if (left)
        {
            nextWidgetYLeft -= shiftAmount;
        }
        else
        {
            nextWidgetYRight -= shiftAmount;
        }
    }

    int x = (left ? widgetHorizontalMargin : rightSideX) + x_offset;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;

    QLabel *label = new QLabel("", window);
    label->setGeometry(x, y, width, LABEL_HEIGHT);

    QTimer *timer = new QTimer(window);
    QObject::connect(timer, &QTimer::timeout, [=]()
                     { label->setText(updateTextLambda()); }); // Update the label text with the lambda function

    timer->start(refreshIntervalMs); // Start the timer to update the label every refreshIntervalMs milliseconds


    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += shiftAmount;
    }
    else
    {
        nextWidgetYRight += shiftAmount;
    }

    return label;
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

QLineSeries* addRadioChart(QMainWindow *window, bool left)
{
    int x = left ? widgetHorizontalMargin : rightSideX;
    int y = left ? nextWidgetYLeft : nextWidgetYRight;
    
    QChartView* chartView = new QChartView(window);
    chartView->setGeometry(x, y, COLUMN_WIDTH, CHART_HEIGHT);

    QLineSeries* series = new QLineSeries();

    //for (int i = 0; i < RADIO_HISTORY; i++)
    for (int i = 0; i < CHART_HISTORY; i++)
    {
        series->append(i, 0);
    }

    QChart* chart = new QChart();
    chart->setTheme(QChart::ChartThemeDark);
    chart->addSeries(series);
    chart->setTitle("Radio Connection");
    chart->createDefaultAxes();
    chart->axes(Qt::Horizontal)[0]->setMin(QVariant()); // Set minimum range to auto
    chart->axes(Qt::Horizontal)[0]->setMax(QVariant()); // Set maximum range to auto
    chart->setAnimationOptions(QChart::GridAxisAnimations);
    // chart->setMargins(QMargins(10, 10, 10, 10));
    chartView->setChart(chart);

    
    // if left, increase the vertical position for the next widget
    if (left)
    {
        nextWidgetYLeft += CHART_HEIGHT;
    }
    else
    {
        nextWidgetYRight += CHART_HEIGHT;
    }

    series->remove(0, 100);

    // RobotControllerGUI::GetInstance().SetRadioSeries(series);

    return series;
}

RobotControllerGUI::RobotControllerGUI()
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
    addLabeledSlider(this, "Orbit Angle Extrapolate MS:", ORBIT_ANGLE_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(this, "Kill Angle Extrapolate MS:", GTP_ANGLE_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(this, "DriveToPos Position Extrapolate MS:", POSITION_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(this, "Orbit Radius:", ORBIT_RADIUS, 0, 300);
    addLabeledSlider(this, "PP Radius:", PURE_PURSUIT_RADIUS, 0, 300);
    addLabeledSlider(this, "Orbit Moving Avg Speed (%):", ORBIT_RADIUS_MOVAVG_SPEED, 0, 100);
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


    // Right column: IMU widget
    // imu widget
    _imuWidget = new IMUWidget(this);
    _imuWidget->setGeometry(rightSideX, nextWidgetYRight, COLUMN_WIDTH - 100, COLUMN_WIDTH - 100);
    nextWidgetYRight += COLUMN_WIDTH + widgetVerticalMargin - 100;

    addRpmWidget(this, QString("Front Weapon"), RobotController::GetInstance().GetFrontWeaponTargetPowerRef(), RobotController::GetInstance().GetFrontWeaponTargetPowerRef(), false);
    addRpmWidget(this, QString("Rear Weapon"), RobotController::GetInstance().GetBackWeaponTargetPowerRef(), RobotController::GetInstance().GetBackWeaponTargetPowerRef(), false);

    addPushButton(this, "Angle Invert", []()
    {
        RobotOdometry::Robot().InvertAngle();
    }, false);

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

    addAutoUpdatingLabel(this, []()
    {
        // after 40 ms of no packets, we consider the robot disconnected
        bool connected = RobotController::GetInstance().GetRobotLink().HasReceivedRecentPacket();
        return QString(connected ? "CONNECTED" : "DISCONNECTED");
    }, false, 100, COLUMN_WIDTH / 3, 0);

    addAutoUpdatingLabel(this, [this]()
    {
        return QString("Send packets/sec: " + QString::number(RobotController::GetInstance().GetRobotLink().GetSendFPSThreadSafe()));
    }, false, 100, COLUMN_WIDTH / 3, 0);


    addAutoUpdatingLabel(this, []()
    {
        return QString("Recv Packets/sec: " + QString::number(RobotController::GetInstance().GetRobotLink().GetReceivedFPSThreadSafe()));
    }, false, 100, COLUMN_WIDTH / 3, COLUMN_WIDTH / 3);


    addAutoUpdatingLabel(this, []()
    {
        return QString("Max dropout (ms): " + QString::number((int) (1000 * RobotController::GetInstance().GetRobotLink().GetTimeDiffThreadSafe())));
    }, false, 100, COLUMN_WIDTH / 3, 2 * COLUMN_WIDTH / 3);


    addAutoUpdatingLabel(this, []()
    {
        return QString("Vision FPS: " + QString::number(RobotController::GetInstance().visionClock.getFPS()));
    }, false, 100, COLUMN_WIDTH / 3, 0);

    addAutoUpdatingLabel(this, []()
    {
        return QString("Vision dropout: " + QString::number((int) (1000 * RobotController::GetInstance().visionClock.getMaxTimeDifference())));
    }, false, 100, COLUMN_WIDTH / 3, COLUMN_WIDTH / 3);

    addAutoUpdatingLabel(this, [this]()
    {
        return QString("Display fps: " + QString::number(_displayImageClock.getFPS()));
    }, false, 100, COLUMN_WIDTH / 3, 2 * COLUMN_WIDTH / 3);


    // names are LD, RD, FW, RW
    std::string motorNames[4] = {"LD", "RD", "FW", "RW"};

    for (int i = 0; i < 4; i ++)
    {
        // left drive
        _vescInfo[i][0] = addAutoUpdatingLabel(this, [i, motorNames]()
        {
            return QString((motorNames[i] + " amps: " + std::to_string((int) RobotController::GetInstance().GetCANData().motorCurrent[i])).c_str());
        }, false, 100, COLUMN_WIDTH / 4, 0);

        _vescInfo[i][1] = addAutoUpdatingLabel(this, [i, motorNames]()
        {
            return QString((motorNames[i] + " volts: " + std::to_string((int) RobotController::GetInstance().GetCANData().motorVoltage[i])).c_str());
        }, false, 100, COLUMN_WIDTH / 4, COLUMN_WIDTH / 4);

        _vescInfo[i][2] = addAutoUpdatingLabel(this, [i, motorNames]()
        {
            return QString((motorNames[i] + " rpm: " + std::to_string((int) RobotController::GetInstance().GetCANData().motorRPM[i])).c_str());
        }, false, 100, COLUMN_WIDTH / 4, 2 * COLUMN_WIDTH / 4);

        _vescInfo[i][3] = addAutoUpdatingLabel(this, [i, motorNames]()
        {
            return QString((motorNames[i] + " esctemp: " + std::to_string((int) RobotController::GetInstance().GetCANData().escFETTemp[i])).c_str());
        }, false, 100, COLUMN_WIDTH / 4, 3 * COLUMN_WIDTH / 4);
    }

    // _radioSeries = addRadioChart(this, false);
    // // make it update every 50ms
    // QTimer *timer = new QTimer(this);
    // QObject::connect(timer, &QTimer::timeout, this, &RobotControllerGUI::UpdateRadioSeries);
    // timer->start(50);




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
void RobotControllerGUI::RefreshFieldImage()
{
    // clear all queued events because we don't want to store more than one refresh event at a time
    QCoreApplication::removePostedEvents(this, QEvent::MetaCall);

    cv::Mat drawingImage;
    bool newImage = RobotController::GetInstance().drawingImageQueue.consumeLatestAndClear(drawingImage);
    // if there is no new image, don't do anything
    if (!newImage || drawingImage.empty())
    {
        std::cerr << "RefreshFieldImage called when no drawingImage" << std::endl;
        return;
    }
    _displayImageClock.markStart();

    // otherwise, convert drawing image to QImage and set it as the image in QLabel
    QImage imageQt(drawingImage.data, drawingImage.cols, drawingImage.rows, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(imageQt);
    // Update the imageLabel pixmap whenever drawingImage is updated
    _imageLabel->setPixmap(pixmap.scaled(_imageLabel->size(), Qt::KeepAspectRatio));
}

void RobotControllerGUI::ShowGUI()
{
    show();
}

/**
 * @brief RobotConfigWindow::SetApp
 * Sets the application object for the GUI and sets the application palette to a dark color scheme
*/
void RobotControllerGUI::SetApp(QApplication& app)
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

RobotControllerGUI& RobotControllerGUI::GetInstance()
{
    static RobotControllerGUI instance;
    return instance;
}

QLabel* RobotControllerGUI::GetImageLabel()
{
    return _imageLabel;
}

QChart* RobotControllerGUI::GetRadioChart()
{
    return _radioChart;
}

void RobotControllerGUI::SetRadioSeries(QLineSeries* series)
{
    _radioSeries = series;
}

// Function to provide access to the vescInfo array
QLabel* RobotControllerGUI::GetVescInfo(int motor, int dataType)
{
    if (motor >= 0 && motor < MOTOR_COUNT && dataType >= 0 && dataType < 4)
    {
        return _vescInfo[motor][dataType];
    }
    else
    {
        // Return nullptr or handle the out-of-bounds condition
        return nullptr;
    }
}

void RobotControllerGUI::UpdateRadioSeries()
{
    bool received = RobotController::GetInstance().GetRobotLink().HasReceivedRecentPacket();
    // std::cout << "Here" << std::endl;
    // std::cout << radioCount << std::endl;
    // std::cout << _radioSeries->count() << std::endl;
    _radioSeries->remove(0);

    if (received) 
        _radioSeries->append(radioCount%CHART_HISTORY, 0.5);
    else
        _radioSeries->append(radioCount%CHART_HISTORY, 0);
    radioCount++;
}

// events
void RobotControllerGUI::mousePressEvent(QMouseEvent *event)
{
    Input::GetInstance().UpdateMousePress(event);
}

void RobotControllerGUI::mouseReleaseEvent(QMouseEvent *event)
{
    Input::GetInstance().UpdateMouseRelease(event);
}

void RobotControllerGUI::mouseMoveEvent(QMouseEvent *event)
{
    Input::GetInstance().UpdateMouseMove(event);
}

bool RobotControllerGUI::eventFilter(QObject *watched, QEvent *event)
{
    Input::GetInstance().UpdateEventFilter(watched, event);
    return QMainWindow::eventFilter(watched, event);
}






////// RADIO CHART ///////

ChartWindow::ChartWindow()
{
    setWindowTitle("Radio Chart");
    setWindowFlags(windowFlags() | Qt::WindowMinimizeButtonHint);
    setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT/2 + 120);


    QChartView* radioChartView = new QChartView(this);
    radioChartView->setGeometry(10, 10, WINDOW_WIDTH/2 - 30, WINDOW_HEIGHT/2 + 100);

    _receiveSeries = new QLineSeries();

    _receiveSeries->append(0, 0);
    // _receiveSeries->setColor(Qt::green);

    
    _sendSeries = new QLineSeries();

    _sendSeries->append(0, 0);
    _sendSeries->setColor(Qt::red);

    _radioChart = new QChart();
    _radioChart->setTheme(QChart::ChartThemeDark);
    _radioChart->addSeries(_receiveSeries);
    _radioChart->addSeries(_sendSeries);
    _radioChart->setTitle("Radio Connection");
    _radioChart->createDefaultAxes();

    _radioChart->legend()->markers()[0]->setLabel("Received");
    _radioChart->legend()->markers()[1]->setLabel("Sent");

    _radioAxisMax = CHART_HISTORY;
    _radioChart->axes(Qt::Horizontal)[0]->setMin(0);
    _radioChart->axes(Qt::Horizontal)[0]->setMax(_radioAxisMax);
    _radioChart->axes(Qt::Horizontal)[0]->setTitleText("Time (s)");

    _radioChart->axes(Qt::Vertical)[0]->setMin(0);
    _radioChart->axes(Qt::Vertical)[0]->setMax(225);
    _radioChart->axes(Qt::Vertical)[0]->setTitleText("Packets/s");

    radioChartView->setChart(_radioChart);

    // make it update every 50ms
    QTimer *radioTimer = new QTimer(this);
    QObject::connect(radioTimer, &QTimer::timeout, this, &ChartWindow::UpdateRadioSeries);
    radioTimer->start(50);



    
    QChartView* ampChartView = new QChartView(this);
    ampChartView->setGeometry(WINDOW_WIDTH/2 + 10, 10, WINDOW_WIDTH/2 - 30, WINDOW_HEIGHT/2 + 100);

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        _ampSeries[i] = new QLineSeries();
        _ampSeries[i]->append(0, 0);
    }

    _ampSeries[1]->setColor(Qt::blue);
    _ampSeries[2]->setColor(Qt::red);
    _ampSeries[3]->setColor(Qt::darkYellow);

    _ampChart = new QChart();
    _ampChart->setTheme(QChart::ChartThemeDark);
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        _ampChart->addSeries(_ampSeries[i]);
    }
    _ampCount = 1;

    _ampChart->setTitle("Motor Currents");
    _ampChart->createDefaultAxes();

    _ampChart->legend()->markers()[0]->setLabel("Left Drive");
    _ampChart->legend()->markers()[1]->setLabel("Right Drive");
    _ampChart->legend()->markers()[2]->setLabel("Front Weapon");
    _ampChart->legend()->markers()[3]->setLabel("Rear Weapon");

    _ampAxisMax = CHART_HISTORY;
    _ampChart->axes(Qt::Horizontal)[0]->setMin(0);
    _ampChart->axes(Qt::Horizontal)[0]->setMax(_ampAxisMax);
    _ampChart->axes(Qt::Horizontal)[0]->setTitleText("Time (s)");

    _ampChart->axes(Qt::Vertical)[0]->setMin(0);
    _ampChart->axes(Qt::Vertical)[0]->setMax(225);
    _ampChart->axes(Qt::Vertical)[0]->setTitleText("Curent (A)");

    ampChartView->setChart(_ampChart);

    // make it update every 50ms
    QTimer *ampTimer = new QTimer(this);
    QObject::connect(ampTimer, &QTimer::timeout, this, &ChartWindow::UpdateAmpSeries);
    ampTimer->start(50);

}

void ChartWindow::ShowGUI()
{
    show();
}

/**
 * @brief RadioChartWindow::SetApp
 * Sets the application object for the GUI and sets the application palette to a dark color scheme
*/
void ChartWindow::SetApp(QApplication& app)
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

ChartWindow& ChartWindow::GetInstance()
{
    static ChartWindow instance;
    return instance;
}

QChart* ChartWindow::GetRadioChart()
{
    return _radioChart;
}

void ChartWindow::UpdateRadioSeries()
{
    if (radioCount * AXIS_CONVERSION >= CHART_HISTORY)
    {
        _receiveSeries->remove(0);
        _sendSeries->remove(0);
    }

    _receiveSeries->append(radioCount * AXIS_CONVERSION, RobotController::GetInstance().GetRobotLink().GetReceivedFPSThreadSafe());
    _sendSeries->append(radioCount * AXIS_CONVERSION, RobotController::GetInstance().GetRobotLink().GetSendFPSThreadSafe());

    if (radioCount * AXIS_CONVERSION >= _radioAxisMax)
    {
        _radioAxisMax += AXIS_ADVANCE;
        _radioChart->axes(Qt::Horizontal)[0]->setMin(_radioAxisMax - CHART_HISTORY);
        _radioChart->axes(Qt::Horizontal)[0]->setMax(_radioAxisMax);
    }

    radioCount++;
}


void ChartWindow::UpdateAmpSeries()
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (_ampCount * AXIS_CONVERSION >= CHART_HISTORY)
            _ampSeries[i]->remove(0);

        _ampSeries[i]->append(_ampCount * AXIS_CONVERSION*2, RobotController::GetInstance().GetCANData().motorCurrent[i]);
    }

    if (_ampCount * AXIS_CONVERSION >= _ampAxisMax)
    {
        _ampAxisMax += AXIS_ADVANCE;
        _ampChart->axes(Qt::Horizontal)[0]->setMin(_ampAxisMax - CHART_HISTORY);
        _ampChart->axes(Qt::Horizontal)[0]->setMax(_ampAxisMax);
    }

    _ampCount++;
}