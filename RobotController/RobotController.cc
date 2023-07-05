#include "RobotController.h"
#include "ServerSocket.h"
#include "RobotStateParser.h"
#include "MathUtils.h"
#include <QApplication>
#include <QMainWindow>

#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include <QThread>
#include <QLineEdit>
#include "RobotConfig.h"

// #define ENABLE_TIMERS
#ifdef ENABLE_TIMERS
    #define TIMER_INIT Clock c;
    #define TIMER_START c.markStart();
    #define TIMER_PRINT(msg) std::cout << msg << " time: " << c.getElapsedTime() << std::endl;
#else
    #define TIMER_INIT 
    #define TIMER_START 
    #define TIMER_PRINT(msg)
#endif

#ifdef ENABLE_VISION
#include "Vision.h"
#include <opencv2/core.hpp>
#endif

#include "Extrapolator.h"
#define WIDTH 1280
#define HEIGHT 720

cv::Mat drawingImage = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

QString SAVE_FILE_NAME = "RobotConfig.txt";

// Constants for window layout
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;
const int COLUMN_WIDTH = WINDOW_WIDTH / 4;
const int COLUMN_SPACING = 20;
const int LABEL_HEIGHT = 30;
const int SPINBOX_HEIGHT = 30;
const int SLIDER_HEIGHT = 30;

int widgetVerticalMargin = 10; // Margin between the label and the spin box or slider
int widgetHorizontalMargin = 10; // Margin between the window border and the widgets
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

QLabel *imageLabel;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Set the dark theme style
    app.setStyle("Fusion");

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


    QMainWindow window;
    window.setWindowTitle("Orbitron Hub");
    window.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT);

    const int LEFT_COLUMN_X = COLUMN_SPACING;
    const int RIGHT_COLUMN_X = LEFT_COLUMN_X + COLUMN_WIDTH + COLUMN_SPACING;

    loadGlobalVariablesFromFile(SAVE_FILE_NAME.toStdString());

    addLabeledSpinBox(&window, "DriveToPos Turn Thresh 1 Deg: ", TURN_THRESH_1_DEG);
    addLabeledSpinBox(&window, "DriveToPos Turn Thresh 2 Deg: ", TURN_THRESH_2_DEG);
    addLabeledSpinBox(&window, "DriveToPos Max Turn Power (%): ", MAX_TURN_POWER_PERCENT);
    addLabeledSpinBox(&window, "DriveToPos Min Turn Power (%): ", MIN_TURN_POWER_PERCENT);
    addLabeledSpinBox(&window, "DriveToPos Scale Down Movement (%): ", SCALE_DOWN_MOVEMENT_PERCENT);
    addLabeledSlider(&window, "DriveToPos Angle Extrapolate MS:", ANGLE_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(&window, "DriveToPos Position Extrapolate MS:", POSITION_EXTRAPOLATE_MS, 0, 1000);
    addLabeledSlider(&window, "DriveToPos Angle Integral Factor:", ANGLE_INTEGRAL_FACTOR, 0, 1000);
    addLabeledSpinBox(&window, "DriveToPos Max integral sum:", MAX_ANGLE_INTEGRAL_SUM_DEG);
    addLabeledSlider(&window, "Master: Orbit Radius:", ORBIT_RADIUS, 0, 300);
    addLabeledSlider(&window, "Master: Orbit dTheta Degrees:", ORBIT_DTHETA_DEG, 0, 300);
    addLabeledSlider(&window, "Opponent Position Extrapolate MS:", OPPONENT_POSITION_EXTRAPOLATE_MS, 0, 1000);

    addToggleButton(&window, "Orbitron Starter", "Start", "Stop", IS_RUNNING);

    addTextInput(&window, "Save File Name: ", SAVE_FILE_NAME);

    // add save button
    addPushButton(&window, "Save", []()
    {
        saveGlobalVariablesToFile(SAVE_FILE_NAME.toStdString());
    });



    // Right column: Display OpenCV Mat (passed by reference)
    imageLabel = new QLabel(&window);
    imageLabel->setGeometry(RIGHT_COLUMN_X, 10, WINDOW_WIDTH - RIGHT_COLUMN_X - COLUMN_SPACING, WINDOW_HEIGHT - 20);

    RobotController rc;

    // Load an OpenCV Mat and set it as the image in QLabel
    QImage image(drawingImage.data, drawingImage.cols, drawingImage.rows, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(image);
    imageLabel->setPixmap(pixmap.scaled(imageLabel->size(), Qt::KeepAspectRatio));

    window.show();

    // Create a separate thread for RobotController
    QThread controllerThread;
    rc.moveToThread(&controllerThread);

    QObject::connect(&controllerThread, &QThread::started, &rc, &RobotController::Run);

    controllerThread.start();

    return app.exec();
}

void updatePixmap(const cv::Mat& drawingImage)
{
    QImage image(drawingImage.data, drawingImage.cols, drawingImage.rows, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(image);
    // Update the imageLabel pixmap whenever drawingImage is updated
    imageLabel->setPixmap(pixmap.scaled(imageLabel->size(), Qt::KeepAspectRatio));
}

RobotController::RobotController()
    : socket{"11115"}
#ifdef ENABLE_VISION
#ifdef SIMULATION
      ,overheadCamL_sim{"overheadCamL"}
    //   ,overheadCamR_sim{"overheadCamR"}
      ,vision{overheadCamL_sim}
#else
      ,overheadCamL_real{0}
    //   ,overheadCamR_real{1}
      ,vision{overheadCamL_real} // overheadCamR_real
#endif
#endif
{

}



void RobotController::Run()
{
    TIMER_INIT
    std::cout << "running" << std::endl;
    Clock lastTime;
    lastTime.markStart();

    unsigned long frames = 0;
    // receive until the peer closes the connection
    while (true)
    {

#ifdef SIMULATION
        // 1. receive state info from unity
        std::string received = socket.receive();
        if (received == "")
        {
            continue;
        }
        frames ++;
        if (frames % 100 == 0)
        {
            std::cout << "fps: " << frames / lastTime.getElapsedTime() << std::endl;
            frames = 0;
            lastTime.markStart();
        }

        // 2. parse state info
        RobotState state = RobotStateParser::parse(received);
#endif

#ifdef ENABLE_VISION
#ifdef SIMULATION
        vision.angle = state.robot_orientation * TO_RAD;
        vision.opponent_angle = state.opponent_orientation * TO_RAD;
        vision.position = cv::Point2f(state.robot_position.x + 20, -state.robot_position.z + 13) * 30;
        vision.opponent_position = cv::Point2f(state.opponent_position.x + 20, -state.opponent_position.z + 13) * 30;
#endif

        // TIMER_START
        // vision.runPipeline();
        // TIMER_PRINT("vision.runPipeline()")

        // char key = cv::waitKey(1);
#endif

        
        // // get birds eye view image
        // TIMER_START
        // drawingImage = (cv::Mat&) vision.GetBirdsEyeImage().clone();
        // TIMER_PRINT("Cloning the drawing image")

#ifdef SIMULATION

        drawingImage = cv::Mat::zeros(720, 1280, CV_8UC3);
        // 3. run our robot controller loop
        TIMER_START
        RobotControllerMessage response = loop(state, drawingImage);
        TIMER_PRINT("loop()")

        // check if space pressed
        // if (key == ' ')
        // {
        //     pause = !pause;
        // }

        if (!IS_RUNNING)
        {
            response.drive_amount = 0;
            response.turn_amount = 0;
        }

        TIMER_START
        // send the response back to unity (tell it how much to drive and turn)
        socket.reply_to_last_sender(RobotStateParser::serialize(response));
        TIMER_PRINT("Reply to last sender")

#endif
        TIMER_START
        // cv::imshow("drawing", drawingImage);
        updatePixmap(drawingImage);
        TIMER_PRINT("imshow()")
    }
}

/**
 * Uses the error between a currentPos and targetPos
 * and returns a power to drive towards the target at (with 2 thresholds)
 * 
 * @param error The target position
 * @param threshold1 The first threshold (full power)
 * @param threshold2 The second threshold (min power)
 * @param minPower The minimum power to drive at
*/
double doubleThreshToTarget(double error,
                         double threshold1, double threshold2,
                         double minPower, double maxPower)
{
    double distance = std::abs(error);
    double ret = 0;

    if (distance >= threshold1)
    {
        // Move towards the target with full power
        ret = maxPower;
    }
    else if (distance < threshold1 && distance > threshold2)
    {
        // Scale linearly from maxPower to minPower
        ret = ((distance - threshold2) / (threshold1 - threshold2)) * (maxPower - minPower) + minPower;
    }
    else
    {
        // Scale linearly from minPower to 0
        ret = (distance / threshold2) * minPower;
    }

    // invert if we need to
    if (error < 0)
    {
        ret *= -1;
    }

    return ret;
}

bool danger = false;

/**
 * Drive the robot to a specified position
 * @param targetPos The target position to drive the robot to
 * @param state The current state of the robot and opponent
 * @param drawingImage The image for drawing debugging information
 * @return The response to send back to Unity
 */
RobotControllerMessage RobotController::driveToPosition(const cv::Point2f currPos, const double currAngle, const cv::Point2f &targetPos, const RobotState &state, cv::Mat &drawingImage, bool chooseNewTarget = false)
{
    static Extrapolator<double> currAngleExtrapolator{0};
    static Extrapolator<cv::Point2f> currPositionExtrapolator{cv::Point2f(0, 0)};
    static double angleIntegralSum = 0;
    static Clock c;
    static bool goToOtherTarget = false;

    double deltaTime = c.getElapsedTime();
    c.markStart();

    currAngleExtrapolator.SetValue(currAngle);
    double currAngleEx = currAngleExtrapolator.Extrapolate(ANGLE_EXTRAPOLATE_MS / 1000.0);
    currPositionExtrapolator.SetValue(currPos);
    cv::Point2f currPosEx = currPositionExtrapolator.Extrapolate(POSITION_EXTRAPOLATE_MS / 1000.0);

    // draw arrow from our position at our angle
    cv::Point2f arrowEnd = currPos + cv::Point2f(100.0 * cos(currAngle), 100.0 * sin(currAngle));
    cv::arrowedLine(drawingImage, currPos, arrowEnd, cv::Scalar(0, 0, 255), 2);
    // draw different colored arrow from our position at extrapolated angle
    cv::Point2f arrowEndEx = currPosEx + cv::Point2f(100.0 * cos(currAngleEx), 100.0 * sin(currAngleEx));
    cv::arrowedLine(drawingImage, currPosEx, arrowEndEx, cv::Scalar(255, 100, 0), 1);

    // draw our position
    cv::circle(drawingImage, currPos, 5, cv::Scalar(0,0,255), 2);
    // draw circle with dotted line at extrapolated position
    cv::circle(drawingImage, currPosEx, 5, cv::Scalar(255,100,0), 1);

    double angleToTarget1 = atan2(targetPos.y - currPosEx.y, targetPos.x - currPosEx.x);
    double angleToTarget2 = angle_wrap(angleToTarget1 + M_PI);
    double deltaAngleRad1 = angle_wrap(angleToTarget1 - currAngleEx);
    double deltaAngleRad2 = angle_wrap(angleToTarget2 - currAngleEx);
    double deltaAngleRad1_noex = angle_wrap(angleToTarget1 - currAngle);
    double deltaAngleRad2_noex = angle_wrap(angleToTarget2 - currAngle);
    if (chooseNewTarget)
    {
        goToOtherTarget = abs(deltaAngleRad1_noex) < abs(deltaAngleRad2_noex);
    }

    double deltaAngleRad = goToOtherTarget ? deltaAngleRad1 : deltaAngleRad2;
    double deltaAngleRad_noex = goToOtherTarget ? deltaAngleRad1_noex : deltaAngleRad2_noex;

    // display the integral sum
    cv::putText(drawingImage, "Integral sum: " + std::to_string(angleIntegralSum), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    // increment the integral sum
    angleIntegralSum += deltaAngleRad_noex * deltaTime;

    // cap the integral sum at a max value
    if (angleIntegralSum > MAX_ANGLE_INTEGRAL_SUM_DEG * TO_RAD)
    {
        angleIntegralSum = MAX_ANGLE_INTEGRAL_SUM_DEG * TO_RAD;
    }
    else if (angleIntegralSum < -MAX_ANGLE_INTEGRAL_SUM_DEG * TO_RAD)
    {
        angleIntegralSum = -MAX_ANGLE_INTEGRAL_SUM_DEG * TO_RAD;
    }

    if (!IS_RUNNING)
    {
        angleIntegralSum = 0;
    }

    // increase the error by a factor of the integral sum
    deltaAngleRad += ANGLE_INTEGRAL_FACTOR * angleIntegralSum / 1000.0;

    RobotControllerMessage response{0, 0};
    response.turn_amount = doubleThreshToTarget(deltaAngleRad, TURN_THRESH_1_DEG * TO_RAD,
                                                TURN_THRESH_2_DEG * TO_RAD, MIN_TURN_POWER_PERCENT / 100.0, MAX_TURN_POWER_PERCENT / 100.0);

    // thrash angle
    // response.turn_amount = deltaAngleRad > 0 ? 1.0 : -1.0;
    // if (abs(deltaAngleRad) < 10 * TO_RAD)
    // {
    //     response.turn_amount = 0;
    // }

    double scaleDownMovement = SCALE_DOWN_MOVEMENT_PERCENT / 100.0;//danger ? 0.8 : 0.0;
    // Slow down when far away from the target angle
    double drive_scale = std::max(scaleDownMovement, 1.0 - abs(response.turn_amount) * scaleDownMovement) * 1.0;

    response.drive_amount = goToOtherTarget ? -drive_scale : drive_scale;

    // Draw debugging information
    cv::circle(drawingImage, targetPos, 10, cv::Scalar(0, 255, 0), 4);

    response.drive_amount *= -1;
    response.turn_amount *= -1;

    return response;
}

Clock runAwayClock;
bool timing = false;

/**
 * This is the main robot controller loop. It is called once per frame.
 * @param state The current state of the robot and opponent
 * @return The response to send back to unity
 */
RobotControllerMessage RobotController::loop(RobotState &state, cv::Mat &drawingImage)
{
    static Extrapolator<cv::Point2f> ourPositionExtrapolator{cv::Point2f(0,0)};
    static Extrapolator<double> ourAngleExtrapolator{0};
    static Extrapolator<cv::Point2f> opponentPositionExtrapolator{cv::Point2f(0, 0)};
    static bool shouldRunClockwiseLast = false;

    // our pos + angle
    cv::Point2f ourPosition = vision.GetRobotPosition();
    ourPositionExtrapolator.SetValue(ourPosition);
    cv::Point2f ourPositionEx = ourPositionExtrapolator.Extrapolate(0.1);
    ourAngleExtrapolator.SetValue(vision.GetRobotAngle());
    double ourAngleEx = ourAngleExtrapolator.Extrapolate(0.3);
    double velocity = norm(ourPositionExtrapolator.GetVelocity());

    // opponent pos + angle
    cv::Point2f opponentPos = vision.GetOpponentPosition();
    opponentPositionExtrapolator.SetValue(opponentPos);
    cv::Point2f opponentPosEx = opponentPositionExtrapolator.Extrapolate(OPPONENT_POSITION_EXTRAPOLATE_MS / 1000.0 * norm(opponentPos - ourPosition) / ORBIT_RADIUS);
    double stickAngleRad = vision.GetOpponentAngle();

    // default just to drive to the center
    cv::Point2f targetPoint = opponentPosEx;

    cv::Point2f usToOpponent = opponentPosEx - ourPosition;
    double angleToOpponent = atan2(usToOpponent.y, usToOpponent.x);
    double angleOpponentToUs = angle_wrap(angleToOpponent + M_PI);

    // draw blue circle around opponent
    cv::circle(drawingImage, opponentPos, ORBIT_RADIUS, cv::Scalar(0, 0, 255), 1);

    // draw orange circle around opponent to show evasion radius
    cv::circle(drawingImage, opponentPosEx, ORBIT_RADIUS, cv::Scalar(255, 165, 0), 4);

    bool orbitRight = abs(angle_wrap(stickAngleRad - 0)) < (30 * TO_RAD) && stickAngleRad != 0;
    bool orbitLeft = abs(angle_wrap(stickAngleRad - M_PI)) < (30 * TO_RAD);

    if (orbitRight || orbitLeft)
    {
        const double EVASION_DELTA_THETA = ORBIT_DTHETA_DEG * TO_RAD;
        // set target point to be on a circle around the opponent
        double angleToOpponent = atan2(opponentPosEx.y - ourPosition.y, opponentPosEx.x - ourPosition.x);
        double evasionAngle = angle_wrap(angleToOpponent + (orbitRight ? 1 : -1) * EVASION_DELTA_THETA);
        targetPoint = opponentPosEx - cv::Point2f(cos(evasionAngle), sin(evasionAngle)) * ORBIT_RADIUS;
    }

    // Draw the point
    cv::circle(drawingImage, targetPoint, 10, cv::Scalar(0, 255, 0), 4);

    bool allowReverse = false;

    // Drive to the opponent position
    RobotControllerMessage response = driveToPosition(ourPosition, vision.GetRobotAngle(), targetPoint, state, drawingImage, allowReverse);

    return response;
}
