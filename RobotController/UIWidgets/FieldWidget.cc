#include "FieldWidget.h"
#include "../RobotController.h"
#include "../GuiUtils.h"

FieldWidget::FieldWidget() : ImageWidget("Field", RobotController::GetInstance().GetDrawingImage(), false)
{
}

FieldWidget &FieldWidget::GetInstance()
{
    static FieldWidget instance;
    return instance;
}

void FieldWidget::Draw()
{
    // update the mat
    UpdateMat(RobotController::GetInstance().GetDrawingImage());
    // call super method
    ImageWidget::Draw();
}