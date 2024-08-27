#include "FieldWidget.h"
#include "../RobotController.h"

FieldWidget* FieldWidget::_instance = nullptr;

FieldWidget::FieldWidget() : ImageWidget("Field", false)
{
    _instance = this;
}

FieldWidget *FieldWidget::GetInstance()
{
    return _instance;
}
