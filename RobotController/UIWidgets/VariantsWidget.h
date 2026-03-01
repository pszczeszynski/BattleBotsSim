#pragma once
#include "../RobotOdometry.h"

class VariantsWidget {
 public:
  VariantsWidget();

  void Draw();

 private:
  void _DrawStartStopButton(const char* label, bool& enabledFlag,
                            OdometryAlg algorithm);
};