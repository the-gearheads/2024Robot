package frc.robot.util;

import edu.wpi.first.math.controller.BangBangController;

public class BetterBangBang extends BangBangController {
  @Override
  public double calculate(double measurement) {

    calculate(measurement, getSetpoint()); // an abuse of implementation details to set m_setpoint
    var setpoint = getSetpoint();

    if(atSetpoint()) {
      return 0;
    }

    return measurement < setpoint ? 1 : -1;
  }

}
