package frc.robot.util;

import edu.wpi.first.math.controller.BangBangController;

/* BangBang except it actually will also go into the reverse direction too */
public class BetterBangBang extends BangBangController {
  @Override
  public double calculate(double measurement) {

    calculate(measurement, getSetpoint()); // an abuse of implementation details to set m_setpoint
    var setpoint = getSetpoint();

    if(atSetpoint()) {
      return 0;
    }

    return measurement < setpoint ? 1 : -1; //TODO: move this into its own method
  }

}
