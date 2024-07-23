package frc.robot.subsystems.winch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.WinchConstants;
import frc.robot.util.ClimberSim;

public class WinchIOSim implements WinchIO {

  private double LOOP_PERIOD_SEC = 0.02;

  private ClimberSim winchSim;
  private boolean lastEnabled = false;
  private double appliedVolts = 0.0;

  public WinchIOSim() {

    winchSim =
        new ClimberSim(
            DCMotor.getNEO(1),
            WinchConstants.WINCH_GEAR_REDUCTION,
            Units.lbsToKilograms(WinchConstants.WINCH_TENSION_LOAD_LB),
            Units.lbsToKilograms(WinchConstants.WINCH_ROBOT_LOAD_LB),
            Units.inchesToMeters(WinchConstants.WINCH_SPOOL_DIA_IN / 2.0),
            Units.inchesToMeters(WinchConstants.WINCH_MIN_HEIGHT_IN),
            Units.inchesToMeters(WinchConstants.WINCH_MAX_HEIGHT_IN),
            Units.inchesToMeters(WinchConstants.WINCH_START_HEIGHT_IN));

    winchSim.setParallelStates(Units.inchesToMeters(WinchConstants.WINCH_START_HEIGHT_IN), 0.0);
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    // Reset voltage when disabled
    if (DriverStation.isDisabled()) {
      setVoltage(0.0);
    }

    // Reset position on enable
    if (DriverStation.isEnabled() && !lastEnabled) {
      winchSim.setParallelStates(Units.inchesToMeters(WinchConstants.WINCH_START_HEIGHT_IN), 0.0);
    }
    lastEnabled = DriverStation.isEnabled();

    // Update sim state
    winchSim.update(LOOP_PERIOD_SEC);

    // Log sim data
    inputs.positionInch = Units.metersToInches(winchSim.getPositionMeters());
    inputs.velocityInchPerSec = Units.metersToInches(winchSim.getVelocityMetersPerSecond());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = winchSim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    winchSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setClimbMode(boolean climbMode) {
    if (climbMode) {
      winchSim.climb();
    } else {
      winchSim.extend();
    }
  }
}
