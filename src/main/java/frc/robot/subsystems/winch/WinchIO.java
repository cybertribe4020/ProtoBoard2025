package frc.robot.subsystems.winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public static class WinchIOInputs {
    public double positionInch = 0.0;
    public double velocityInchPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WinchIOInputs inputs) {}

  /** Run the winch motor(s) at the specified voltages. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Enable or disable brake mode on the motor(s). */
  public default void setBrakeMode(boolean shouldBrake) {}

  /** Are the winches in climb or extend mode? */
  public default void setClimbMode(boolean climbMode) {}
}
