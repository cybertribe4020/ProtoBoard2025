package frc.robot.subsystems.arm1;

import org.littletonrobotics.junction.AutoLog;

public interface Arm1IO {
  @AutoLog
  public static class Arm1IOInputs {
    public double internalPositionRad = 0.0;
    public double internalVelocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Arm1IOInputs inputs) {}

  /** Run the arm1 motor(s) at the specified voltages. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean arm1Brake) {}
}
