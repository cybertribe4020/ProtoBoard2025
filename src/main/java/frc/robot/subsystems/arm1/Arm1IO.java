package frc.robot.subsystems.arm1;

import org.littletonrobotics.junction.AutoLog;

public interface Arm1IO {
  @AutoLog
  public static class Arm1IOInputs {
    public double arm1AbsolutePositionRad = 0.0;
    public double arm1InternalPositionRad = 0.0;
    public double arm1InternalVelocityRadPerSec = 0.0;
    public double arm1AppliedVolts = 0.0;
    public double[] arm1CurrentAmps = new double[] {};
    public double[] arm1TempCelsius = new double[] {};
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
