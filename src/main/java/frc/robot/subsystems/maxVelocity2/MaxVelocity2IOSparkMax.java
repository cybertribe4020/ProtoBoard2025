package frc.robot.subsystems.maxVelocity2;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class MaxVelocity2IOSparkMax implements MaxVelocity2IO {
  private static final double GEAR_RATIO = 4.0; // this is the gear reduction (driven/driving)

  private final CANSparkMax leader = new CANSparkMax(2, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  public MaxVelocity2IOSparkMax() {
    leader.restoreFactoryDefaults();
    leader.setCANTimeout(250);
    leader.setInverted(false);
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    // for velocity control with low inertia, reduce the encoder sensor filtering
    // default filter values add so much effective dead time that P control is almost impossible
    // period can be 8-64 (default 32) for NEO, 1-100 (default) for Vortex
    // average depth can be 1, 2, 4, 8 (default) for NEO, 1-64 (default) for Vortex
    encoder.setMeasurementPeriod(16);
    encoder.setAverageDepth(2);

    leader.setCANTimeout(0);
    leader.burnFlash();
  }

  @Override
  public void updateInputs(MaxVelocity2IOInputs inputs) {
    inputs.positionRot = encoder.getPosition() / GEAR_RATIO;
    inputs.velocityRPM = encoder.getVelocity() / GEAR_RATIO;
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = leader.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRPM, double ffVolts) {
    pid.setReference(
        velocityRPM * GEAR_RATIO, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
