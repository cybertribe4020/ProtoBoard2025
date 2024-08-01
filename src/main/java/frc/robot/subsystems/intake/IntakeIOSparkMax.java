package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO =
      (24.0 / 16.0); // this is the gear reduction (driven/driving)

  // lower is also the back roller
  private final CANSparkMax lower = new CANSparkMax(12, MotorType.kBrushless);
  // upper is alos the front roller
  private final CANSparkMax upper = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder encLower = lower.getEncoder();
  private final RelativeEncoder encUpper = upper.getEncoder();
  private final SparkPIDController pidLower = lower.getPIDController();
  private final SparkPIDController pidUpper = upper.getPIDController();

  public IntakeIOSparkMax() {
    lower.restoreFactoryDefaults();
    upper.restoreFactoryDefaults();
    lower.setCANTimeout(250);
    upper.setCANTimeout(250);
    lower.setInverted(true);
    upper.setInverted(true);
    lower.enableVoltageCompensation(12.0);
    lower.setSmartCurrentLimit(40);
    upper.enableVoltageCompensation(12.0);
    upper.setSmartCurrentLimit(40);

    // for velocity control with low inertia, reduce the encoder sensor filtering
    // default filter values add so much effective dead time that P control is almost impossible
    encLower.setMeasurementPeriod(16);
    encUpper.setMeasurementPeriod(16);
    encLower.setAverageDepth(2);
    encUpper.setAverageDepth(2);

    lower.setCANTimeout(0);
    upper.setCANTimeout(0);
    lower.burnFlash();
    upper.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Note that the 0 list position is the lower/back roller and 1 is upper/front
    inputs.positionRot =
        new double[] {encLower.getPosition() / GEAR_RATIO, encUpper.getPosition() / GEAR_RATIO};
    inputs.velocityRPM =
        new double[] {encLower.getVelocity() / GEAR_RATIO, encUpper.getVelocity() / GEAR_RATIO};
    inputs.appliedVolts =
        new double[] {
          lower.getAppliedOutput() * lower.getBusVoltage(),
          upper.getAppliedOutput() * upper.getBusVoltage()
        };
    inputs.currentAmps = new double[] {lower.getOutputCurrent(), upper.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double voltsLower, double voltsUpper) {
    lower.setVoltage(voltsLower);
    upper.setVoltage(voltsUpper);
  }

  @Override
  public void setVelocity(
      double velocityRPMLower, double ffVoltsLower, double velocityRPMUpper, double ffVoltsUpper) {
    pidLower.setReference(
        velocityRPMLower * GEAR_RATIO, ControlType.kVelocity, 0, ffVoltsLower, ArbFFUnits.kVoltage);
    pidUpper.setReference(
        velocityRPMUpper * GEAR_RATIO, ControlType.kVelocity, 0, ffVoltsUpper, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    lower.stopMotor();
    upper.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidLower.setP(kP, 0);
    pidLower.setI(kI, 0);
    pidLower.setD(kD, 0);
    pidLower.setFF(0, 0);
    pidUpper.setP(kP, 0);
    pidUpper.setI(kI, 0);
    pidUpper.setD(kD, 0);
    pidUpper.setFF(0, 0);
  }
}
