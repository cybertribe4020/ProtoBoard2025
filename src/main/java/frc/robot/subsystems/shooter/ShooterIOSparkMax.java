package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  private static final double GEAR_RATIO = 1.0; // this is the gear reduction (driven/driving)

  // motor on the lower shooter axle is CAN 20 and set as the lower here
  // motor on the upper shooter axle is CAN 21 and set as the upper
  private final CANSparkMax lower = new CANSparkMax(20, MotorType.kBrushless);
  private final CANSparkMax upper = new CANSparkMax(21, MotorType.kBrushless);
  private final RelativeEncoder encoderLower = lower.getEncoder();
  private final RelativeEncoder encoderUpper = upper.getEncoder();
  private final SparkPIDController pidLower = lower.getPIDController();
  private final SparkPIDController pidUpper = lower.getPIDController();

  public ShooterIOSparkMax() {
    lower.restoreFactoryDefaults();
    upper.restoreFactoryDefaults();
    lower.setCANTimeout(250);
    upper.setCANTimeout(250);
    lower.setInverted(false);
    upper.setInverted(false);
    lower.enableVoltageCompensation(12.0);
    upper.enableVoltageCompensation(12.0);
    lower.setSmartCurrentLimit(30);
    upper.setSmartCurrentLimit(30);

    // for velocity control with low inertia, reduce the encoder sensor filtering
    // default filter values add so much effective dead time that P control is almost impossible
    encoderLower.setMeasurementPeriod(16);
    encoderLower.setAverageDepth(2);
    encoderUpper.setMeasurementPeriod(16);
    encoderUpper.setAverageDepth(2);

    lower.setCANTimeout(0);
    upper.setCANTimeout(0);
    lower.burnFlash();
    upper.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionLowerRad = Units.rotationsToRadians(encoderLower.getPosition() / GEAR_RATIO);
    inputs.velocityLowerRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderLower.getVelocity() / GEAR_RATIO);
    inputs.appliedVoltsLower = lower.getAppliedOutput() * lower.getBusVoltage();
    inputs.currentAmpsLower = lower.getOutputCurrent();
    inputs.positionUpperRad = Units.rotationsToRadians(encoderUpper.getPosition() / GEAR_RATIO);
    inputs.velocityUpperRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderUpper.getVelocity() / GEAR_RATIO);
    inputs.appliedVoltsUpper = upper.getAppliedOutput() * upper.getBusVoltage();
    inputs.currentAmpsLower = upper.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    lower.setVoltage(volts);
    upper.setVoltage(volts);
  }

  @Override
  public void setVoltageEach(double voltsLower, double voltsUpper) {
    lower.setVoltage(voltsLower);
    upper.setVoltage(voltsUpper);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pidLower.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
    pidUpper.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
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
