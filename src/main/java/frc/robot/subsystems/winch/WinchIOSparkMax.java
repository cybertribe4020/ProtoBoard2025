package frc.robot.subsystems.winch;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.WinchConstants;

public class WinchIOSparkMax implements WinchIO {

  private CANSparkMax spark;
  private RelativeEncoder internalEncoder;

  // Power on the robot with the climber ropes retracted so the hook mounts are parallel to the
  // floor
  // This position is the WINCH_START_HEIGHT_IN
  // Initialize the internal motor encoders to the shaft revolutions corresponding to this height
  private final double encoderPositionStartRev =
      WinchConstants.WINCH_GEAR_REDUCTION
          * WinchConstants.WINCH_START_HEIGHT_IN
          / (WinchConstants.WINCH_SPOOL_DIA_IN * Math.PI);

  public WinchIOSparkMax(int canID, boolean isInverted) {
    spark = new CANSparkMax(canID, MotorType.kBrushless);
    internalEncoder = spark.getEncoder();

    spark.restoreFactoryDefaults();
    spark.setCANTimeout(250);
    spark.setInverted(isInverted);
    setBrakeMode(true);
    spark.enableVoltageCompensation(12.0);
    spark.setSmartCurrentLimit(WinchConstants.WINCH_CURRENT_LIMIT);

    internalEncoder.setPosition(encoderPositionStartRev);

    // Start with default filtering on the NEO internal encoders
    // Reduce filtering if needed for good control
    // internalEncoder.setMeasurementPeriod(16); // default is 32
    // internalEncoder.setAverageDepth(2); // default is 8
    // internalEncoder.setMeasurementPeriod(16); // default is 32
    // internalEncoder.setAverageDepth(2); // default is 8

    spark.setCANTimeout(0);
    spark.burnFlash();
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    // native NEO encoder position in revolutions
    inputs.positionInch =
        internalEncoder.getPosition()
            / WinchConstants.WINCH_GEAR_REDUCTION
            * WinchConstants.WINCH_SPOOL_DIA_IN
            * Math.PI;
    // native NEO encoder velocity in revolutions per minute
    inputs.velocityInchPerSec =
        internalEncoder.getVelocity()
            / WinchConstants.WINCH_GEAR_REDUCTION
            / 60.0
            * WinchConstants.WINCH_SPOOL_DIA_IN
            * Math.PI;
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.tempCelsius = spark.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    spark.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
