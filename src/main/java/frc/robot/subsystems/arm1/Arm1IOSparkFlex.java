package frc.robot.subsystems.arm1;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ArmConstants;

public class Arm1IOSparkFlex implements Arm1IO {

  private final CANSparkFlex leader = new CANSparkFlex(22, MotorType.kBrushless);
  private final RelativeEncoder motorInternalEncoder = leader.getEncoder();
  private final DutyCycleEncoder extAbsoluteEncoder = new DutyCycleEncoder(8);

  private final boolean isMotorInverted = false;
  private final double absoluteEncoderOffsetRad =
      Units.degreesToRadians(ArmConstants.ARM_ENCODER_OFFSET_DEG);
  // Power on the robot with the arm fully down against the physical stops
  // This position is the ARM_MIN_ANGLE_DEG
  // Initialize the internal motor encoder to the shaft revolutions corresponding to this angle
  private final double intEncoderPositionStowRev =
      ArmConstants.ARM_MIN_ANGLE_DEG / 360.0 * ArmConstants.ARM_GEAR_REDUCTION;

  public Arm1IOSparkFlex() {
    leader.restoreFactoryDefaults();

    leader.setCANTimeout(250);

    leader.setInverted(isMotorInverted);
    setBrakeMode(true);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    motorInternalEncoder.setPosition(intEncoderPositionStowRev);
    motorInternalEncoder.setMeasurementPeriod(10);
    motorInternalEncoder.setAverageDepth(2);

    leader.setCANTimeout(0);

    leader.burnFlash();

    extAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
  }

  @Override
  public void updateInputs(Arm1IOInputs inputs) {
    inputs.arm1AbsolutePositionRad =
        (extAbsoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI) - absoluteEncoderOffsetRad;
    inputs.arm1InternalPositionRad =
        Units.rotationsToRadians(motorInternalEncoder.getPosition())
            / ArmConstants.ARM_GEAR_REDUCTION;
    inputs.arm1InternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motorInternalEncoder.getVelocity())
            / ArmConstants.ARM_GEAR_REDUCTION;
    inputs.arm1AppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.arm1CurrentAmps = new double[] {leader.getOutputCurrent()};
    inputs.arm1TempCelsius = new double[] {leader.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    leader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
