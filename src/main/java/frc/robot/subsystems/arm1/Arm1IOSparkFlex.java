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

  private final boolean isMotorInverted = true;

  // Offset so through-bore encoder will read 0 when the arm is parallel to the floor
  private final double absoluteEncoderOffsetRad =
      Units.degreesToRadians(ArmConstants.ARM_ENCODER_OFFSET_DEG);

  // Power on the robot with the arm fully down against the physical stops
  // This position is the ARM_MIN_ANGLE_DEG
  // Initialize the internal motor encoder to the shaft revolutions corresponding to this angle
  private final double intEncoderPositionStopsRev =
      ArmConstants.ARM_MIN_ANGLE_DEG / 360.0 * ArmConstants.ARM_GEAR_REDUCTION;

  public Arm1IOSparkFlex() {
    leader.restoreFactoryDefaults();
    leader.setCANTimeout(250);
    leader.setInverted(isMotorInverted);
    setBrakeMode(true);
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    motorInternalEncoder.setPosition(intEncoderPositionStopsRev);

    // Vortex has a much higher resolution encoder than the NEO
    // More filtering is possible on the Vortex encoder while keeping the signal delay manageable
    // period can be 8-64 (default 32) for NEO, 1-100 (default) for Vortex
    // average depth can be 1, 2, 4, 8 (default) for NEO, 1-64 (default) for Vortex
    motorInternalEncoder.setMeasurementPeriod(32); // was 10 if need to go back
    motorInternalEncoder.setAverageDepth(8); // was 2 if need to go back

    leader.setCANTimeout(0);
    leader.burnFlash();

    extAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
  }

  @Override
  public void updateInputs(Arm1IOInputs inputs) {
    inputs.absolutePositionRad =
        (extAbsoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI) - absoluteEncoderOffsetRad;
    inputs.internalPositionRad =
        Units.rotationsToRadians(motorInternalEncoder.getPosition())
            / ArmConstants.ARM_GEAR_REDUCTION;
    inputs.internalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motorInternalEncoder.getVelocity())
            / ArmConstants.ARM_GEAR_REDUCTION;
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = leader.getOutputCurrent();
    inputs.tempCelsius = leader.getMotorTemperature();
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
