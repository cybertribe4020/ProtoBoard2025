// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for Talon FX drive motor controller, SparkMAX turn motor controller, and
 * CANcoder
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
@SuppressWarnings({"deprecation", "removal"})
public class ModuleIO4020P5 implements ModuleIO {

  private final WPI_TalonFX driveTalon;
  private final CANSparkMax turnSparkMax;
  private final WPI_CANCoder cancoder;

  private final RelativeEncoder turnRelativeEncoder;

  // Gear ratio for SDS MK4i L2 with 16T drive motor pinion
  private final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIO4020P5(int index) {
    switch (index) {
      case 0: // FL
        driveTalon = new WPI_TalonFX(4);
        turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        cancoder = new WPI_CANCoder(3);
        absoluteEncoderOffset = new Rotation2d(2.638); // ToDo: MUST BE CALIBRATED
        break;
      case 1: // FR
        driveTalon = new WPI_TalonFX(2);
        turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        cancoder = new WPI_CANCoder(1);
        absoluteEncoderOffset = new Rotation2d(0.723); // ToDo: MUST BE CALIBRATED
        break;
      case 2: // BL
        driveTalon = new WPI_TalonFX(6);
        turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        cancoder = new WPI_CANCoder(5);
        absoluteEncoderOffset = new Rotation2d(-2.790); // ToDo: MUST BE CALIBRATED
        break;
      case 3: // BR
        driveTalon = new WPI_TalonFX(8);
        turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        cancoder = new WPI_CANCoder(7);
        absoluteEncoderOffset = new Rotation2d(-2.065); // ToDo: MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.supplyCurrLimit.currentLimit = 40.0;
    driveConfig.supplyCurrLimit.enable = true;
    driveTalon.configAllSettings(driveConfig);
    setDriveBrakeMode(true);
    // driveTalon.configVoltageCompSaturation(12.0);
    // driveTalon.enableVoltageCompensation(true);

    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.enableVoltageCompensation(12.0);

    CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
    cancoder.configAllSettings(cancoderConfig);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveTalon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0,
        10); // faster position rate for odometry - Phoenix5 can't separate velocity from position

    turnSparkMax.setCANTimeout(0);
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveTalon.getSelectedSensorPosition(0) / 2048.0)
            / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveTalon.getSelectedSensorVelocity(0) * 10.0 / 2048.0)
            / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveTalon.getMotorOutputVoltage();
    inputs.driveCurrentAmps = new double[] {driveTalon.getSupplyCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(cancoder.getAbsolutePosition() / 360.0)
            .minus(absoluteEncoderOffset);

    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveTalon.setInverted(InvertType.InvertMotorOutput); // 4020 code has drive motors inverted
    driveTalon.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
