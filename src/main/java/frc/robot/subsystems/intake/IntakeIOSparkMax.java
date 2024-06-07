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

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = (24.0 / 16.0);

  private final CANSparkMax lower = new CANSparkMax(12, MotorType.kBrushless);
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

    lower.setInverted(false);
    upper.setInverted(true);

    lower.enableVoltageCompensation(12.0);
    lower.setSmartCurrentLimit(30);
    upper.enableVoltageCompensation(12.0);
    upper.setSmartCurrentLimit(30);

    lower.burnFlash();
    upper.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad =
        new double[] {
          Units.rotationsToRadians(encLower.getPosition() / GEAR_RATIO),
          Units.rotationsToRadians(encUpper.getPosition() / GEAR_RATIO)
        };
    inputs.velocityRadPerSec =
        new double[] {
          Units.rotationsPerMinuteToRadiansPerSecond(encLower.getVelocity() / GEAR_RATIO),
          Units.rotationsPerMinuteToRadiansPerSecond(encUpper.getVelocity() / GEAR_RATIO)
        };
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
      double velocityRadPerSecLower, double velocityRadPerSecUpper, double ffVolts) {
    pidLower.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSecLower) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
    pidUpper.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSecUpper) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    lower.stopMotor();
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
