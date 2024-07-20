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

  // motor on the lower shooter axle is CAN 20 and set as the leader here
  // motor on the upper shooter axle is CAN 21 and set as the follower
  private final CANSparkMax leader = new CANSparkMax(20, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(21, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  public ShooterIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();
    leader.setCANTimeout(250);
    follower.setCANTimeout(250);
    follower.follow(leader, false);
    leader.setInverted(false);
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    // for velocity control with low inertia, reduce the encoder sensor filtering
    // default filter values add so much effective dead time that P control is almost impossible
    encoder.setMeasurementPeriod(16);
    encoder.setAverageDepth(2);

    leader.setCANTimeout(0);
    follower.setCANTimeout(0);
    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
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
