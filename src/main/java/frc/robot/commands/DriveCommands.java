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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double TRANSLATION_RATE_SCALE = 1.0;
  private static final double ROTATION_RATE_SCALE = 0.6;

  private DriveCommands() {}

  // Drive with two joysticks
  // X and Y translation suppliers are usually two axes of one stick
  // Omega rotation supplier is usually one axis of another stick
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband to joystick axis values
          // Need this because joysticks rarely settle exactly to the center point
          // Without deadband, the robot may move with hands off the sticks
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values to get a nonlinear response to how far you push the sticks
          // Without this, it would be extremely difficult to drive slowly
          // Also apply scaling after squaring to set the upper bound of translation and rotation
          // speed
          linearMagnitude = TRANSLATION_RATE_SCALE * (linearMagnitude * linearMagnitude);
          omega = ROTATION_RATE_SCALE * Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity by combining magnitude and direction
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          Logger.recordOutput("JoystickDrive/linearMagnitude", linearMagnitude);
          Logger.recordOutput("JoystickDrive/linearVelocity", linearVelocity);
          Logger.recordOutput("JoystickDrive/omega", omega);

          // Turn joystick inputs into robot chassis speed requests
          if (drive.driveFieldCentric) {
            // Joystick inputs are field-centric, so they need to be converted
            // into robot-centric chassis speeds
            // Current robot rotation from the gyro is required to make this conversion
            boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec(),
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));
          } else {
            // Joystick inputs are already robot-centric, so just send them
            drive.runVelocity(
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec()));
          }
        },
        drive);
  }
}
