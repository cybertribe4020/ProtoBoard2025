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

package frc.robot.subsystems.arm1;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm1 extends SubsystemBase {
  private final Arm1IO io;
  private final Arm1IOInputsAutoLogged inputs = new Arm1IOInputsAutoLogged();
  private final ArmFeedforward ffModel;
  private final ProfiledPIDController pid;
  private Double pidOutput;
  private Double feedforwardOutput;
  private Double angleGoalRad = Units.degreesToRadians(ArmConstants.ARM_INIT_ANGLE_DEG);
  private Boolean armClosedLoop = true;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(inputs.arm1InternalPositionRad),
              6,
              new Color8Bit(Color.kYellow)));

  /** Creates a new Arm. */
  public Arm1(Arm1IO io) {
    this.io = io;

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", mech2d);
    armTower.setColor(new Color8Bit(Color.kBlue));

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new ArmFeedforward(0.0, 0.16, 6.6, 0.01);
        pid = new ProfiledPIDController(60.0, 0.0, 1.0, new TrapezoidProfile.Constraints(1.8, 4.0));
        break;
      case REPLAY:
        ffModel = new ArmFeedforward(0.0, 0.21, 6.93, 0.01);
        pid = new ProfiledPIDController(80.0, 0.0, 3.0, new TrapezoidProfile.Constraints(1.7, 8.0));
        break;
      case SIM:
        ffModel = new ArmFeedforward(0.0, 0.21, 6.93, 0.01);
        pid = new ProfiledPIDController(80.0, 0.0, 3.0, new TrapezoidProfile.Constraints(1.7, 8.0));
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
        pid = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (armClosedLoop) {
      pidOutput = pid.calculate(inputs.arm1InternalPositionRad, angleGoalRad);
      feedforwardOutput = ffModel.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);
    } else {
      pid.reset(inputs.arm1InternalPositionRad);
    }

    Logger.recordOutput("Arm1/PID Goal", angleGoalRad);
    Logger.recordOutput("Arm1/PID SP", pid.getSetpoint().position);
    Logger.recordOutput("Arm1/PID PV", inputs.arm1InternalPositionRad);
    Logger.recordOutput("Arm1/PID OP", pidOutput);
    Logger.recordOutput("Arm1/FF OP", feedforwardOutput);
    Logger.recordOutput("Arm1/PID Volts", inputs.arm1AppliedVolts);
    Logger.recordOutput("Arm1/PID SP V", pid.getSetpoint().velocity);

    // Update the Mechanism Arm angle based on the simulated arm angle
    arm.setAngle(Units.radiansToDegrees(inputs.arm1InternalPositionRad));
  }

  /** Run closed loop control to move the arm to the desired position */
  public void setGoalDeg(double setpointDeg) {
    armClosedLoop = true;
    angleGoalRad = Units.degreesToRadians(MathUtil.clamp(setpointDeg, ArmConstants.ARM_MIN_ANGLE_DEG, ArmConstants.ARM_MAX_ANGLE_DEG));
  }

  /** Stops the Arm. */
  public void stop() {
    io.stop();
    armClosedLoop = false;
  }

  /** Returns the current position in degrees. */
  @AutoLogOutput
  public double getPositionDeg() {
    return Units.radiansToDegrees(inputs.arm1InternalPositionRad);
  }
}
