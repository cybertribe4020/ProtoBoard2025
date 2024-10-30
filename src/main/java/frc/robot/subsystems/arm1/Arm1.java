package frc.robot.subsystems.arm1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private Double angleGoalRad = Units.degreesToRadians(ArmConstants.ARM_TARGET_DEG);
  private String armStatus = "#000000";
  public Boolean armClosedLoop = false;

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
              Units.radiansToDegrees(inputs.internalPositionRad),
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
    pid.setTolerance(Units.degreesToRadians(ArmConstants.ARM_ANGLE_TOLERANCE_DEG));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (armClosedLoop) {
      pidOutput = pid.calculate(inputs.internalPositionRad, angleGoalRad);
      feedforwardOutput = ffModel.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);

      Logger.recordOutput("Arm/atGoal", pid.atGoal());
      Logger.recordOutput("Arm/angleGoalRad", angleGoalRad);
      Logger.recordOutput("Arm/angleSPRad", pid.getSetpoint().position);
      Logger.recordOutput("Arm/angleSPRadPerSec", pid.getSetpoint().velocity);
      Logger.recordOutput("Arm/feedbackOP", pidOutput);
      Logger.recordOutput("Arm/feedforwardOP", feedforwardOutput);

    } else {
      pid.reset(inputs.internalPositionRad);
    }

    if (inputs.internalPositionRad < -0.523) {
      armStatus = "#4CAF50"; // green
    } else if (inputs.internalPositionRad < 0.872) {
      armStatus = "#F44336"; // red
    } else {
      armStatus = "#9528CC"; // purple
    }

    Logger.recordOutput("Arm/angleInternalRad", inputs.internalPositionRad);
    Logger.recordOutput("Arm/angleInternalDeg", Units.radiansToDegrees(inputs.internalPositionRad));
    Logger.recordOutput("Arm/motorVolts", inputs.appliedVolts);
    Logger.recordOutput("Arm/armIsUp", armIsUp());
    Logger.recordOutput("Arm/status", armStatus);

    // Update the Mechanism Arm angle based on the simulated arm angle
    arm.setAngle(Units.radiansToDegrees(inputs.internalPositionRad));
  }

  /** Set the goal for the arm angle. Put controller in auto if not already. */
  public void setGoalDeg(double setpointDeg) {
    armClosedLoop = true;
    angleGoalRad =
        Units.degreesToRadians(
            MathUtil.clamp(
                setpointDeg, ArmConstants.ARM_MIN_ANGLE_DEG, ArmConstants.ARM_MAX_ANGLE_DEG));
  }

  /** Stops the Arm. */
  public void stop() {
    io.stop();
    armClosedLoop = false;
  }

  /** Returns the current arm angle in degrees. */
  @AutoLogOutput(key = "Arm/angleInternalDeg")
  public double getPositionDeg() {
    return Units.radiansToDegrees(inputs.internalPositionRad);
  }

  // Check if arm is within a tolerance of the down/load position.
  // Typically used as a permissive in commands.
  public boolean armAtTarget() {
    return Units.radiansToDegrees(inputs.internalPositionRad)
        <= ArmConstants.ARM_TARGET_DEG + ArmConstants.ARM_IS_DOWN_TOLERANCE_DEG;
  }

  public boolean armAtZero() {
    return Units.radiansToDegrees(inputs.internalPositionRad)
        <= ArmConstants.ARM_IS_DOWN_TOLERANCE_DEG;
  }

  // Define "arm up" as an angle greater than 0 radians - currently no
  // shooting position would be lower than that
  public boolean armIsUp() {
    return inputs.internalPositionRad >= 0.0;
  }

  // If the arm is less than 75 degrees it is not in an amp position
  public boolean armIsNotAmped() {
    return inputs.internalPositionRad < Units.degreesToRadians(75.0);
  }

  // Has the arm reached the closed-loop goal?
  // Tolerance is set separately with ARM_ANGLE_TOLERANCE_DEG.
  public boolean atGoal() {
    return pid.atGoal();
  }

  // Move the arm to the stow/loading position.
  public Command armToTargetCommand() {
    return new RunCommand(() -> setGoalDeg(ArmConstants.ARM_TARGET_DEG)).until(() -> armAtTarget());
  }

  public Command armToZeroCommand() {
    return new RunCommand(() -> setGoalDeg(0.0)).until(() -> armAtZero());
  }
}
