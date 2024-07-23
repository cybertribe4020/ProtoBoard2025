package frc.robot.subsystems.winch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WinchConstants;
import org.littletonrobotics.junction.Logger;

public class Winch extends SubsystemBase {
  private final WinchIO io;
  private final WinchIOInputsAutoLogged inputs = new WinchIOInputsAutoLogged();
  private final ProfiledPIDController pid;
  private ElevatorFeedforward ffModel;
  private Double pidOutput;
  private Double feedforwardOutput;
  private Double heightGoalInch = WinchConstants.WINCH_START_HEIGHT_IN;
  private String side;
  private Double lastGoal = heightGoalInch;
  public Boolean winchClosedLoop = false;

  // Create a Mechanism2d display of an Winch with a fixed WinchTower and moving Winch.
  // private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  // private final MechanismRoot2d winchPivot = mech2d.getRoot("WinchPivot", 30, 30);
  // private final MechanismLigament2d winchTower =
  //     winchPivot.append(new MechanismLigament2d("WinchTower", 30, -90));
  // private final MechanismLigament2d winch =
  //     winchPivot.append(
  //         new MechanismLigament2d(
  //             "Winch",
  //             30,
  //             Units.radiansToDegrees(inputs.internalPositionRad),
  //             6,
  //             new Color8Bit(Color.kYellow)));

  /** Creates a new Winch. */
  public Winch(WinchIO io, String side) {
    this.io = io;
    this.side = side;

    // Put Mechanism 2d to SmartDashboard
    // SmartDashboard.putData("Winch " + side + " Sim", mech2d);
    // winchTower.setColor(new Color8Bit(Color.kBlue));

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.87, 0.0);
        pid =
            new ProfiledPIDController(
                20.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    WinchConstants.WINCH_VELOCITY_EXTEND_IPS,
                    WinchConstants.WINCH_ACCEL_EXTEND_IPS2));
        break;
      case REPLAY:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.87, 0.0);
        pid =
            new ProfiledPIDController(
                20.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    WinchConstants.WINCH_VELOCITY_EXTEND_IPS,
                    WinchConstants.WINCH_ACCEL_EXTEND_IPS2));
        break;
      case SIM:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.87, 0.0);
        pid =
            new ProfiledPIDController(
                20.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    WinchConstants.WINCH_VELOCITY_EXTEND_IPS,
                    WinchConstants.WINCH_ACCEL_EXTEND_IPS2));
        break;
      default:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
        pid = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        break;
    }
    pid.setTolerance(WinchConstants.WINCH_HEIGHT_TOLERANCE_IN);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Winch" + side, inputs);

    if (winchClosedLoop) {
      pidOutput = pid.calculate(inputs.positionInch, heightGoalInch);
      feedforwardOutput = ffModel.calculate(pid.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);

      Logger.recordOutput("Winch" + side + "/atGoal", pid.atGoal());
      Logger.recordOutput("Winch" + side + "/heightGoalInch", heightGoalInch);
      Logger.recordOutput("Winch" + side + "/heightSPInch", pid.getSetpoint().position);
      Logger.recordOutput("Winch" + side + "/heightSPInchPerSec", pid.getSetpoint().velocity);
      Logger.recordOutput("Winch" + side + "/feedbackOP", pidOutput);
      Logger.recordOutput("Winch" + side + "/feedforwardOP", feedforwardOutput);

    } else {
      pid.reset(inputs.positionInch);
    }

    Logger.recordOutput("Winch" + side + "/heightInch", inputs.positionInch);
    Logger.recordOutput(
        "Winch" + side + "/heightSpoolRot",
        inputs.positionInch / WinchConstants.WINCH_SPOOL_DIA_IN / Math.PI);
    Logger.recordOutput("Winch" + side + "/motorVolts", inputs.appliedVolts);
    Logger.recordOutput("Winch" + side + "/closedLoop", winchClosedLoop);

    // Update the Mechanism Winch angle based on the simulated winch angle
    // winch.setAngle(Units.radiansToDegrees(inputs.internalPositionRad));
  }

  /** Set the goal for the winch height. Put controller in auto if not already. */
  public void setGoalInch(double setpointInch) {
    winchClosedLoop = true;
    heightGoalInch =
        MathUtil.clamp(
            setpointInch, WinchConstants.WINCH_MIN_HEIGHT_IN, WinchConstants.WINCH_MAX_HEIGHT_IN);
    if (heightGoalInch > lastGoal) {
      io.setClimbMode(true);
    } else {
      io.setClimbMode(false);
    }
    lastGoal = heightGoalInch;
  }

  // Adjust profile constraints
  public void setProfile(double velocity, double accel) {
    pid.setConstraints(new TrapezoidProfile.Constraints(velocity, accel));
  }

  // Winch system is quite different between extending with a gas spring
  // and retracting with a robot.  Change tuning for each mode.
  public void winchTuning(boolean tuneToClimb) {
    if (tuneToClimb) {
      pid.setConstraints(
          new TrapezoidProfile.Constraints(
              WinchConstants.WINCH_VELOCITY_CLIMB_IPS, WinchConstants.WINCH_ACCEL_CLIMB_IPS2));
      ffModel = new ElevatorFeedforward(0.0, 0.68, 1.1, 0.0);
    } else {
      pid.setConstraints(
          new TrapezoidProfile.Constraints(
              WinchConstants.WINCH_VELOCITY_EXTEND_IPS, WinchConstants.WINCH_ACCEL_EXTEND_IPS2));
      ffModel = new ElevatorFeedforward(0.0, 0.0, 0.87, 0.0);
    }
  }

  /** Stops the Winch. */
  public void stop() {
    io.stop();
    winchClosedLoop = false;
  }

  // Has the winch reached the closed-loop goal?
  // Tolerance is set separately with ARM_ANGLE_TOLERANCE_DEG.
  public boolean atGoal() {
    return pid.atGoal();
  }
}
