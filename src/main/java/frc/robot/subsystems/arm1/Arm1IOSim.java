package frc.robot.subsystems.arm1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;

public class Arm1IOSim implements Arm1IO {

  private double LOOP_PERIOD_SEC = 0.02;

  private SingleJointedArmSim arm1Sim;
  private boolean lastEnabled = false;
  private double arm1AppliedVolts = 0.0;

  public Arm1IOSim() {

    arm1Sim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(1),
            ArmConstants.ARM_GEAR_REDUCTION,
            SingleJointedArmSim.estimateMOI(
                Units.inchesToMeters(ArmConstants.ARM_LENGTH_IN),
                Units.lbsToKilograms(ArmConstants.ARM_MASS_LBF)),
            Units.inchesToMeters(ArmConstants.ARM_LENGTH_IN),
            Units.degreesToRadians(ArmConstants.ARM_MIN_ANGLE_DEG),
            Units.degreesToRadians(ArmConstants.ARM_MAX_ANGLE_DEG),
            true,
            Units.degreesToRadians(ArmConstants.ARM_MIN_ANGLE_DEG));

    arm1Sim.setState(VecBuilder.fill(Units.degreesToRadians(ArmConstants.ARM_MIN_ANGLE_DEG), 0.0));
  }

  @Override
  public void updateInputs(Arm1IOInputs inputs) {
    // Reset voltage when disabled
    if (DriverStation.isDisabled()) {
      setVoltage(0.0);
    }

    // Reset position on enable
    if (DriverStation.isEnabled() && !lastEnabled) {
      arm1Sim.setState(
          VecBuilder.fill(Units.degreesToRadians(ArmConstants.ARM_MIN_ANGLE_DEG), 0.0));
    }
    lastEnabled = DriverStation.isEnabled();

    // Update sim state
    arm1Sim.update(LOOP_PERIOD_SEC);

    // Log sim data
    inputs.arm1AbsolutePositionRad = arm1Sim.getAngleRads();
    inputs.arm1InternalPositionRad = arm1Sim.getAngleRads();
    inputs.arm1InternalVelocityRadPerSec = arm1Sim.getVelocityRadPerSec();
    inputs.arm1AppliedVolts = arm1AppliedVolts;
    inputs.arm1CurrentAmps = new double[] {arm1Sim.getCurrentDrawAmps()};
    inputs.arm1TempCelsius = new double[] {};
  }

  @Override
  public void setVoltage(double volts) {
    arm1AppliedVolts = MathUtil.clamp(volts, -12, 12);
    arm1Sim.setInputVoltage(arm1AppliedVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
