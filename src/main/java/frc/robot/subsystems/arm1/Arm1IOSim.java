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
  private double appliedVolts = 0.0;

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
    inputs.internalPositionRad = arm1Sim.getAngleRads();
    inputs.internalVelocityRadPerSec = arm1Sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = arm1Sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    arm1Sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
