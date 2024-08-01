package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
public class ModuleIO4020 implements ModuleIO {

  private final TalonFX driveTalon;
  private final CANSparkMax turnSparkMax;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final RelativeEncoder turnRelativeEncoder;

  // Gear ratio for SDS MK4i L2 with 16T drive motor pinion
  private final double DRIVE_GEAR_RATIO =
      (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0); // this is the gear reduction (driven/driving)
  private final double TURN_GEAR_RATIO = 150.0 / 7.0; // this is the gear reduction (driven/driving)

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIO4020(int index) {
    switch (index) {
      case 0: // FL
        driveTalon = new TalonFX(4);
        turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        cancoder = new CANcoder(3);
        absoluteEncoderOffset = new Rotation2d(5.28); // ToDo: MUST BE CALIBRATED
        break;
      case 1: // FR
        driveTalon = new TalonFX(2);
        turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        cancoder = new CANcoder(1);
        absoluteEncoderOffset = new Rotation2d(1.92); // ToDo: MUST BE CALIBRATED
        break;
      case 2: // BL
        driveTalon = new TalonFX(6);
        turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        cancoder = new CANcoder(5);
        absoluteEncoderOffset = new Rotation2d(3.30); // ToDo: MUST BE CALIBRATED
        break;
      case 3: // BR
        driveTalon = new TalonFX(8);
        turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        cancoder = new CANcoder(7);
        absoluteEncoderOffset = new Rotation2d(0.49); // ToDo: MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.enableVoltageCompensation(12.0);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();

    turnRelativeEncoder.setPosition(0.0);
    // default encoder sensor filtering may be too much filtering for good control
    // reduce filtering substantially compared to defaults
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);
    driveTalon.optimizeBusUtilization();

    turnSparkMax.setCANTimeout(0);
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnAbsolutePositionRaw =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
