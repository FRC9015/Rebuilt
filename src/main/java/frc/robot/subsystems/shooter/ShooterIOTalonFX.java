package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOTalonFX implements ShooterIO {

  public final TalonFX flywheelMotorLeft;
  public final TalonFX flywheelMotorRight;
  public final TalonFX hoodMotor;
  //   public final CANcoder hoodEncoder;
  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tuning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tuning/maxPosition", 1.0);
  private final MotionMagicVoltage hoodMagicVoltage = new MotionMagicVoltage(122);

  public ShooterIOTalonFX(int flywheelID1, int flywheelID2, int hoodID) {
    flywheelMotorLeft = new TalonFX(flywheelID1);
    flywheelMotorRight = new TalonFX(flywheelID2);
    hoodMotor = new TalonFX(hoodID);
    // hoodEncoder = new CANcoder(hoodEncoderID);
    // Configure motor
    TalonFXConfiguration flyWheelConfigLeft =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs);

    TalonFXConfiguration flyWheelConfigRight =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs);

    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.hoodSlotPositionConfigs)
            .withFeedback(Constants.ShooterConstants.HOOD_FEEDBACK_CONFIGS)
            .withMotionMagic(Constants.ShooterConstants.HOOD_MAGIC_CONFIGS);

    flyWheelConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flyWheelConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flyWheelConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxPosition.get();
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minPosition.get();
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    flywheelMotorLeft.getConfigurator().apply(flyWheelConfigLeft);
    flywheelMotorRight.getConfigurator().apply(flyWheelConfigRight);
    hoodMotor.getConfigurator().apply(hoodConfig);

    motorVolts = flywheelMotorLeft.getMotorVoltage();
    motorAmps = flywheelMotorLeft.getStatorCurrent();
    motorRPM = flywheelMotorLeft.getVelocity();
    motorPosition = hoodMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM, motorPosition);

    ParentDevice.optimizeBusUtilizationForAll(flywheelMotorLeft, flywheelMotorRight, hoodMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);
    inputs.flywheelAppliedVolts = motorVolts.getValueAsDouble();
    inputs.hoodEncoderPosition = hoodMotor.getPosition().getValueAsDouble();
    inputs.flywheelCurrentSpeed = flywheelMotorLeft.getVelocity().getValueAsDouble();
    inputs.flywheelRPM = motorRPM.getValueAsDouble();
    inputs.flywheelAppliedVolts = motorVolts.getValueAsDouble();
    inputs.flywheelCurrentAmps = flywheelMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.hoodEncoderConnected = false;
  }

  @Override
  public void stopFlywheels() {
    flywheelMotorLeft.stopMotor();
    flywheelMotorRight.stopMotor();
  }

  @Override
  public void stopHood() {
    hoodMotor.stopMotor();
  }

  @Override
  public void stopShooter() {
    stopFlywheels();
    stopHood();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    flywheelMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    flywheelMotorRight.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    hoodMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875
  // Unit of output: RPS
  @Override
  public void setFlyWheelSpeed(double speed) {
    final VelocityVoltage flyWheelVelocityVoltage = new VelocityVoltage(0.0);
    flywheelMotorLeft.setControl(
        flyWheelVelocityVoltage
            .withVelocity(speed)
            .withAcceleration(Constants.ShooterConstants.FLYWHEEL_ACCELERATION)
            .withFeedForward(Constants.ShooterConstants.FEEDFORWARD_VOLTAGE)
            .withSlot(0));

    flywheelMotorRight.setControl(
        new Follower(flywheelMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void setHoodPosition(double position) {
    final double clampedPosition =
        MathUtil.clamp(
            position,
            Constants.ShooterConstants.HOOD_MIN_POS,
            Constants.ShooterConstants.HOOD_MAX_POS);

    hoodMotor.setControl(hoodMagicVoltage.withPosition(clampedPosition).withSlot(0));
  }

  public double getPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getFlyWheelSpeed() {
    double averageVelocity =
        (flywheelMotorLeft.getVelocity().getValueAsDouble()
                + flywheelMotorRight.getVelocity().getValueAsDouble())
            / 2;
    return averageVelocity;
  }
}
