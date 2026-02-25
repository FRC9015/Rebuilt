package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
  public final TalonFX kickerMotor;
  public final TalonFX hoodMotor;

  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tuning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tuning/maxPosition", 1.0);
  private final MotionMagicVoltage hoodMagicVoltage =
      new MotionMagicVoltage(Constants.ShooterConstants.HOOD_MAX_POS);
  private final MotionMagicVelocityVoltage flywheelMagicVelocityVoltage =
      new MotionMagicVelocityVoltage(0.0);
  private final MotionMagicVelocityVoltage kickerMagicVelocityVoltage =
      new MotionMagicVelocityVoltage(0.0);

  private final double idealKickerSpeed = 0.0; // TODO figure out ideal kicker speed

  public ShooterIOTalonFX(int flywheelID1, int flywheelID2, int hoodID, int kickerID) {
    flywheelMotorLeft = new TalonFX(flywheelID1);
    flywheelMotorRight = new TalonFX(flywheelID2);
    kickerMotor = new TalonFX(kickerID);
    hoodMotor = new TalonFX(hoodID);
    // Configure motor
    TalonFXConfiguration flyWheelConfigLeft =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs);

    flyWheelConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    flyWheelConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withSlot1(Constants.ShooterConstants.hoodSlotPositionConfigs)
            .withFeedback(Constants.ShooterConstants.hoodFeedbackConfigs)
            .withMotionMagic(Constants.ShooterConstants.hoodMagicConfigs);

    TalonFXConfiguration flyWheelConfigRight =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.flyWheelSlotVelocityConfigs);

    flyWheelConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flyWheelConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration kickerConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.kickerSlotVelocityConfigs)
            .withFeedback(Constants.ShooterConstants.kickerFeedbackConfigs);

    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxPosition.get();
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minPosition.get();
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    flywheelMotorLeft.getConfigurator().apply(flyWheelConfigLeft);
    flywheelMotorRight.getConfigurator().apply(flyWheelConfigRight);
    kickerMotor.getConfigurator().apply(kickerConfig);  
    hoodMotor.getConfigurator().apply(hoodConfig);

    motorVolts = flywheelMotorLeft.getMotorVoltage();
    motorAmps = flywheelMotorLeft.getStatorCurrent();
    motorRPM = flywheelMotorLeft.getVelocity();
    motorPosition = hoodMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM, motorPosition);

    ParentDevice.optimizeBusUtilizationForAll(flywheelMotorLeft, flywheelMotorRight, kickerMotor, hoodMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);
    inputs.flywheelAppliedVolts = motorVolts.getValueAsDouble();
    inputs.hoodEncoderPosition = hoodMotor.getPosition().getValueAsDouble();
    inputs.flywheelCurrentSpeed = flywheelMotorLeft.getVelocity().getValueAsDouble();
    inputs.flywheelRPM = motorRPM.getValueAsDouble();
    inputs.flywheelCurrentAmps = flywheelMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.hoodEncoderConnected = hoodMotor.getPosition().isAllGood();
  }

  @Override
  public void stopFlywheels() {
    flywheelMotorLeft.stopMotor();
    flywheelMotorRight.stopMotor();
    kickerMotor.stopMotor();
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

  // Minimum Value of speedValue: -100 RPS
  // Maximum Value of speedValue: 100 RPS
  // Unit of output: RPS
  @Override
  public void setFlyWheelSpeed(double speed) {

    flywheelMotorLeft.setControl(flywheelMagicVelocityVoltage.withVelocity(speed));
    flywheelMotorRight.setControl(flywheelMagicVelocityVoltage.withVelocity(speed));

    //Running kicker motor along with flywheels since it's required to shoot
    kickerMotor.setControl(kickerMagicVelocityVoltage.withVelocity(idealKickerSpeed));
  }

  @Override
  public void setHoodPosition(double position) {
    final double clampedPosition =
        MathUtil.clamp(
            position,
            Constants.ShooterConstants.HOOD_MIN_POS,
            Constants.ShooterConstants
                .HOOD_MAX_POS); // TODO figure out max and min position for Hood

    hoodMotor.setControl(hoodMagicVoltage.withPosition(clampedPosition).withSlot(0));
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
