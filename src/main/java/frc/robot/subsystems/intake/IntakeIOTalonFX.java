package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class IntakeIOTalonFX implements IntakeIO {

  public final TalonFX intakeMotor;
  public final TalonFX pivotMotor;
  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tunning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tunning/maxPOsition", 1.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage intakeMagicVoltage = new MotionMagicVoltage(0.0);

  private final double kFeedForward = 0.5; // Feedforward value for position control (in volts)

  public IntakeIOTalonFX(int intakeID1, int pivotID1) {
    intakeMotor = new TalonFX(intakeID1);
    pivotMotor = new TalonFX(pivotID1);

    // Configure motor
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.intakeConstants.intakeSlotPositionConfigs)
            .withSlot1(Constants.intakeConstants.intakeSlotVelocityConfigs)
            .withFeedback(Constants.intakeConstants.GROUND_FEEDBACK_CONFIGS)
            .withMotionMagic(Constants.intakeConstants.GROUND_MAGIC_CONFIGS);

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(motorConfig);
    pivotMotor.getConfigurator().apply(motorConfig);

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorVolts = intakeMotor.getMotorVoltage();
    motorAmps = intakeMotor.getStatorCurrent();
    motorRPM = intakeMotor.getVelocity();
    motorPosition = intakeMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM);

    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, pivotMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);
    inputs.intakeAppliedVolts = motorVolts.getValueAsDouble();
    inputs.intakeEncoderPosition = pivotMotor.getPosition().getValueAsDouble();
    inputs.intakeCurrentSpeed = intakeMotor.getVelocity().getValueAsDouble();
    inputs.intakeRPM = motorRPM.getValueAsDouble();
    inputs.intakeAppliedVolts = motorVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.intakeEncoderPosition = motorPosition.getValueAsDouble();
    inputs.intakeEncoderConnected = false;
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void updatePIDFromDashboard() {
    // Implement PID update logic if needed
  }

  @Override
  public void setBrakeMode(boolean enable) {
    intakeMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    pivotMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875
  // Unit of output: RPS
  @Override
  public void setIntakeSpeed(double speed) {
    final VelocityVoltage intakeVelocityVoltage = new VelocityVoltage(0.0).withSlot(1);
    intakeMotor.setControl(
        intakeVelocityVoltage.withVelocity(
            MathUtil.clamp(
                speed,
                Constants.intakeConstants.INTAKE_MIN_SPEED,
                Constants.intakeConstants.INTAKE_MAX_SPEED)));
  }

  @Override
  public void setIntakePosition(double position) {

    final double clampedPosition =
        MathUtil.clamp(
            position,
            Constants.intakeConstants.INTAKE_MIN_POS,
            Constants.intakeConstants.INTAKE_MAX_POS);

    pivotMotor.setControl(intakeMagicVoltage.withPosition(clampedPosition));
  }

  public double getPosition() {
    return intakeMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }
}
