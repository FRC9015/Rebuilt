package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeIOTalonFX implements IntakeIO {

  public final TalonFX intakeMotor;
  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tunning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tunning/maxPOsition", 1.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0);

  public IntakeIOTalonFX(int intakeID1) {
    intakeMotor = new TalonFX(intakeID1);

    // Configure motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(motorConfig);
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorVolts = intakeMotor.getMotorVoltage();
    motorAmps = intakeMotor.getStatorCurrent();
    motorRPM = intakeMotor.getVelocity();
    motorPosition = intakeMotor.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);
    inputs.IntakeAppliedVolts = motorVolts.getValueAsDouble();
    inputs.IntakeCurrentAmps = motorAmps.getValueAsDouble();
    inputs.IntakeRPM = motorRPM.getValueAsDouble();
    inputs.IntakeEncoderPosition = motorPosition.getValueAsDouble();
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    intakeMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setIntakeSpeed(double voltage) {
    intakeMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public double getPosition() {
    return intakeMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }
}