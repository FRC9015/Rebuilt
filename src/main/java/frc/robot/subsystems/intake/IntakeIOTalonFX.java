package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
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
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0);

  private final double kFeedForward = 0.5; // Feedforward value for position control (in volts)

  public IntakeIOTalonFX(int intakeID1, int pivotID1) {
    intakeMotor = new TalonFX(intakeID1);
    pivotMotor = new TalonFX(pivotID1);

    // Configure motor
    TalonFXConfiguration motorConfig = 
      new TalonFXConfiguration()
        .withSlot0(Constants.intakeConstants.intakeSlotConfigs)
        .withFeedback(Constants.intakeConstants.GROUND_FEEDBACK_CONFIGS);

  
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
    inputs.IntakeAppliedVolts = motorVolts.getValueAsDouble();
    inputs.IntakeEncoderPosition = pivotMotor.getPosition().getValueAsDouble();
    inputs.IntakeCurrentSpeed =
     intakeMotor.getVelocity().getValueAsDouble();
    inputs.IntakeRPM = motorRPM.getValueAsDouble();
    inputs.IntakeAppliedVolts = motorVolts.getValueAsDouble();
    inputs.IntakeCurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.IntakeEncoderPosition = motorPosition.getValueAsDouble();
    inputs.IntakeEncoderConnected = false;
    
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

  @Override
  public void setIntakePosition(double position) {
    
    final PositionVoltage pivotPositionControl = new PositionVoltage(0.0);

    pivotMotor.setControl(pivotPositionControl.withPosition(position).withFeedForward(kFeedForward)); 
  } 

  public double getPosition() {
    return intakeMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

}
