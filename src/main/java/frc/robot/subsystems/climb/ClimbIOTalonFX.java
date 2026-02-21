package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** TalonFX-based I/O implementation for the climb subsystem. */
public class ClimbIOTalonFX implements ClimbIO {
  public final TalonFX climbMotor1;
  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tuning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tuning/maxPosition", 1.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0);

  // Constants extracted to avoid magic numbers
  private static final double STATUS_UPDATE_FREQUENCY = 50.0;
  private static final double MAX_OUTPUT_VOLTAGE = 12.0;

  public ClimbIOTalonFX(int climbID1) {
    climbMotor1 = new TalonFX(climbID1);

    // Configure motor
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.climbConstants.climbSlot0Configs)
            .withFeedback(Constants.climbConstants.CLIMB_FEEDBACK_CONFIGS)
            .withMotionMagic(Constants.climbConstants.CLIMB_MAGIC_CONFIGS);
    
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    // Apply Config
    climbMotor1.getConfigurator().apply(motorConfig);

    motorVolts = climbMotor1.getMotorVoltage();
    motorAmps = climbMotor1.getStatorCurrent();
    motorRPM = climbMotor1.getVelocity();
    motorPosition = climbMotor1.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(
        STATUS_UPDATE_FREQUENCY, motorVolts, motorAmps, motorRPM, motorPosition);
    ParentDevice.optimizeBusUtilizationForAll(climbMotor1);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);
    inputs.climberAppliedVolts = motorVolts.getValueAsDouble();
    inputs.climberCurrentAmps = motorAmps.getValueAsDouble();
    inputs.climberRPM = motorRPM.getValueAsDouble();
    inputs.climberPosition = motorPosition.getValueAsDouble();
  }

  @Override
  public void stop() {
    climbMotor1.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    com.ctre.phoenix6.configs.MotorOutputConfigs config = new com.ctre.phoenix6.configs.MotorOutputConfigs();
    climbMotor1.getConfigurator().refresh(config);
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    climbMotor1.getConfigurator().apply(config);
  }

  @Override
  public void setClimbVoltage(double voltage) {
    climbMotor1.setVoltage(MathUtil.clamp(voltage, -MAX_OUTPUT_VOLTAGE, MAX_OUTPUT_VOLTAGE));
  }

  @Override
  public void setClimbPosition(double position) {
    final double clampedPosition =
        MathUtil.clamp(
            position,
            Constants.climbConstants.CLIMB_MIN_POS,
            Constants.climbConstants.CLIMB_MAX_POS);
    climbMotor1.setControl(positionVoltage.withPosition(clampedPosition));
  }

  public double getPosition() {
    return climbMotor1.getPosition().getValueAsDouble();
  }
}
