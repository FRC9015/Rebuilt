package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.turretConstants;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final CANcoder encoder13;
  private final CANcoder encoder15;

  private final StatusSignal<Angle> encoder13Pos;
  private final StatusSignal<Angle> encoder15Pos;
  private final StatusSignal<Voltage> motorVolts;
  private final StatusSignal<Current> motorAmps;
  private final StatusSignal<Angle> motorPos;
  private int count = 0;
  ;

  // Filters to stop the jumping by smoothing the raw encoder data
  private final MedianFilter filter13 = new MedianFilter(5);
  private final MedianFilter filter15 = new MedianFilter(5);

  public TurretIOTalonFX(int motorID, int encoderId13, int encoderId15) {
    turretMotor = new TalonFX(motorID);
    encoder13 = new CANcoder(encoderId13);
    encoder15 = new CANcoder(encoderId15);

    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(turretConstants.MAXROTATION)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(turretConstants.MINROTATION))
            .withMotionMagic(turretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(turretConstants.SLOT0_CONFIGS)
            .withFeedback(turretConstants.FEEDBACK_CONFIGS);

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretMotor.getConfigurator().apply(motorConfig);

    CANcoderConfiguration encConfig = new CANcoderConfiguration();
    encConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoder13.getConfigurator().apply(encConfig);
    encoder15.getConfigurator().apply(encConfig);

    encoder13Pos = encoder13.getAbsolutePosition();
    encoder15Pos = encoder15.getAbsolutePosition();
    motorVolts = turretMotor.getMotorVoltage();
    motorAmps = turretMotor.getStatorCurrent();
    motorPos = turretMotor.getPosition();

    // High update frequency for resolver signals
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, encoder13Pos, encoder15Pos);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Wait for signals to be synced in time
    BaseStatusSignal.waitForAll(0.02, encoder13Pos, encoder15Pos, motorVolts, motorAmps, motorPos);

    // Apply Median Filters to raw data to remove spikes/noise
    double r13 = filter13.calculate(encoder13Pos.getValueAsDouble());
    double r15 = filter15.calculate(encoder15Pos.getValueAsDouble());

    if (turretConstants.INVERT_ENCODERS) {
      inputs.encoder13PositionRot = r13;
      inputs.encoder15PositionRot = r15; // MathUtil.inputModulus(1.0 - r15, 0, 1.0);
    } else {
      inputs.encoder13PositionRot =
          encoder13Pos.getValueAsDouble(); // MathUtil.inputModulus(r13, 0, 1.0);
      inputs.encoder15PositionRot =
          encoder15Pos.getValueAsDouble(); // MathUtil.inputModulus(r15, 0, 1.0);
    }

    inputs.turretMotorPosition = motorPos.getValueAsDouble();
    inputs.turretAppliedVolts = motorVolts.getValueAsDouble();
    inputs.turretCurrentAmps = motorAmps.getValueAsDouble();

    // Pure Mathematical Resolver
    Double resolved = resolveAbsolute(inputs.encoder13PositionRot, inputs.encoder15PositionRot);
    inputs.turretResolvedPosition = motorPos.getValueAsDouble();
    // if (resolved != null) {
    //   inputs.turretResolvedValid = true;
    //   inputs.turretResolvedPosition = resolved;
    // } else {
    //   inputs.turretResolvedValid = false;
    // }
  }

  private Double resolveAbsolute(double raw13, double raw15) {
    double bestMatch = -1;
    double minError = Double.MAX_VALUE;
    double e13 = 0.0;
    double e15 = 0.0;
    double d13 = 0.0;
    double d15 = 0.0;
    int idk = 0;
    double val13 = MathUtil.inputModulus(raw13, 0, 1.0);
    double val15 = MathUtil.inputModulus(raw15, 0, 1.0);

    if (count == 12) {
      count = 0;
    }
    if (raw15 >= 0.95 && raw15 <= 1.05) {
      count++;
    }
    if (count >= 6) {
      idk = 1;
    }
    return (count / 6) + (raw15 / 6);

    // loops:
    // {
    //   for (int tooth = 0; tooth < (int) turretConstants.E1_TEETH; tooth++) {
    //     e13 = (tooth % 13) / 13.0;
    //     d13 = (tooth + val13) * (turretConstants.E1_TEETH / turretConstants.T_TEETH);
    //     // Circular distance in "Teeth" uni1ts
    //     // d13 = Math.abs(MathUtil.inputModulus(e13 - raw13, 0, 1)) * 13.0;
    //     System.out.println("1");
    //     for (int tooth2 = 0; tooth2 < (int) turretConstants.E2_TEETH; tooth2++) {
    //       d15 = (tooth2 + val15) * (turretConstants.E2_TEETH / turretConstants.T_TEETH);
    //       // d15 = Math.abs(MathUtil.inputModulus(e15 - raw15, 0, 1)) * 15.0;
    //       System.out.println(d13 - d15);
    //       if (Math.abs(d13 - d15) < turretConstants.CRT_TOOTH_TOLERANCE) {
    //         double totalError = d13 + d15;
    //         // Score based: pick the most perfect match
    //         // if (totalError < minError) {
    //         minError = totalError;
    //         System.out.println("3");
    //         bestMatch = d13;
    //         break loops;
    //       }
    //     }
    //   }

    // return bestMatch != -1 ? bestMatch : null;
  }

  @Override
  public void seedMotorPosition(double pos) {
    turretMotor.setPosition(pos);
  }

  @Override
  public void stop() {
    turretMotor.stopMotor();
  }

  @Override
  public void setBrakeMode() {
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCoastMode() {
    turretMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void setTurretPosition(double pos) {
    double safe = MathUtil.clamp(pos, turretConstants.MINROTATION, turretConstants.MAXROTATION);
    turretMotor.setControl(new MotionMagicVoltage(safe));
  }
}
