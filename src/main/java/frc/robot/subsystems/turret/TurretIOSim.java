package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
  private final TalonFX turretMotor;
  private final TalonFXSimState turretMotorSim;
  private final DCMotorSim turretPhysicsSim;
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private double setpointDegrees = 0.0;

  // Simulation Physics Constants
  private static final double TURRET_GEARING = TurretConstants.ENCODER_TO_TURRET_GEAR_RATIO;
  private static final double TURRET_MOI_KG_M2 = 0.1463;

  public TurretIOSim() {
    turretMotor = new TalonFX(MotorIDConstants.TURRET_MOTOR_ID);
    turretMotorSim = turretMotor.getSimState();
    var turretPlant =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), TURRET_MOI_KG_M2, TURRET_GEARING);

    turretPhysicsSim = new DCMotorSim(turretPlant, DCMotor.getKrakenX44(1));
    turretMotorSim.setSupplyVoltage(12.0);

    // Motor Config
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(TurretConstants.MAXROTATION)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(TurretConstants.MINROTATION))
            .withMotionMagic(TurretConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(TurretConstants.SIM_CONFIGS)
            .withFeedback(TurretConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(60.0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(60.0)
                    .withSupplyCurrentLimitEnable(true));

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    turretMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // 1. CHASSIS COMPENSATION (The "Smart" part)

    // Pull the actual gyro/chassis angular velocity here.

    // If the robot is spinning, we need to tell the motor sim so it doesn't fight the turn.

    // Replace 0.0 with your DriveSubsystem's angular velocity in Radians Per Second.

    double chassisAngularVelocityRadPerSec = 0.0;

    // 2. HIGH-RESOLUTION PHYSICS STEP

    // Running 20 steps of 1ms is MUCH more stable than 1 step of 20ms.

    // This prevents the PID from "teleporting" and oscillating.

    for (int i = 0; i < 20; i++) {

      turretPhysicsSim.setInputVoltage(turretMotorSim.getMotorVoltage());

      turretPhysicsSim.update(0.001);
    }

    // 3. GET POSITION AND WRAP

    double rawMechPos = turretPhysicsSim.getAngularPositionRotations();

    // We wrap the position so the PID always takes the shortest path

    double wrappedPos =
        MathUtil.inputModulus(rawMechPos, TurretConstants.MINROTATION, TurretConstants.MAXROTATION);

    // 4. SYNC TALON SIM STATE

    // We update the Talon with the wrapped position so it never thinks it's multiple rotations

    // away.

    turretMotorSim.setRawRotorPosition(wrappedPos * TURRET_GEARING);

    // VELOCITY SYNC:

    // Motor Velocity = (Internal Physics Speed) + (External Robot Rotation Speed)

    double motorVelRotPerSec = (turretPhysicsSim.getAngularVelocityRPM() / 60.0);

    double chassisVelRotPerSec = chassisAngularVelocityRadPerSec / (2.0 * Math.PI);

    // This tells the Talon: "You are already moving this fast because the robot is turning."

    turretMotorSim.setRotorVelocity((motorVelRotPerSec + chassisVelRotPerSec) * TURRET_GEARING);

    // 5. UPDATE IO INPUTS

    inputs.encoder13Connected = true;

    inputs.encoder15Connected = false;

    inputs.encoder13PositionRot = wrappedPos;

    inputs.turretAppliedVolts = turretMotorSim.getMotorVoltage();

    inputs.turretCurrentAmps = turretPhysicsSim.getCurrentDrawAmps();

    inputs.turretMotorPosition = wrappedPos;

    inputs.turretSetpoint = setpointDegrees;

    inputs.turretResolvedPosition = wrappedPos;

    // Calculate error in degrees for logging/AdvantageScope

    inputs.turretError = Math.abs(inputs.turretResolvedPosition - inputs.turretSetpoint) * 360.0;
  }

  @Override
  public void seedMotorPosition(double positionRotations) {

    turretMotor.setPosition(positionRotations);

    turretMotorSim.setRawRotorPosition(positionRotations * TURRET_GEARING);
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
  public void setTurretPosition(double positionDegrees) {

    double rotations = positionDegrees / 360.0;

    double safePosition = MathUtil.clamp(rotations, -0.7, 0.7);

    turretMotor.setControl(motionMagicVoltage.withPosition(safePosition));

    setpointDegrees = safePosition;
  }

  @Override
  public void setTurretVoltage(double voltage) {

    turretMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setTurretSetPoint(double value) {

    setpointDegrees = value;
  }
}
