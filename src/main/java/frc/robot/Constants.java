// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** The runtime mode for the robot (real, simulation, or replay). */
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class MotorIDConstants {
    public static final int TURRET_MOTOR_ID = 60;

    public static final int INTAKE_ROLLER_LEFT_ID = 51;
    public static final int INTAKE_ROLLER_RIGHT_ID = 52;
    public static final int INTAKE_PIVOT_LEFT_ID = 53;
    public static final int INTAKE_ENCODER_ID = 50;
    public static final int INDEXER1_MOTOR_ID = 5;
    public static final int INDEXER2_MOTOR_ID = 6;
  }

  public static class RobotDimensionConstants {
    public static final Distance BUMPER_THICKNESS = Inches.of(5.9375); // frame to edge of bumper
    public static final Distance FRAME_SIZE_Y = Inches.of(30.5); // left to right (y-axis)
    public static final Distance FRAME_SIZE_X = Inches.of(23.5); // front to back (x-axis)

    public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
    public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
  }

  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);

    public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

    public static final Distance FUNNEL_RADIUS = Inches.of(24);
    public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

    public static final Distance TRENCH_BUMP_X =
        Inches.of(181.56); // x position of the center of the trench and bump
    public static final Distance TRENCH_WIDTH = Inches.of(49.86); // y width of the trench
    public static final Distance TRENCH_BUMP_LENGTH =
        Inches.of(47); // x length of the trench and bump
    public static final Distance TRENCH_BAR_WIDTH = Inches.of(4); // x width of the trench bar
    public static final Distance TRENCH_BLOCK_WIDTH =
        Inches.of(12); // y width of block separating bump and trench
    public static final Distance BUMP_WIDTH = Inches.of(73); // y width of bump

    public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);

    public static final Pose2d HUB_POSE_BLUE =
        new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.845), new Rotation2d());
    public static final Pose2d HUB_POSE_RED = FlippingUtil.flipFieldPose(HUB_POSE_BLUE);
    public static final Translation3d HUB_TARGET_TRANSLATION =
        new Translation3d(
            Units.inchesToMeters(182.11), Units.inchesToMeters(158.845), Units.inchesToMeters(72));
    public static final Translation3d HUB_TARGET_TOLERANCE =
        new Translation3d(
            Units.inchesToMeters(24), Units.inchesToMeters(21), Units.inchesToMeters(0.02));
  }

  public static class ZoneConstants {
    public static final Distance EXTRA_DUCK_DISTANCE = Meters.of(0.5);
  }

  public static class VisionConstants {
    public static final double MAX_AMBIGUITY = 0.3;
    public static final int MAX_AVERAGE_DISTANCE = 3;
    public static final int STD_DEV_RANGE = 30;

    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double FIELD_LENGTH = aprilTagLayout.getFieldLength();
    public static final double FIELD_WIDTH = aprilTagLayout.getFieldWidth();
    public static final Transform3d FRONT_CAMERA =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(3.5),
                Units.inchesToMeters(-15.25),
                Units.inchesToMeters(7.25)),
            new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(270)));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(5, 5, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
  /** Configuration and tuning constants for the intake mechanism. */
  public static class IntakeConstants {
    public static final Slot0Configs intakeSlotPositionConfigs =
        new Slot0Configs()
            .withKP(0.001)
            .withKI(0)
            .withKD(0.02)
            .withKG(0.01)
            .withKA(0)
            .withKS(0)
            .withKV(0);
    public static final Slot0Configs ROLLER_SLOT0_CONFIGS =
        new Slot0Configs().withKP(2).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    public static final Slot0Configs PIVOT_SLOT0_CONFIGS =
        new Slot0Configs().withKP(2).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    public static final MotionMagicConfigs PIVOT_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final MotionMagicConfigs ROLLER_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(50).withMotionMagicCruiseVelocity(5);

    public static final FeedbackConfigs PIVOT_FEEDBACK_CONFIGS =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    private static final MotorOutputConfigs rollerOutputLeftConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration rollerConfigLeft =
        new TalonFXConfiguration()
            .withMotorOutput(rollerOutputLeftConfigs)
            .withSlot0(ROLLER_SLOT0_CONFIGS)
            .withMotionMagic(ROLLER_MAGIC_CONFIGS);

    private static final MotorOutputConfigs rollerOutputRightConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration rollerConfigRight =
        new TalonFXConfiguration()
            .withMotorOutput(rollerOutputRightConfigs)
            .withSlot0(ROLLER_SLOT0_CONFIGS)
            .withMotionMagic(ROLLER_MAGIC_CONFIGS);

    private static final MotorOutputConfigs pivotOutputLeftConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    public static final TalonFXConfiguration pivotConfigLeft =
        new TalonFXConfiguration()
            .withSlot0(PIVOT_SLOT0_CONFIGS)
            .withFeedback(PIVOT_FEEDBACK_CONFIGS)
            .withMotionMagic(PIVOT_MAGIC_CONFIGS)
            .withMotorOutput(pivotOutputLeftConfigs);

    private static final MotorOutputConfigs pivotOutputRightConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    public static final TalonFXConfiguration pivotConfigRight =
        new TalonFXConfiguration()
            .withSlot0(PIVOT_SLOT0_CONFIGS)
            .withFeedback(PIVOT_FEEDBACK_CONFIGS)
            .withMotionMagic(PIVOT_MAGIC_CONFIGS)
            .withMotorOutput(pivotOutputRightConfigs);

    public static final double INTAKE_MAX_POS = 300.0;
    public static final double INTAKE_MIN_POS = 0.0;
    public static final double INTAKE_MAX_SPEED = 512.0;
    public static final double INTAKE_MIN_SPEED = -511.0;

    public static final double PIVOT_MAX_POS = 300.0;
    public static final double PIVOT_MIN_POS = 0.0;
    public static final double PIVOT_DEPLOYED_POSITION = 100.0;
    public static final double PIVOT_STOWED_POSITION = 10.0;
  }

  public static class SimConstants {
    // Simulation constants (e.g., physics parameters) can be added here
    public static final double INTAKE_LENGTH = 0.2;
    public static final double INTAKE_WIDTH = 0.7;
    public static final int HOPPER_CAPACITY = 50;
    public static final String GAMEPIECE = "Fuel";
  }

  public static class ShooterConstants {
    //  Following naming scheme for subsystem motor and sensor ids
    public static final int FLY_WHEEL_LEFT_ID = 56;
    public static final int FLY_WHEEL_RIGHT_ID = 57;
    public static final int HOOD_ID = 58;
    public static final int KICKER_ID = 59;
    public static final double FLYWHEEL_RPM_TOLERANCE = 10.0; // TODO: tune this value
    public static final double HOOD_RESTING_ANGLE = 10.0;

    public static final Slot0Configs flyWheelSlotVelocityConfigs =
        new Slot0Configs().withKP(0).withKI(0).withKD(0).withKG(0).withKA(0).withKS(0).withKV(0);
    // TODO: Tune kicker PID values
    public static final Slot0Configs kickerSlotVelocityConfigs =
        new Slot0Configs().withKP(0).withKI(0).withKD(0).withKG(0).withKA(0).withKS(0).withKV(0);

    public static final FeedbackConfigs kickerFeedbackConfigs =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // TODO tune these values once final bot comes; reference this link:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html

    public static final Slot1Configs hoodSlotPositionConfigs =
        new Slot1Configs()
            .withKP(0.0)
            .withKI(0)
            .withKD(0.0)
            .withKG(0.0)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    // TODO tune these values once final bot comes.

    public static final MotionMagicConfigs hoodMagicConfigs =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final FeedbackConfigs hoodFeedbackConfigs =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // TODO tune these values
    public static final double SHOOTER_MAX_SPEED = 100;
    public static final double SHOOTER_MIN_SPEED = 0.0;
    public static final double FLYWHEEL_ACCELERATION = 100;
    public static final double FEEDFORWARD_VOLTAGE = 12.0;
    public static final double HOOD_MAX_POS = 0.0;
    public static final double HOOD_MIN_POS = 0.0;
    public static final double HOOD_DEPLOYED_POSITION = 0.0;
    public static final double HOOD_STOWED_POSITION = 0.0;
  }

  public static class LedConstants {
    public static final int CANDLE_ID1 = 0; // TODO: replace with actual CAN ID
    public static final double DEFAULT_STROBE_FRAME_RATE = 50.0;
  }

  public static class ClimbConstants {
    public static final Slot0Configs climbSlot0Configs =
        // TODO: Tune these values once final bot comes
        new Slot0Configs()
            .withKP(0.1)
            .withKI(0)
            .withKD(0)
            .withKG(0.01)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final FeedbackConfigs climbFeedbackConfigs =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    public static final MotionMagicConfigs climbMagicConfigs =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final double CLIMB_MAX_POS = 300.0;
    public static final double CLIMB_MIN_POS = 0.0;

    public static final double CLIMB_POSITION_TOLERANCE = 0.01; // TODO: Tune this value
  }

  public static class TurretConstants {
    // --- GEAR TEETH ---
    public static final int T_TEETH = 90; // Gear count on final turret gear
    public static final int E1_TEETH = 13; // Gear on Encoder 1
    public static final int E2_TEETH = 15; // Gear on Encoder 2

    public static final int ENCODER_13_TOOTH = 61; // Encoder 13 motor id
    public static final int ENCODER_15_TOOTH = 62; // Encoder 15 motor id

    // --- MATH CONSTANTS ---
    /** The error allowance (in turret rotations) when comparing encoder predictions. */
    public static final double CRT_TOLERANCE = 0.034;

    /**
     * The difference threshold between calculated and internal motor position to trigger a re-seed.
     */
    public static final double SYNC_THRESHOLD = 0.05;

    /** Search limit for Encoder 1 (should be equal to e2_teeth). */
    public static final int E1_SEARCH_LIMIT = (int) E2_TEETH;
    /** Search limit for Encoder 2 (should be equal to e1_teeth). */
    public static final int E2_SEARCH_LIMIT = (int) E1_TEETH;

    // --- MOVEMENT LIMITS ---
    public static final double MAXROTATION = 0.7;
    public static final double MINROTATION = -0.7;

    public static final double ENCODER13_MAGNET_OFFSET = -0.2917480;
    public static final double ENCODER15_MAGNET_OFFSET = -0.3728027;

    // total gear ratio on turret
    public static final double ENCODER_TO_TURRET_GEAR_RATIO = 37.5;
    // --- MOTOR CONFIGS ---
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(150).withMotionMagicCruiseVelocity(25);
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(12).withKI(0.0).withKD(0).withKG(0).withKA(0).withKS(0).withKV(0);
    public static final FeedbackConfigs FEEDBACK_CONFIGS =
        new FeedbackConfigs()
            .withSensorToMechanismRatio(ENCODER_TO_TURRET_GEAR_RATIO)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
  }
}
