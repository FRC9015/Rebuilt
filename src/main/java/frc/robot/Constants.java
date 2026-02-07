// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class CameraConstants {
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double cameraHeight = Units.inchesToMeters(7);
    public static final double cameraPitch = Units.degreesToRadians(15);
    public static final double cameraXOffset = Units.inchesToMeters(13.5);
    public static final Transform3d placeHolderCamera =
        new Transform3d(
            new Translation3d(cameraXOffset, Units.inchesToMeters(0), cameraHeight),
            new Rotation3d(0, cameraPitch, Units.degreesToRadians(0)));
    
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(5, 5, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class VisionConstants {
    public static final double MAX_AMBIGUITY = 0.1;
    public static final int MAX_AVERAGE_DISTANCE = 3;
    public static final int STD_DEV_RANGE = 30;
  }
}
