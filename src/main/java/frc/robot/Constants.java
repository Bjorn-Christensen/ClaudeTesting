package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
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

public final class Constants {

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Drivetrain Constants
  public static final class DrivetrainConstants {
    public static final double MAX_SPEED = Units.feetToMeters(19.5); // Theoretically: ~18-20
    public static final double WHEEL_LOCK_TIME = 5; // Hold time on motor brakes when disabled (seconds)
    public static final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  }

  // Drive Joystick Constants
  public static class ControllerConstants {
    public static final double DEADZONE = 0.2;
    public static final double LEFT_TRIGGER_DEADZONE = 0.1, RIGHT_TRIGGER_DEADZONE = 0.1;
  }

  // April Tags
  public static class FieldConstants {
    public static final int[] HUMAN_LOAD_TAGS_BLUE = {29,30};
    public static final int[] HUMAN_LOAD_TAGS_RED = {13,14};
    public static final Pose2d HUB_POSE_BLUE = new Pose2d(
        Units.inchesToMeters(182.11),
        Units.inchesToMeters(158.84),
        new Rotation2d());
    public static final Pose2d HUB_POSE_RED = new Pose2d(
        Units.inchesToMeters(651.22) - Units.inchesToMeters(182.11),
        Units.inchesToMeters(158.84),
        new Rotation2d());
    public static final double HUB_STANDOFF_DISTANCE = 2.0; // meters from hub center to ideal shooting position
  }

  // Vision setup
  public static class VisionConstants {
    // Set to false when cameras are not physically connected (e.g. shooter testing)
    public static final boolean CAMERAS_ENABLED = true;

    // One entry per camera you have configured in PhotonVision
    public static final List<CameraConfig> CAMERAS = List.of(
      new CameraConfig(
        "frontCam",
        new Transform3d(new Translation3d(0.25, 0.0, Units.inchesToMeters(6.0)),
        new Rotation3d(0.0, Math.toRadians(0.0), 0.0))
      ),

      new CameraConfig(
        "backCam",
        new Transform3d(new Translation3d(-0.25, 0.0, Units.inchesToMeters(6.0)),
        new Rotation3d(0.0, Math.toRadians(0.0),  Math.toRadians(180)))
      )
    );
    

    // Std-devs when we only trust a single tag (meters for x/y, radians for heading)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
      VecBuilder.fill(1.5, 1.5, Math.toRadians(35.0));

    // Std-devs when multiple tags contribute to the estimate
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
      VecBuilder.fill(0.3, 0.3, Math.toRadians(7.0));
  }

  // Compact config holder for each Photon camera
  public static final class CameraConfig {
    public final String name;
    public final Transform3d robotToCam;

    public CameraConfig(String name, Transform3d robotToCam) {
      this.name = name;
      this.robotToCam = robotToCam;
    }
  }

  // Shooter Constants
  public static class ShooterConstants {
    // CAN IDs — update to match your robot's CAN bus
    public static final int FEED_MOTOR_ID        = 10;
    public static final int FLYWHEEL_MOTOR_ID    = 11;
    public static final int BACK_ROLLER_MOTOR_ID = 12;

    // Velocity PID — Neo Vortex starting values (tune on carpet)
    // kV = 12V / 6784 RPM (Neo Vortex free speed) — units: Volts per RPM
    public static final double FEED_KP  = 0.0002, FEED_KI  = 0.0, FEED_KD  = 0.0, FEED_KV  = 0.001769;
    public static final double FLYWHEEL_KP  = 0.0002, FLYWHEEL_KI  = 0.0, FLYWHEEL_KD  = 0.0, FLYWHEEL_KV  = 0.001769;
    public static final double BACK_ROLLER_KP = 0.0002, BACK_ROLLER_KI = 0.0, BACK_ROLLER_KD = 0.0, BACK_ROLLER_KV = 0.001769;

    // Flywheel considered at speed within this many RPM of target
    public static final double FLYWHEEL_RPM_TOLERANCE = 50.0;

    // Default RPMs
    public static final double DEFAULT_FLYWHEEL_RPM = 4000.0;
    public static final double DEFAULT_FEED_RPM     = 1500.0;
  }

}
