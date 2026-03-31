package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    public static final boolean CAMERAS_ENABLED = false;

    // One entry per camera you have configured in PhotonVision.
    // Height: 25 inches above robot center. Pitch: slightly upward to see field AprilTags.
    // Adjust CAMERA_PITCH_DEG if tags appear at the edge of frame — increase to look higher.
    public static final double CAMERA_HEIGHT_INCHES = 25.0;
    public static final double CAMERA_PITCH_DEG     = 15.0; // positive = nose up

    public static final List<CameraConfig> CAMERAS = List.of(
      new CameraConfig(
        "frontCam",
        new Transform3d(
          new Translation3d(0.25, 0.0, Units.inchesToMeters(CAMERA_HEIGHT_INCHES)),
          new Rotation3d(0.0, Math.toRadians(CAMERA_PITCH_DEG), 0.0))
      ),

      new CameraConfig(
        "backCam",
        new Transform3d(
          new Translation3d(-0.25, 0.0, Units.inchesToMeters(CAMERA_HEIGHT_INCHES)),
          new Rotation3d(0.0, Math.toRadians(CAMERA_PITCH_DEG), Math.toRadians(180.0)))
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

  // Intake Constants
  public static class IntakeConstants {
    // CAN ID — update to match your robot's CAN bus
    public static final int ROLLER_MOTOR_ID = 20; // SparkFlex, NEO Vortex

    // Current limit (amps) — 35A protects against hard game piece jams; still ample intake torque
    public static final int ROLLER_CURRENT_LIMIT = 35;

    // Roller duty-cycle output
    public static final double INTAKE_SPEED  =  0.80; // intake game piece
    public static final double OUTTAKE_SPEED = -0.60; // eject game piece
  }

  // Intake Pivot Constants
  public static class IntakePivotConstants {
    public static final int LEADER_MOTOR_ID   = 21; // SparkMax, NEO 550
    public static final int FOLLOWER_MOTOR_ID = 22; // SparkMax, NEO 550

    // NEO 550 safe peak current; limits current into the mechanical hard stop
    public static final int PIVOT_CURRENT_LIMIT = 20;

    // Duty-cycle output — positive rotates toward the bumper stop (down)
    // Tune DEPLOY_SPEED first; reduce if the pivot hits the bumper too hard
    public static final double PIVOT_DEPLOY_SPEED  =  0.30; // TUNE THIS
    public static final double PIVOT_RETRACT_SPEED = -0.40; // TUNE THIS

    // Follower is on the opposite side of the pivot point so it must be inverted
    public static final boolean FOLLOWER_INVERTED = true;
  }

  // Shooter Constants
  public static class ShooterConstants {
    // CAN IDs — update to match your robot's CAN bus
    public static final int FEED_MOTOR_ID        = 16;
    public static final int FLYWHEEL_MOTOR_ID    = 17;
    public static final int BACK_ROLLER_MOTOR_ID = 18;
    public static final int AGITATOR_MOTOR_ID    = 19; // SparkFlex, NEO Vortex

    // Current limits (amps) — sized to match 40A PDP/PDH breakers and protect motors.
    // Flywheel: 50A allows fast spin-up while staying below breaker trip threshold.
    //           Reduce to 40A if breakers trip during match play.
    public static final int FLYWHEEL_CURRENT_LIMIT    = 50;
    public static final int FEED_CURRENT_LIMIT         = 30;
    public static final int BACK_ROLLER_CURRENT_LIMIT  = 40;
    public static final int AGITATOR_CURRENT_LIMIT     = 30;

    // Agitator duty-cycle speed — runs open-loop whenever the feed is active
    // Positive = toward shooter; reduce if balls jam, increase if feed starves
    public static final double AGITATOR_SPEED = 0.6; // TUNE THIS

    // Closed-loop ramp rates (seconds to reach full output) — prevents voltage sag on spin-up
    public static final double FLYWHEEL_RAMP_RATE    = 0.25;
    public static final double FEED_RAMP_RATE         = 0.10;
    public static final double BACK_ROLLER_RAMP_RATE  = 0.25;

    // Flywheel & back roller PID — Neo Vortex starting values (tune on carpet)
    // kV = 1 / 6784 RPM (Neo Vortex free speed) — applied as duty-cycle per RPM
    public static final double FLYWHEEL_KP  = 0.0002, FLYWHEEL_KI  = 0.0, FLYWHEEL_KD  = 0.0, FLYWHEEL_KV  = 0.000152;
    public static final double BACK_ROLLER_KP = 0.0002, BACK_ROLLER_KI = 0.0, BACK_ROLLER_KD = 0.0, BACK_ROLLER_KV = 0.000152;

    // Feed motor PID — tune separately; feed roller has a different load than the flywheel
    public static final double FEED_KP  = 0.0002, FEED_KI  = 0.0, FEED_KD  = 0.0, FEED_KV  = 0.000152;

    // Back roller speed as a fraction of flywheel RPM.
    // 1.0 = matched speeds (no net spin, flattest trajectory)
    // < 1.0 = flywheel wins → backspin → Magnus effect curves ball upward (steeper arc)
    // > 1.0 = back roller wins → topspin → flatter arc
    // Tune in 0.05 steps. Start at 0.85 to compensate for rolling entry.
    public static final double BACK_ROLLER_SPEED_RATIO = 0.85;

    // Flywheel considered at speed within this many RPM of target
    public static final double FLYWHEEL_RPM_TOLERANCE = 50.0;

    // Default RPMs (used as fallback; range table takes over during normal operation)
    public static final double DEFAULT_FLYWHEEL_RPM = 4000.0;
    public static final double DEFAULT_FEED_RPM     = 1500.0;

    // Distance (feet from hub center) → Flywheel RPM
    // Keys are written in feet for easy measurement with a tape measure.
    // Units.feetToMeters() converts them to meters for the robot's internal distance calculations.
    // All RPM values are PLACEHOLDERS — measure actual shots on carpet and replace each entry.
    // Tune the 7 ft entry first (nearest to HUB_STANDOFF_DISTANCE), then work outward.
    public static final InterpolatingDoubleTreeMap RANGE_TABLE = new InterpolatingDoubleTreeMap();
    static {
      RANGE_TABLE.put(Units.feetToMeters(5.0),  2200.0);
      RANGE_TABLE.put(Units.feetToMeters(5.5),  2350.0);
      RANGE_TABLE.put(Units.feetToMeters(6.0),  2500.0);
      RANGE_TABLE.put(Units.feetToMeters(6.5),  2700.0);
      RANGE_TABLE.put(Units.feetToMeters(7.0),  2900.0);  // near HUB_STANDOFF_DISTANCE — tune this first
      RANGE_TABLE.put(Units.feetToMeters(7.5),  3100.0);
      RANGE_TABLE.put(Units.feetToMeters(8.0),  3300.0);
      RANGE_TABLE.put(Units.feetToMeters(8.5),  3550.0);
      RANGE_TABLE.put(Units.feetToMeters(9.0),  3750.0);
      RANGE_TABLE.put(Units.feetToMeters(9.5),  3900.0);
      RANGE_TABLE.put(Units.feetToMeters(10.0), 4050.0);
      RANGE_TABLE.put(Units.feetToMeters(10.5), 4200.0);
      RANGE_TABLE.put(Units.feetToMeters(11.0), 4350.0);
      RANGE_TABLE.put(Units.feetToMeters(11.5), 4600.0);
      RANGE_TABLE.put(Units.feetToMeters(12.0), 4800.0);
      RANGE_TABLE.put(Units.feetToMeters(13.0), 5000.0);
    }
  }

}
