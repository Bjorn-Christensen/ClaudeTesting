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
    // CAN IDs — update to match your robot's CAN bus
    public static final int ROLLER_MOTOR_ID         = 20; // SparkFlex, NEO Vortex
    public static final int PIVOT_LEADER_MOTOR_ID   = 21; // SparkMax, NEO 550
    public static final int PIVOT_FOLLOWER_MOTOR_ID = 22; // SparkMax, NEO 550

    // Current limits (amps)
    // Roller: 35A protects against hard game piece jams; still ample intake torque
    // Pivot:  NEO 550 safe peak current; limits current into the mechanical hard stop
    public static final int ROLLER_CURRENT_LIMIT = 35;
    public static final int PIVOT_CURRENT_LIMIT  = 15;

    // Roller duty-cycle output
    public static final double INTAKE_SPEED  =  0.50; // intake game piece
    public static final double OUTTAKE_SPEED = -0.50; // eject game piece

    // Pivot duty-cycle output — positive rotates toward the bumper stop (down)
    // Tune DEPLOY_SPEED first; reduce if the pivot hits the bumper too hard
    public static final double PIVOT_DEPLOY_SPEED  =  0.30; // TUNE THIS
    public static final double PIVOT_RETRACT_SPEED = -0.40; // TUNE THIS

    // Pivot follower is on the opposite side of the pivot point so it must be inverted
    public static final boolean PIVOT_FOLLOWER_INVERTED = true;
  }

  // Shooter Constants
  public static class ShooterConstants {
    // CAN IDs — update to match your robot's CAN bus
    public static final int FLYWHEEL_MOTOR_ID          = 16;
    public static final int FEED_MOTOR_ID              = 17;
    public static final int FEED_FOLLOWER_MOTOR_ID     = 14;
    public static final int BACK_ROLLER_MOTOR_ID       = 18; // independent back roller (formerly feed follower)
    public static final int AGITATOR_MOTOR_ID          = 19; // SparkFlex, NEO Vortex
    public static final int AGITATOR_FOLLOWER_MOTOR_ID = 15; // follows AGITATOR_MOTOR_ID, inverted

    // Current limits (amps)
    // Back roller is the heavy/high-inertia roller and gets the higher limit for spin-up torque.
    // Flywheel is a lightweight front roller — lower limit is sufficient.
    public static final int FLYWHEEL_CURRENT_LIMIT     = 25;
    public static final int FEED_CURRENT_LIMIT         = 30;
    public static final int BACK_ROLLER_CURRENT_LIMIT  = 40;
    public static final int AGITATOR_CURRENT_LIMIT     = 30;

    // Agitator duty-cycle speed — runs open-loop whenever the feed is active
    // Positive = toward shooter; reduce if balls jam, increase if feed starves
    public static final double AGITATOR_SPEED = 0.3;

    // Nominal battery voltage used for voltage compensation on closed-loop motors.
    // Set to the minimum expected voltage under full shooter load (~11 V), not the resting voltage.
    // The Spark can only scale output downward — setting this above actual available voltage clips at 100%.
    public static final double VOLTAGE_COMPENSATION = 11.0;

    // Closed-loop ramp rates (seconds to reach full output) — prevents voltage sag on spin-up
    // Back roller is heavier (more inertia) so it ramps slower to avoid current spikes.
    // Flywheel is light and can ramp quickly.
    public static final double FLYWHEEL_RAMP_RATE     = 0.15;
    public static final double FEED_RAMP_RATE         = 0.25;
    public static final double BACK_ROLLER_RAMP_RATE  = 0.50;

    // Flywheel PID
    // kV = 1 / 6784 RPM (Neo Vortex free speed) — applied as duty-cycle per RPM
    // kP corrects speed droop when a ball passes through; increase if recovery between shots is slow,
    // decrease if the motor oscillates (hunts) around the setpoint.
    public static final double FLYWHEEL_KP = 0.0002, FLYWHEEL_KI = 0.0, FLYWHEEL_KD = 0.0, FLYWHEEL_KV = 0.000147;

    // Feed motor PID
    public static final double FEED_KP = 0.0002, FEED_KI = 0.0, FEED_KD = 0.0, FEED_KV = 0.000147;

    // Back roller PID — same kP reasoning as flywheel
    public static final double BACK_ROLLER_KP = 0.0002, BACK_ROLLER_KI = 0.0, BACK_ROLLER_KD = 0.0, BACK_ROLLER_KV = 0.000147;

    // At-speed tolerances (RPM)
    public static final double FLYWHEEL_RPM_TOLERANCE     = 50.0;
    public static final double FEED_RPM_TOLERANCE         = 50.0;
    public static final double BACK_ROLLER_RPM_TOLERANCE  = 50.0;

    // Default RPMs (used as fallback; range table takes over during normal operation)
    public static final double DEFAULT_FLYWHEEL_RPM     = 2500.0;
    public static final double DEFAULT_FEED_RPM         = 1500.0;
    public static final double DEFAULT_BACK_ROLLER_RPM  = 2500.0;

    // How long to keep feeding balls during an auto shoot sequence
    public static final double SHOOT_DURATION_SECONDS = 2.0;

  }

}
