package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.CameraConfig;

// Vision subsystem using PhotonVision that plugs into swervelib's pose estimator
public class Vision extends SubsystemBase {

  // ====== Context from the drive (for telemetry only) ======
  private final Supplier<Pose2d> currentPoseSupplier; // used in sendable
  private final Field2d field; // for visualization

  // One entry per camera.
  private final Map<String, Cam> cams = new LinkedHashMap<>();

  // --- Simulation-only members ---
  private VisionSystemSim visionSim; // targets + cameras

  // Used to disable vision for very beginning of match and limit update freq
  private double visionPauseUntil = 0.0;

  public Vision(Supplier<Pose2d> currentPoseSupplier, Field2d field) {
    this.currentPoseSupplier = currentPoseSupplier;
    this.field = field;

    // Build a camera + estimator for each configured camera in Constants.Vision.CAMERAS
    for (CameraConfig cfg : VisionConstants.CAMERAS) {
      PhotonCamera camera = new PhotonCamera(cfg.name);

      PhotonPoseEstimator estimator = new PhotonPoseEstimator(
          Constants.APRIL_TAG_FIELD_LAYOUT,
          cfg.robotToCam
      );

      cams.put(cfg.name, new Cam(camera, estimator));
    }

    // Simulation Initialization
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(Constants.APRIL_TAG_FIELD_LAYOUT);

      for (CameraConfig cfg : VisionConstants.CAMERAS) {
        Cam cam = cams.get(cfg.name);

        // Camera properties (FOV, res, FPS)
        cam.camProps = new SimCameraProperties();
        cam.camProps.setCalibration(640, 480,             // width, height
                                    Rotation2d.fromDegrees(100.0));  // diagonal FOV approx
        cam.camProps.setFPS(30);
        cam.camProps.setAvgLatencyMs(35);
        cam.camProps.setLatencyStdDevMs(5);

        cam.cameraSim = new PhotonCameraSim(cam.camera, cam.camProps);
        cam.cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cam.cameraSim, cfg.robotToCam);
      }
    }
  }

  @Override
  public void periodic() {

    for (var entry : cams.entrySet()) {
      String name = entry.getKey();
      Cam cam = entry.getValue();

      // Get latest frame and estimate robot pose
      List<PhotonPipelineResult> frames = cam.camera.getAllUnreadResults();
      if (frames.isEmpty()) {
        continue;
      }

      for (PhotonPipelineResult frame : frames) {
        Optional<EstimatedRobotPose> estimate =  cam.estimator.estimateCoprocMultiTagPose(frame);
        if (estimate.isEmpty()) {
          // Fallback to lowest ambiguity if multi‑tag fails
          estimate = cam.estimator.estimateLowestAmbiguityPose(frame);
        }

        if (estimate.isPresent()) {
          EstimatedRobotPose est = estimate.get();

          // Visualize
          if (field != null) {
            field.getObject("Vision/" + name + "/Estimate")
                  .setPose(est.estimatedPose.toPose2d());

            var seen = new ArrayList<Pose2d>();
            if (frame.hasTargets()) {
              for (PhotonTrackedTarget t : frame.getTargets()) {
                try {
                  Optional<Pose3d> tagPose =
                      Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(t.getFiducialId());
                  tagPose.ifPresent(p3 -> seen.add(p3.toPose2d()));
                } catch (Exception ignored) {}
              }
            }
            field.getObject("Vision/" + name + "/SeenTags").setPoses(seen);
          }

          // Store so updatePoseEstimation() can feed it to the swerve drive
          cam.lastEstimate = estimate;

          // Only handle first valid estimate per set of frames
          break;
        }
      }
    }
  }

  // Pushes all valid camera estimates into the pose estimator.
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // Causes loop overrun but only used in sim to maintain camera-drivetrain pose connection
    if (now < visionPauseUntil) return;   // Short quiet period after any reset

    for (Cam cam : cams.values()) {
      if (cam.lastEstimate.isEmpty()) continue;
      EstimatedRobotPose est = cam.lastEstimate.get();
      if (est.timestampSeconds < visionPauseUntil) continue; // Drop pre-reset frames
      Matrix<N3, N1> stdDevs =
          (est.targetsUsed.size() >= 2) ? VisionConstants.MULTI_TAG_STD_DEVS : VisionConstants.SINGLE_TAG_STD_DEVS;

      swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
    }
  }

  // =================== Public helpers ===================

  // Nearest AprilTag Pose2d from a list
  public Pose2d nearestTagFromList(int[] tagIds, SwerveDrive swerveDrive) {
    Pose2d ref = currentPoseSupplier.get();
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      ref = swerveDrive.getSimulationDriveTrainPose().get();
    }
    Pose2d nearest = null;
    double minMeters = Double.MAX_VALUE;

    for (int id : tagIds) {
      // Simple sanity check that april tag list was correctly built in Constants file
      try {
        Optional<Pose3d> pose3d = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(id);
        if (pose3d.isEmpty()) continue;

        Pose2d tag2d = pose3d.get().toPose2d();
        double d = ref.getTranslation().getDistance(tag2d.getTranslation());
        if (d < minMeters) {
          minMeters = d;
          nearest = tag2d;
        }
      } catch (Exception ignored) {}
    }
    return nearest;
  }

  // Called from SwerveSubsystem.simulationPeriodic() to avoid loop overruns in periodic()
  public void updateSim(Pose2d simPose) {
    if (visionSim != null) {
      visionSim.update(simPose);
    }
  }

  // Mostly used for start of simulation to ignore robot pose teleportation
  public void pauseVisionFor(double seconds) {
    visionPauseUntil = Timer.getFPGATimestamp() + seconds;
  }

  // =================== Internal per-camera structure ===================
  private static class Cam {
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;

    // --- Simulation-only per-camera ---
    PhotonCameraSim cameraSim;
    SimCameraProperties camProps;

    Optional<EstimatedRobotPose> lastEstimate = Optional.empty();

    Cam(PhotonCamera camera, PhotonPoseEstimator estimator) {
      this.camera = camera;
      this.estimator = estimator;
    }
  }
}
