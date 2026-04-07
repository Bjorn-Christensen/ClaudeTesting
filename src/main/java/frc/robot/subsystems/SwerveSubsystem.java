package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    private Vision vision;

    // Precision scoring PID controller
    private final HolonomicDriveController precisionHDC;
    private static final double POS_TOL_M              = 0.05;                // tolerance in meters
    private static final double ANG_TOL_RAD            = Math.toRadians(2.0); // tolerance in radians
    private static final double FACING_GOAL_TOLERANCE_DEG = 5.0;

    public SwerveSubsystem(File directory) {

        // Configure telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DrivetrainConstants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.setHeadingCorrection(true); // Holds heading during pure translation; update desired heading while rotating.

        // Enable vision tracking and path planner
        if (VisionConstants.CAMERAS_ENABLED) {
            setupPhotonVision();
            swerveDrive.stopOdometryThread(); // Stop the odometry thread if we are using vision that way we can synchronize updates better.
        }
        setupPathPlanner();

        // Initialize precision movement controller
        precisionHDC = new HolonomicDriveController(
            new PIDController(2.5, 0.0, 0.2),   // X PID (tune on carpet)
            new PIDController(2.5, 0.0, 0.2),   // Y PID
            new ProfiledPIDController(3.0, 0.0, 0.2, // Theta PID (profiled)
                new TrapezoidProfile.Constraints(
                    swerveDrive.getMaximumChassisAngularVelocity(),
                    2.0 * swerveDrive.getMaximumChassisAngularVelocity())));
    }

    // ---- Drive ----

    /** Drive the robot given a chassis field-oriented velocity. */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    // ---- Vision ----

    /** Create the PhotonVision subsystem and attach it to the swerve drive pose estimator. */
    public void setupPhotonVision() {
        vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    }

    // ---- Periodic ----

    @Override
    public void periodic() {
        if (VisionConstants.CAMERAS_ENABLED) {
            swerveDrive.updateOdometry();
            vision.updatePoseEstimation(swerveDrive);
        }

        // Authoritative robot pose for AdvantageScope
        Logger.recordOutput("Robot/Pose",              swerveDrive.getPose());
        Logger.recordOutput("Robot/DistanceToHub",     getDistanceToHub());
        Logger.recordOutput("Robot/DistanceToHubFeet", Units.metersToFeet(getDistanceToHub()));
    }

    @Override
    public void simulationPeriodic() {
        // Update simulated camera poses outside the main loop to avoid overruns
        if (VisionConstants.CAMERAS_ENABLED)
            swerveDrive.getSimulationDriveTrainPose().ifPresent(vision::updateSim);
    }

    // ---- State ----

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    // ---- PathPlanner / Autonomous Control ----

    /** Create a path following command using AutoBuilder. This will also trigger event markers. */
    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /** Set up PathPlanner AutoBuilder; taken directly from YAGSL — only tune PPHolonomicController PIDs. */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings.
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                (speedsRobotRelative, moduleFeedForwards) -> {
                    swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces());
                },
                // PPHolonomicController is the built-in path following controller for holonomic drive trains
                new PPHolonomicDriveController(
                    new PIDConstants(10.0, 0.0, 4.0), // Translation PID constants
                    new PIDConstants(3.0, 0.0, 0.2)   // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance.
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Preload PathPlanner path finding
        PathfindingCommand.warmupCommand();
    }

    /** Set odometry to the auto start pose and suppress stale vision frames. */
    public void primeStartingPose(Pose2d start) {
        // Put odometry at the auto start pose before the match starts
        resetOdometry(start);

        // Tell vision to align
        if (VisionConstants.CAMERAS_ENABLED && SwerveDriveTelemetry.isSimulation) {
            vision.pauseVisionFor(0.35); // ~ camera latency + buffer
        }
    }

    // ---- Hub Targeting ----

    /** Returns the straight-line distance in meters from the robot to the hub center. */
    public double getDistanceToHub() {
        var alliance = DriverStation.getAlliance();
        Pose2d hubPose = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue)
            ? FieldConstants.HUB_POSE_BLUE : FieldConstants.HUB_POSE_RED;
        return getPose().getTranslation().getDistance(hubPose.getTranslation());
    }

    /** Returns true when the robot's heading is within FACING_GOAL_TOLERANCE_DEG of the hub. */
    public boolean isFacingGoal() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        Pose2d hubPose = (alliance.get() == DriverStation.Alliance.Blue)
            ? FieldConstants.HUB_POSE_BLUE : FieldConstants.HUB_POSE_RED;

        Translation2d toHub = hubPose.getTranslation().minus(getPose().getTranslation());
        Rotation2d required = toHub.getAngle();
        return Math.abs(getPose().getRotation().minus(required).getDegrees()) < FACING_GOAL_TOLERANCE_DEG;
    }

    /**
     * Navigate to the ideal shooting position (HUB_STANDOFF_DISTANCE from hub center) and rotate
     * to face the hub. Uses PathPlanner pathfinding followed by precision PID line-up.
     */
    public Command lineUpForShooter() {

        Pose2d hubPose;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            hubPose = FieldConstants.HUB_POSE_BLUE;
        } else if (alliance.isPresent()) {
            hubPose = FieldConstants.HUB_POSE_RED;
        } else {
            return Commands.print("No Alliance Selected: Can't Shoot");
        }

        Translation2d curTranslation = swerveDrive.getPose().getTranslation();
        Translation2d hubTranslation = hubPose.getTranslation();
        Translation2d vecHubToRobot  = curTranslation.minus(hubTranslation); // Vector from hub → robot
        double magnitude = vecHubToRobot.getNorm();
        if (magnitude < 0.01) {
            return Commands.print("Robot too close to hub center: can't compute approach direction");
        }
        Translation2d norm = vecHubToRobot.div(magnitude);

        // Constrain to alliance's side of the hub.
        // If the robot is on the wrong side, snap to the nearest valid boundary (±90°, Y-axis).
        boolean onWrongSide = (alliance.get() == DriverStation.Alliance.Blue) ? norm.getX() > 0 : norm.getX() < 0;
        if (onWrongSide) {
            double clampedAngle = norm.getY() >= 0 ? Math.PI / 2 : -Math.PI / 2;
            norm = new Translation2d(Math.cos(clampedAngle), Math.sin(clampedAngle));
        }

        Translation2d targetTranslation = hubTranslation.plus(norm.times(FieldConstants.HUB_STANDOFF_DISTANCE));
        Rotation2d targetRotation = norm.getAngle().rotateBy(Rotation2d.fromDegrees(180));
        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(), 4.0,
            swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            // Goal end velocity in meters/sec
            edu.wpi.first.units.Units.MetersPerSecond.of(0))
            .andThen(precisionLineUp(targetPose));
    }

    /** PID-based final approach to a target pose; locks the robot when at reference. */
    public Command precisionLineUp(Pose2d targetPose) {

        precisionHDC.setTolerance(new Pose2d(POS_TOL_M, POS_TOL_M, new Rotation2d(ANG_TOL_RAD)));
        precisionHDC.setEnabled(true);

        // Sync the profiled theta controller to the robot's actual heading before the loop starts.
        // Without this, the ProfiledPIDController uses a stale internal state on the first
        // calculate() call, causing a large rotation spike before the profile settles.
        return runOnce(() -> precisionHDC.getThetaController().reset(
                    swerveDrive.getPose().getRotation().getRadians(), 0.0))
            .andThen(runEnd(
                () -> {
                    Pose2d cur = swerveDrive.getPose();
                    ChassisSpeeds commanded =
                        precisionHDC.calculate(cur, targetPose, 0.0, targetPose.getRotation());
                    double vx    = MathUtil.clamp(commanded.vxMetersPerSecond,       -1.0, 1.0);
                    double vy    = MathUtil.clamp(commanded.vyMetersPerSecond,       -1.0, 1.0);
                    double omega = MathUtil.clamp(commanded.omegaRadiansPerSecond,
                                                 -Units.degreesToRadians(360), Units.degreesToRadians(360));
                    swerveDrive.driveFieldOriented(
                        ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, cur.getRotation()));
                },
                () -> swerveDrive.lockPose())
            .until(precisionHDC::atReference)
            .withTimeout(1.0));
    }
}
