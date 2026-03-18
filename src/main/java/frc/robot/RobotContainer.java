package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;

public class RobotContainer {

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(DrivetrainConstants.swerveJsonDirectory);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Joysticks
  final CommandXboxController controllerXbox    = new CommandXboxController(0); // driver
  final CommandXboxController operatorXbox      = new CommandXboxController(1); // operator

  // Auton Chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  // Subsystem requirements required for commands writting direct within subsystem class
  private final Set<Subsystem> swerveReq = Set.of(swerveSubsystem);

  // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> controllerXbox.getLeftY() * -1,
                                                                () -> controllerXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> controllerXbox.getRightX() * -1)
                                                            .deadband(ControllerConstants.DEADZONE)
                                                            .scaleTranslation(0.8)
                                                            .scaleRotation(0.8)
                                                            .allianceRelativeControl(true);

  // Constructor
  public RobotContainer() {
    configureButtonBindings();

    // Register named commands for PathPlanner
    NamedCommands.registerCommand("lineUpShot",
        Commands.defer(() -> swerveSubsystem.lineUpForShooter(), Set.of(swerveSubsystem)));
    NamedCommands.registerCommand("runIntake",  intakeSubsystem.intakeCommand());
    NamedCommands.registerCommand("stopIntake", intakeSubsystem.stopCommand());

    // Shoot for a fixed duration using the range table and isFacingGoal as the ready gate.
    // Adjust the timeout seconds to match how long you want the robot to feed balls.
    final double SHOOT_DURATION_SECONDS = 2.0;
    NamedCommands.registerCommand("shoot",
        shooterSubsystem.shootOnReadyCommand(
            () -> ShooterConstants.RANGE_TABLE.get(swerveSubsystem.getDistanceToHub()),
            ShooterConstants.DEFAULT_FEED_RPM,
            swerveSubsystem::isFacingGoal
        ).withTimeout(SHOOT_DURATION_SECONDS));

    // Build chooser after NamedCommands are registered so that event markers have something to call
    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser("Test"));
  }

  private void configureButtonBindings() {

    // ---- Driver controller (port 0) — driving only ----

    // Auto line-up to shooting position
    controllerXbox.povUp().onTrue(Commands.defer(() -> swerveSubsystem.lineUpForShooter(), swerveReq));

    // Reset odometry to (0,0) — useful during testing/practice, never press mid-match
    controllerXbox.povLeft().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d()), swerveSubsystem));

    // ---- Operator controller (port 1) — shooter & intake ----

    // Hold left bumper to test-shoot — range-adjusted RPM, no facing-goal check.
    // Use when cameras are off or during bench/distance testing.
    operatorXbox.leftBumper().whileTrue(
        shooterSubsystem.shootOnReadyCommand(
            () -> ShooterConstants.RANGE_TABLE.get(swerveSubsystem.getDistanceToHub()),
            ShooterConstants.DEFAULT_FEED_RPM,
            () -> true
        )
    );

    // Hold right bumper to shoot — range-adjusted RPM, requires facing the goal.
    operatorXbox.rightBumper().whileTrue(
        shooterSubsystem.shootOnReadyCommand(
            () -> ShooterConstants.RANGE_TABLE.get(swerveSubsystem.getDistanceToHub()),
            ShooterConstants.DEFAULT_FEED_RPM,
            swerveSubsystem::isFacingGoal
        )
    );

    // Hold left trigger to run intake rollers inward
    operatorXbox.leftTrigger(ControllerConstants.LEFT_TRIGGER_DEADZONE)
        .whileTrue(intakeSubsystem.intakeCommand());

    // Hold X to eject / reverse intake rollers
    operatorXbox.x().whileTrue(intakeSubsystem.ejectCommand());

  }

  // Auton chooser called on Autonomous Init
  public Command getAutonomousCommand() {
    Command selected = autoChooser.get();
    if(selected != null) {
      String autoName = selected.getName();
      try {
        var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        if (!paths.isEmpty()) {
          Pose2d startPose = paths.get(0).getPathPoses().get(0);
          swerveSubsystem.primeStartingPose(startPose);
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    return selected;
  }

  // Set drive Controls For Teleop
  public void setSwerveDefaultCommand() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
  }

  // Set drive motor brakes
  public void setDriveMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
    if(!brake) swerveSubsystem.getSwerveDrive().lockPose();
  }

}
