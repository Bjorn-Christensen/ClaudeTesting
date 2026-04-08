package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex feedMotor;
    private final SparkFlex feedFollowerMotor;
    private final SparkFlex flywheelMotor;
    private final SparkFlex agitatorMotor;
    private final SparkFlex agitatorFollowerMotor;

    private final SparkClosedLoopController feedController;
    private final SparkClosedLoopController flywheelController;

    private double targetFlywheelRpm = 0.0;
    private double targetFeedRpm     = 0.0;

    // Tracks whether the feed is currently active so we don't thrash motor
    // commands every loop iteration when gating on flywheel speed.
    private boolean feedActive = false;

    private static final TunableNumber kAgitatorSpeed =
        new TunableNumber("Tuning/Shooter/AgitatorSpeed",     ShooterConstants.AGITATOR_SPEED);
    private static final TunableNumber kFlywheelTolerance =
        new TunableNumber("Tuning/Shooter/FlywheelTolerance", ShooterConstants.FLYWHEEL_RPM_TOLERANCE);

    public ShooterSubsystem() {
        feedMotor             = new SparkFlex(ShooterConstants.FEED_LEADER_MOTOR_ID,       MotorType.kBrushless);
        feedFollowerMotor     = new SparkFlex(ShooterConstants.FEED_FOLLOWER_MOTOR_ID,     MotorType.kBrushless);
        flywheelMotor         = new SparkFlex(ShooterConstants.FLYWHEEL_MOTOR_ID,          MotorType.kBrushless);
        agitatorMotor         = new SparkFlex(ShooterConstants.AGITATOR_MOTOR_ID,          MotorType.kBrushless);
        agitatorFollowerMotor = new SparkFlex(ShooterConstants.AGITATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        SparkFlexConfig feedConfig = new SparkFlexConfig();
        feedConfig.inverted(false);
        feedConfig.idleMode(IdleMode.kCoast);
        feedConfig.smartCurrentLimit(ShooterConstants.FEED_CURRENT_LIMIT);
        feedConfig.voltageCompensation(10.0);
        feedConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.FEED_KP)
            .i(ShooterConstants.FEED_KI)
            .d(ShooterConstants.FEED_KD);
        feedConfig.closedLoop.feedForward.kV(ShooterConstants.FEED_KV);
        feedConfig.closedLoopRampRate(ShooterConstants.FEED_RAMP_RATE);
        feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig feedFollowerConfig = new SparkFlexConfig();
        feedFollowerConfig.follow(ShooterConstants.FEED_LEADER_MOTOR_ID, false);
        feedFollowerConfig.idleMode(IdleMode.kCoast);
        feedFollowerConfig.smartCurrentLimit(ShooterConstants.FEED_CURRENT_LIMIT);
        feedFollowerMotor.configure(feedFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        flywheelConfig.inverted(true);
        flywheelConfig.idleMode(IdleMode.kCoast);
        flywheelConfig.smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT);
        flywheelConfig.voltageCompensation(10.0);
        flywheelConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.FLYWHEEL_KP)
            .i(ShooterConstants.FLYWHEEL_KI)
            .d(ShooterConstants.FLYWHEEL_KD);
        flywheelConfig.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_KV);
        flywheelConfig.closedLoopRampRate(ShooterConstants.FLYWHEEL_RAMP_RATE);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig agitatorConfig = new SparkFlexConfig();
        agitatorConfig.inverted(false);
        agitatorConfig.idleMode(IdleMode.kCoast);
        agitatorConfig.smartCurrentLimit(ShooterConstants.AGITATOR_CURRENT_LIMIT);
        agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig agitatorFollowerConfig = new SparkFlexConfig();
        agitatorFollowerConfig.follow(ShooterConstants.AGITATOR_MOTOR_ID, true);
        agitatorFollowerConfig.idleMode(IdleMode.kCoast);
        agitatorFollowerConfig.smartCurrentLimit(ShooterConstants.AGITATOR_CURRENT_LIMIT);
        agitatorFollowerMotor.configure(agitatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feedController     = feedMotor.getClosedLoopController();
        flywheelController = flywheelMotor.getClosedLoopController();
    }

    // ---- State setters ----

    public void setFlywheelRpm(double rpm) {
        targetFlywheelRpm = rpm;
        flywheelController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Start the feed at the given RPM. Only sends motor commands if the feed
     * isn't already running at that setpoint, preventing 50Hz command thrashing.
     */
    public void startFeed(double rpm) {
        if (!feedActive || targetFeedRpm != rpm) {
            targetFeedRpm = rpm;
            feedController.setSetpoint(rpm, ControlType.kVelocity);
            agitatorMotor.set(kAgitatorSpeed.get());
            feedActive = true;
        }
    }

    /**
     * Stop the feed and agitator. Only sends motor commands if the feed is
     * currently active, preventing redundant stop calls every loop iteration.
     */
    public void stopFeed() {
        if (feedActive) {
            targetFeedRpm = 0.0;
            feedMotor.stopMotor();
            agitatorMotor.stopMotor();
            feedActive = false;
        }
    }

    public void stopAll() {
        targetFlywheelRpm = 0.0;
        flywheelMotor.stopMotor();
        stopFeed();
    }

    // ---- Queries ----

    /**
     * Returns true when the flywheel is within tolerance of its target RPM.
     * Only checks the low side — overshoot during spin-up does not block feeding.
     */
    public boolean flywheelAtSpeed() {
        double actual = flywheelMotor.getEncoder().getVelocity();
        return targetFlywheelRpm > 0
            && actual >= (targetFlywheelRpm - kFlywheelTolerance.get());
    }

    // ---- Commands ----

    public Command spinUpCommand(double rpm) {
        return runEnd(
            () -> setFlywheelRpm(rpm),
            () -> {
                flywheelMotor.stopMotor();
                targetFlywheelRpm = 0.0;
            }
        );
    }

    public Command feedCommand(double rpm) {
        return runEnd(
            () -> startFeed(rpm),
            this::stopFeed
        );
    }

    /**
     * Hold to shoot. Spins up the flywheel immediately; only starts the feed
     * once the flywheel is at speed AND robotReady is true. The feed is held
     * cleanly off during any RPM dip rather than being toggled every loop tick,
     * which eliminates the revving sound from constant start/stop commands.
     */
    public Command shootOnReadyCommand(DoubleSupplier flywheelRpm, double feedRpm, BooleanSupplier robotReady) {
        return runEnd(
            () -> {
                setFlywheelRpm(flywheelRpm.getAsDouble());
                if (flywheelAtSpeed() && robotReady.getAsBoolean()) {
                    startFeed(feedRpm);
                } else {
                    stopFeed();
                }
            },
            this::stopAll
        );
    }

    public Command shootOnReadyCommand(double flywheelRpm, double feedRpm, BooleanSupplier robotReady) {
        return shootOnReadyCommand(() -> flywheelRpm, feedRpm, robotReady);
    }

    public Command stopCommand() {
        return runOnce(this::stopAll);
    }

    // ---- Periodic ----

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/Flywheel/TargetRPM",       targetFlywheelRpm);
        Logger.recordOutput("Shooter/Flywheel/ActualRPM",       flywheelMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Flywheel/CurrentAmps",     flywheelMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Flywheel/Faults",          flywheelMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/Feed/TargetRPM",           targetFeedRpm);
        Logger.recordOutput("Shooter/Feed/ActualRPM",           feedMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Feed/Active",              feedActive);
        Logger.recordOutput("Shooter/Feed/CurrentAmps",         feedMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Feed/Faults",              feedMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/FeedFollower/ActualRPM",   feedFollowerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FeedFollower/CurrentAmps", feedFollowerMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/FeedFollower/Faults",      feedFollowerMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/Agitator/Output",                  agitatorMotor.getAppliedOutput());
        Logger.recordOutput("Shooter/Agitator/CurrentAmps",             agitatorMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Agitator/Faults",                  agitatorMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/AgitatorFollower/CurrentAmps",     agitatorFollowerMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/AgitatorFollower/Faults",          agitatorFollowerMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/FlywheelAtSpeed",          flywheelAtSpeed());
    }
}