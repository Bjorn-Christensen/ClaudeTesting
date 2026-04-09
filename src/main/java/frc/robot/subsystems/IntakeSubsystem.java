package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.IntakeConstants;

/**
 * Controls the intake roller and pivot mechanisms.
 *
 * Roller hardware: NEO Vortex on SparkFlex — open-loop duty-cycle.
 * Pivot hardware:  two NEO 550s on SparkMax controllers with a 10:1 gear reduction.
 *                  Leader + hardware follower. Open-loop duty-cycle, coast idle mode.
 *
 * The pivot has two states — deployed (resting on robot bumpers) and retracted.
 * Coast mode is used so the motors spin free when the intake sits on the bumper stop,
 * avoiding heat build-up from fighting the hard stop.
 */
public class IntakeSubsystem extends SubsystemBase {

    // ---- Roller ----
    private final SparkFlex rollerMotor;

    // ---- Pivot ----
    private final SparkMax pivotLeader;
    private final SparkMax pivotFollower;

    public IntakeSubsystem() {

        // --- Roller ---
        rollerMotor = new SparkFlex(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.inverted(false);
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Pivot leader ---
        pivotLeader   = new SparkMax(IntakeConstants.PIVOT_LEADER_MOTOR_ID,   MotorType.kBrushless);
        pivotFollower = new SparkMax(IntakeConstants.PIVOT_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.inverted(false);
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT);
        pivotLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Pivot follower (mirrors leader at firmware level) ---
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT);
        followerConfig.follow(IntakeConstants.PIVOT_LEADER_MOTOR_ID, IntakeConstants.PIVOT_FOLLOWER_INVERTED);
        pivotFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ---- Roller low-level control ----

    public void runIntake()     { rollerMotor.set(IntakeConstants.INTAKE_SPEED); }
    public void reverseIntake() { rollerMotor.set(IntakeConstants.OUTTAKE_SPEED); }
    public void stopIntake()    { rollerMotor.stopMotor(); }

    // ---- Pivot low-level control ----

    public void deployPivot()  { pivotLeader.set(IntakeConstants.PIVOT_DEPLOY_SPEED); }
    public void retractPivot() { pivotLeader.set(IntakeConstants.PIVOT_RETRACT_SPEED); }
    public void stopPivot()    { pivotLeader.stopMotor(); }

    // ---- Roller commands ----

    /** Run intake rollers inward while held; stop on release. */
    public Command intakeCommand() {
        return runEnd(this::runIntake, this::stopIntake);
    }

    /** Reverse intake rollers (eject) while held; stop on release. */
    public Command ejectCommand() {
        return runEnd(this::reverseIntake, this::stopIntake);
    }

    /** Stop intake rollers immediately (one-shot, for use in autos). */
    public Command stopCommand() {
        return runOnce(this::stopIntake);
    }

    // ---- Pivot commands ----

    /**
     * Drive the pivot down toward the bumper stop while held; coast to rest on release.
     * The intake stays on the bumper without motor power — no need to keep holding.
     * Typical binding: operatorXbox.rightTrigger().whileTrue(intake.deployPivotCommand())
     */
    public Command deployPivotCommand() {
        return runEnd(this::deployPivot, this::stopPivot).withName("Intake/DeployPivot");
    }

    /**
     * Drive the pivot up to the retracted position while held; stop on release.
     * Hold until the pivot is fully retracted.
     * Typical binding: operatorXbox.b().whileTrue(intake.retractPivotCommand())
     */
    public Command retractPivotCommand() {
        return runEnd(this::retractPivot, this::stopPivot).withName("Intake/RetractPivot");
    }

    // ---- Periodic ----

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Roller/Output",        rollerMotor.getAppliedOutput());
        Logger.recordOutput("Intake/Pivot/LeaderOutput",   pivotLeader.getAppliedOutput());
        Logger.recordOutput("Intake/Pivot/LeaderAmps",     pivotLeader.getOutputCurrent());
        Logger.recordOutput("Intake/Pivot/FollowerAmps",   pivotFollower.getOutputCurrent());
        Logger.recordOutput("Intake/Pivot/LeaderFaults",   pivotLeader.getFaults().rawBits);
        Logger.recordOutput("Intake/Pivot/FollowerFaults", pivotFollower.getFaults().rawBits);
    }
}
