package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.IntakeConstants;

public class IntakePivotSubsystem extends SubsystemBase {

    private final SparkMax pivotLeader;
    private final SparkMax pivotFollower;

    public IntakePivotSubsystem() {
        pivotLeader   = new SparkMax(IntakeConstants.PIVOT_LEADER_MOTOR_ID,   MotorType.kBrushless);
        pivotFollower = new SparkMax(IntakeConstants.PIVOT_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.inverted(false);
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT);
        leaderConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(20)
            .motorTemperaturePeriodMs(500);
        pivotLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT);
        followerConfig.follow(IntakeConstants.PIVOT_LEADER_MOTOR_ID, IntakeConstants.PIVOT_FOLLOWER_INVERTED);
        followerConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(500)
            .motorTemperaturePeriodMs(500);
        pivotFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void deployPivot()  { pivotLeader.set(IntakeConstants.PIVOT_DEPLOY_SPEED); }
    public void retractPivot() { pivotLeader.set(IntakeConstants.PIVOT_RETRACT_SPEED); }
    public void stopPivot()    { pivotLeader.stopMotor(); }

    /**
     * Drive the pivot down toward the bumper stop while held; coast to rest on release.
     * Typical binding: operatorXbox.leftBumper().whileTrue(pivot.deployPivotCommand())
     */
    public Command deployPivotCommand() {
        return startEnd(this::deployPivot, this::stopPivot).withName("Intake/DeployPivot");
    }

    /**
     * Drive the pivot up to the retracted position while held; stop on release.
     * Typical binding: operatorXbox.rightBumper().whileTrue(pivot.retractPivotCommand())
     */
    public Command retractPivotCommand() {
        return startEnd(this::retractPivot, this::stopPivot).withName("Intake/RetractPivot");
    }

    /** Retract slightly then re-deploy to free a jammed game piece. */
    public Command unjamCommand() {
        return startEnd(this::retractPivot, this::stopPivot)
            .withTimeout(0.2)
            .andThen(startEnd(this::deployPivot, this::stopPivot).withTimeout(0.2))
            .withName("Intake/Unjam");
    }

    /** Deploy the pivot for 0.5 seconds then stop — for use in autonomous. */
    public Command deployPivotAutoCommand() {
        return deployPivotCommand().withTimeout(0.5).withName("Intake/DeployPivotAuto");
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Pivot/LeaderOutput",   pivotLeader.getAppliedOutput());
        Logger.recordOutput("Intake/Pivot/LeaderAmps",     pivotLeader.getOutputCurrent());
        Logger.recordOutput("Intake/Pivot/FollowerAmps",   pivotFollower.getOutputCurrent());
        Logger.recordOutput("Intake/Pivot/LeaderFaults",   pivotLeader.getFaults().rawBits);
        Logger.recordOutput("Intake/Pivot/FollowerFaults", pivotFollower.getFaults().rawBits);
    }
}
