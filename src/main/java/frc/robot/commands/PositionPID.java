package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import java.util.Set;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PositionPID extends Command {

    public SwerveSubsystem mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = SwerveSubsystem.CONTROLLER;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault()
            .getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public static final Distance kPositionTolerance = Centimeter.of(1.0);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);
    public static final Time kEndTriggerDebounce = Seconds.of(0.1);

    private PositionPID(SwerveSubsystem mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

            var rotation = MathUtil.isNear(0.0, diff.getRotation().getRotations(),
                    kRotationTolerance.getRotations(), 0.0, 1.0);

            var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

            var speed = mSwerve.getSpeed() < kSpeedTolerance.in(MetersPerSecond);

            System.out.println(
                    "end trigger conditions R: " + rotation + "\tP: " + position + "\tS: " + speed);

            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout) {
        return new PositionPID(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.drive(new ChassisSpeeds(0, 0, 0));
        });
    }

    public static Command generateDeferredCommand(SwerveSubsystem swerve, Supplier<Pose2d> goalPose,
            Time timeout) {
        return Commands.defer(() -> {
            return generateCommand(swerve, goalPose.get(), timeout);
        }, Set.of(swerve));
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        mSwerve.drive(mDriveController.calculateRobotRelativeSpeeds(mSwerve.getPose(), goalState));
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
