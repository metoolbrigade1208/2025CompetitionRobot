// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LocationService;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.Output;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final ProfiledPIDController drivePoseTranslationPID = new ProfiledPIDController(10.0, 0.0,
      0.1, new Constraints(Constants.MAX_SPEED, Constants.MAX_ACCELERATION));
  private final ProfiledPIDController drivePoseAnglePIDController =
      new ProfiledPIDController(10.0, 0.0, 0.1, new Constraints(1000, 10000));
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...

  private static final boolean useDrivebase = true;
  private final SwerveSubsystem drivebase =
      useDrivebase ? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"))
          : null;

  // private final Intake intake = new Intake(drivebase.getSwerveDrive());

  private final LocationService locate = new LocationService();

  private static final boolean useIntake = true;
  private final Intake intake = useIntake ? new Intake() : null;

  private static final boolean useElevator = true;
  private final Elevator elevator = useElevator ? new Elevator() : null;

  private static final boolean useOutput = true;
  private final Output output = useOutput ? new Output() : null;


  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("tagRobotPoses");


  NetworkTable offsetTable = inst.getTable("SmartDashboard");

  IntegerTopic OffsetTopic = offsetTable.getIntegerTopic("Offset");
  IntegerPublisher offsetPub = OffsetTopic.publish();


  private int elevatorLevel = 1;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream
      .of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1,
          () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX).deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8).allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream
      .of(drivebase.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRawAxis(4))
      .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(true)
      .driveToPose(() -> autoPose(), drivePoseTranslationPID, drivePoseAnglePIDController)
      .driveToPoseEnabled(() -> autoPoseEnable());

  private Pose2d autoPose() {

    Pose2d tagAutoPose2d = locate.getTagAutoPose2d();
    if (tagAutoPose2d == null) {
      poseable = false;
      return new Pose2d();
    }
    return tagAutoPose2d;
  }

  private boolean poseable = false;

  private boolean autoPoseEnable() {
    return poseable;
  }

  // Derive the heading axis with math!p\
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(-driverXbox.getRawAxis(4) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(-driverXbox.getRawAxis(4) * Math.PI) * (Math.PI * 2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    drivePoseAnglePIDController.enableContinuousInput(0, Math.PI * 2);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(
          Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());


    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                       // above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      // publish tag based robot poses to network tables
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      driverXbox.rightTrigger(0.2).onTrue(intake.armUpCommand());
      driverXbox.rightTrigger(0.1).onFalse(intake.armDownCommand());
      driverXbox.rightTrigger(0.8).whileTrue(intake.startIntakeCommand());
      driverXbox.povLeft()
          .whileTrue(Commands.runOnce(() -> offsetPub.set(LocationService.Offset.LEFT.getVal())));
      driverXbox.povRight()
          .whileTrue(Commands.runOnce(() -> offsetPub.set(LocationService.Offset.RIGHT.getVal())));
      driverXbox.povCenter()
          .whileTrue(Commands.runOnce(() -> offsetPub.set(LocationService.Offset.CENTER.getVal())));
      driverXbox.rightStick().onTrue(Commands.runOnce(() -> poseable = true))
          .onFalse(Commands.runOnce(() -> poseable = false));
      // driverXbox.povUp().onTrue(Command.runOnce(() -> {
      // elevatorLevel = java.Math.max(elevatorLevel + 1, 4);
      // }));
      driverXbox.back().whileTrue(output.gripCoralCommand());
      driverXbox.start().whileTrue(output.ejectCoralCommand());
    }
    if (true)

    // Operator Bindings:
    // Right Trigger = go to intake position on elevator,
    // Left Trigger = manual override elevator controls (Must hold button to move joysitcks)
    // Dpad Down = go to elevator level 1
    // Dpad Right = go to elevator level 2
    // Dpad Up = go to elevator level 3
    // Dpad Left = go to elevator level 4

    {
      opXbox.rightTrigger().onTrue(elevator.elevatorLevelIntakeCommand());
      opXbox.povDown().onTrue(elevator.elevatorLevel1Command());
      opXbox.povRight().onTrue(elevator.elevatorLevel2Command());
      opXbox.povUp().onTrue(elevator.elevatorLevel3Command());
      opXbox.povLeft().onTrue(elevator.elevatorLevel4Command());
      opXbox.leftBumper().whileTrue(output.gripCoralCommand());
      opXbox.rightBumper().whileTrue(output.ejectCoralCommand());
      // opXbox.leftTrigger().whileTrue(elevator.elevatorManualOverideCommand(opXbox.getHID()));

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
