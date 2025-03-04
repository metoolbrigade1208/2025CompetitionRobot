// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.LocationService;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class AutoScoring extends Command {
  /** Creates a new AutoScoring. */
  public AutoScoring(LocationService locationService, Elevator elevator,
      SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(locationService);
    this.addRequirements(elevator);
    this.addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {

  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
