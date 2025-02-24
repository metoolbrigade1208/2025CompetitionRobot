// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ListIterator;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class LocationService extends SubsystemBase {
  /** Creates a new LocationService. */
  public LocationService(SwerveDrive drive) {
    m_drive = drive;
  }

  List<AprilTag> tags =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags();
  private SwerveDrive m_drive;

  // Returns the closest tag ID to the robot
  public int closestTagId() {
    Pose2d robot = m_drive.getPose();
    ListIterator<AprilTag> iter = tags.listIterator();
    int closestTagId = -1;
    double closestDistance = Double.MAX_VALUE;
    while (iter.hasNext()) {
      AprilTag tag = iter.next();
      double distance =
          robot.getTranslation().getDistance(tag.pose.getTranslation().toTranslation2d());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTagId = tag.ID;
      }
    }
    return closestTagId;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

