// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LocationService extends SubsystemBase {
  // singleton instance
  private static LocationService instance;

  public static LocationService getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Instance not created yet");
    }
    return instance;
  }

  private static final AprilTagFieldLayout field =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  /** Creates a new LocationService. */
  public LocationService() {
    m_drive = SwerveSubsystem.getInstance().getSwerveDrive();
    offsetSub = OffsetTopic.subscribe(0);
    if (instance != null) {
      throw new IllegalStateException("Cannot create new instance of singleton class");
    }
    instance = this;
  }

  public enum Offset {
    LEFT(-1), RIGHT(1), CENTER(0);

    private int val;

    Offset(int i) {
      val = i;
    }

    public int getVal() {
      return val;
    }



  }

  List<AprilTag> tags = field.getTags();
  private SwerveDrive m_drive;
  private List<Integer> redSourceTags = List.of(1, 2);
  private List<Integer> blueSourceTags = List.of(12, 13);
  private List<Integer> redReefTags = List.of(6, 7, 8, 9, 10, 11);
  private List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);
  private List<Integer> bargeTags = List.of(4, 5, 14, 15);

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("SmartDashboard");

  IntegerTopic OffsetTopic = table.getIntegerTopic("Offset");
  IntegerSubscriber offsetSub;

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

  // in source region
  public boolean inSourceRegion() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return redSourceTags.contains(closestTagId()); // red alliance source april tags
      }
      if (ally.get() == Alliance.Blue) {

        return blueSourceTags.contains(closestTagId()); // blue alliance source april tags
      }
    }
    return false;
  }

  // in reef region
  public boolean inReefRegion() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return redReefTags.contains(closestTagId()); // red alliance reef april tags
      }
      if (ally.get() == Alliance.Blue) {
        return blueReefTags.contains(closestTagId()); // blue alliance reef april tags
      }
    }
    return false;
  }

  // in processor region
  public boolean inProcessorRegion() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return closestTagId() == 3; // red alliance processor april tag
      }
      if (ally.get() == Alliance.Blue) {
        return closestTagId() == 16; // blue alliance processor april tag
      }
    }
    return false;
  }

  // in barge region
  public boolean inBargeRegion() {
    return bargeTags.contains(closestTagId()); // barge april tags
  }

  /**
   * Generates a Pose2d for the reef region from a tag ID
   * 
   * OFFSET is whether to be offset to the left or right of the tag
   * 
   * @param TagID - the tag ID to generate the pose from
   * @param offset - if null, it will return as if used CENTER
   * @return Pose2d - the pose of the robot lined up on the tag.
   */
  public Pose2d genPoseForReefFromTag(int TagID, Offset offset) {
    double inOffset = 0;
    offset = offset == null ? Offset.CENTER : offset;
    switch (offset) {
      case LEFT:
        inOffset = -12.94 / 2;
        break;
      case RIGHT:
        inOffset = 12.94 / 2;
        break;
      default:
        break;
    }
    Pose2d tagPose = field.getTagPose(TagID).orElse(new Pose3d()).toPose2d();
    Transform2d poseOffset = new Transform2d(Constants.kRobotWidth.div(2), Inches.of(inOffset),
        Rotation2d.fromDegrees(90));
    return tagPose.transformBy(poseOffset);
  }

  public Pose2d genPoseForSourceFromTag(int TagID, Offset offset) {
    double inOffset = 0;
    offset = offset == null ? Offset.CENTER : offset;
    switch (offset) {
      case LEFT:
        inOffset = -12.94 / 2;
        break;
      case RIGHT:
        inOffset = 12.94 / 2;
        break;
      default:
        break;
    }
    Pose2d tagPose = field.getTagPose(TagID).orElse(new Pose3d()).toPose2d();
    Transform2d poseOffset = new Transform2d(Constants.kRobotWidth.div(2), Inches.of(inOffset),
        Rotation2d.fromDegrees(270));
    return tagPose.transformBy(poseOffset);
  }

  public Pose2d genPoseForProcessorFromTag(int TagID) {
    Pose2d tagPose = field.getTagPose(TagID).orElse(new Pose3d()).toPose2d();
    Transform2d poseOffset =
        new Transform2d(Constants.kRobotWidth.div(2), Inches.of(0.0), Rotation2d.fromDegrees(0));
    return tagPose.transformBy(poseOffset);
  }

  public Pose2d getTagAutoPose2d() {
    int offsetNum = (int) offsetSub.get();
    Offset offset = Offset.CENTER;
    switch (offsetNum) {
      case -1:
        offset = Offset.LEFT;
        break;
      case 1:
        offset = Offset.RIGHT;
        break;
      default:
        break;
    }
    if (inReefRegion()) {
      return genPoseForReefFromTag(closestTagId(), offset);
    }
    if (inSourceRegion()) {
      return genPoseForSourceFromTag(closestTagId(), offset);
    }
    if (inProcessorRegion()) {
      return genPoseForProcessorFromTag(closestTagId());
    }
    return null;
  }

  public BooleanSupplier autoPoseBool(double maxPoseDistance) {
    return () -> (m_drive.getPose().getTranslation()
        .getDistance(getTagAutoPose2d().getTranslation())) < maxPoseDistance;
  }

  public BooleanSupplier atAutoPose() {
    return autoPoseBool(Units.inchesToMeters(2));
  }

  public BooleanSupplier nearAutoPose() {
    return autoPoseBool(Units.inchesToMeters(36));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

