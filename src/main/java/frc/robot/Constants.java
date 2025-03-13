// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Function;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Optional<RobotConfig> config = loadConfig(
      Filesystem.getDeployDirectory().toPath().resolve("config.json").toString());

  private static double getConfigValue(Function<RobotConfig, Double> mapper, double defaultValue) {
    return config.flatMap(c -> Optional.ofNullable(mapper.apply(c))).orElse(defaultValue);
  }

  public static final double LEVEL_Intake = Units.inchesToMeters(getConfigValue(c -> c.inIntake, 18.0));
  public static final double LEVEL_1 = Units.inchesToMeters(getConfigValue(c -> c.inL1, 18.0));
  public static final double LEVEL_2 = Units.inchesToMeters(getConfigValue(c -> c.inL2, 32.0));
  public static final double LEVEL_3 = Units.inchesToMeters(getConfigValue(c -> c.inL3, 48.0));
  public static final double LEVEL_4 = Units.inchesToMeters(getConfigValue(c -> c.inL4, 72.0));

  public static final double ROBOT_MASS = Units.lbsToKilograms(148 - 20.3); // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = 4.8;
  public static final double MAX_ACCELERATION = 5.0;
  public static final Distance kRobotLength = Inches.of(41);
  public static final Distance kRobotWidth = Inches.of(30);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class IntakeConstants {
    public static final int kArmMotorPort = 62;
    public static final int kArmMotor2Port = 61;
    public static final int kIntakeMotorPort = 60;
    public static final int kIRsensorport = 3;
    public static final int kArmUpLimitPort = 4;

    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmKp = 50.0;
    public static final double kDefaultArmSetpointDegrees = 75.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    public static final double kArmReduction = 64.0;
    public static final double kArmEncoderGearing = 4.0 / 3.0; // ratio of encoder position to arm
                                                               // position
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-15);
    public static final double kMaxAngleRads = Units.degreesToRadians(100);
    public static final double kArmKp = getConfigValue(c -> c.armKp, 1.0);
    public static final double kArmKi = getConfigValue(c -> c.armKi, 0);
    public static final double kArmKd = getConfigValue(c -> c.armKd, 0.01);
    public static final double kArmKs = 0;
    public static final double kArmKg = 0;
    public static final double kArmKv = 0;
    public static final double kArmKa = 0;
    public static final double kArmDownPosition = Units.degreesToRotations(90);
    public static final double kArmUpPosition = 0;
    public static final double kArmMaxSpeed = 1000;
    public static final double kArmMaxAcceleration = 1500;
    public static final double kArmMaxError = Units.degreesToRotations(1);

    public static final double kIntakeRunSpeed = 1.0;

    public static final double kIntakeKp = getConfigValue(c -> c.intakeKp, 0);
    public static final double kIntakeKi = getConfigValue(c -> c.intakeKi, 0);
    public static final double kIntakeKd = getConfigValue(c -> c.intakeKd, 0);
    public static final double kIntakeKv = 473;
    public static final double kIntakeMaxSpeed = getConfigValue(c -> c.intakeMaxSpeed, 0.5);
    public static final double kIntakeMaxAcceleration = 0.5;
    public static final double kIntakeMaxError = 0.5;

    public static final double kArmPositionConversionFactor = 2 * Math.PI;

  }

  public static class elevator {
    public static final double kElevatorKp = 0.2;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 5;

    public static final double kElevatorkS = 0;
    public static final double kElevatorkG = 0;
    public static final double kElevatorkV = 473; // for NEO V1.1
    public static final double kElevatorkA = 0;
    public static final int kMotorPort = 51;
    public static int kMotorPort2 = 52;
    public static int kLimitSwitchPort = 6;

    public static final double kElevatorGearing = 9.0;
    public static final double kCarriageMass = 10.0;
    public static final double kElevatorDrumCirc = Units.inchesToMeters(.25) * 22; // 22 teeth
                                                                                   // number 25
                                                                                   // chain (quater
                                                                                   // inch)
    public static final double kElevatorDrumRadius = kElevatorDrumCirc / (2 * Math.PI);
    public static final double kMinElevatorHeightMeters = LEVEL_1;
    public static final double kMaxElevatorHeightMeters = LEVEL_4;
    // Position is rotation to meter
    public static final double kPositionConversionFactor = (kElevatorDrumCirc / kElevatorGearing) * 2;
    // Velocity is rpm to mps
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final double kVelocityMultiplier = 5.0;
    public static final double kElevatorPositionTolerance = 0.5 / kPositionConversionFactor;
  }

  static Optional<RobotConfig> loadConfig(String path) {
    ObjectMapper objectMapper = new ObjectMapper();
    try {
      return Optional.of(objectMapper.readValue(new File(path), RobotConfig.class));
    } catch (IOException e) {
      e.printStackTrace();
      return Optional.empty();
    }
  }

  public static class OutputConstants {
    public static final int kOutputMotorPort = 55; // SparkMax moter for output
    public static final int kOutputGearbox = 1; // Gearbox for output
    public static final int kIRsensorport = 9; // IR sensor for output

    public static final double kOutputKp = 0;
    public static final double kOutputKi = 0;
    public static final double kOutputKd = 0;
    public static final double kOutputKv = 473;
    public static final double kOutputMaxSpeed = 0.5;
    public static final double kOutputMaxAcceleration = 0.5;
    public static final double kOutputMaxError = 0.5;
    public static final double kOutputRunSpeed = 1.0;
  }

}
