// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double LEVEL_Intake = (Units.inchesToMeters(37.5));
  public static final double LEVEL_1 = (Units.inchesToMeters(18));
  public static final double LEVEL_2 = (Units.inchesToMeters(32));
  public static final double LEVEL_3 = (Units.inchesToMeters(48));
  public static final double LEVEL_4 = (Units.inchesToMeters(72));

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = 1.5;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
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
    public static final int kArmMotorPort = 61;
    public static final int kIntakeMotorPort = 62;
    public static final int kIRsensorport = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmKp = 50.0;
    public static final double kDefaultArmSetpointDegrees = 75.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    public static final double kArmReduction = 200.0;
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-15);
    public static final double kMaxAngleRads = Units.degreesToRadians(100);
    public static final double kArmKp = 1;
    public static final double kArmKi = 0;
    public static final double kArmKd = 0.01;
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

    public static final double kIntakeKp = 0;
    public static final double kIntakeKi = 0;
    public static final double kIntakeKd = 0;
    public static final double kIntakeKv = 473;
    public static final double kIntakeMaxSpeed = 0.5;
    public static final double kIntakeMaxAcceleration = 0.5;
    public static final double kIntakeMaxError = 0.5;

    public static final double kArmPositionConversionFactor = 2 * Math.PI;

  }
  public static class elevator {
    public static final double kElevatorKp = 1;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    public static final double kElevatorkS = 0;
    public static final double kElevatorkG = 0;
    public static final double kElevatorkV = 473; // for NEO V1.1
    public static final double kElevatorkA = 0;
    public static final int kMotorPort = 51;
    public static int kMotorPort2 = 52;

    public static final double kElevatorGearing = 9.0;
    public static final double kCarriageMass = 10.0;
    public static final double kElevatorDrumCirc = Units.inchesToMeters(.25) * 22; // 22 teeth
                                                                                   // number 25
                                                                                   // chain (quater
                                                                                   // inch)
    public static final double kElevatorDrumRadius = kElevatorDrumCirc / 2 * Math.PI;
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 2.5;
    // Position is rotation to meter
    public static final double kPositionConversionFactor =
        (kElevatorDrumCirc / kElevatorGearing) * 2;
    // Velocity is rpm to mps
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final double kVelocityMultiplier = 5.0;
  }
}
