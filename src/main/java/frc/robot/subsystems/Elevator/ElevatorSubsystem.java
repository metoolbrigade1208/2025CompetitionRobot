// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import yams.mechanisms.SmartMechanism;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {

  // Singleton stuff
  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Instance not created yet");
    }
    return instance;
  }

  // Standard classes for controlling our elevator

  private final SparkMax m_motorLeader =
      new SparkMax(Constants.elevator.kMotorPort, MotorType.kBrushless);
  private final SparkMax m_motorFollower =
      new SparkMax(Constants.elevator.kMotorPort2, MotorType.kBrushless);


  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Mechanism Circumference is the distance traveled by each mechanism rotation converting
      // rotations to meters.
      .withMechanismCircumference(Constants.elevator.kElevatorDrumCirc)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(Constants.elevator.kElevatorKp, 0, Constants.elevator.kElevatorKd,
          MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      .withSimClosedLoopController(Constants.elevator.kElevatorKp, 0,
          Constants.elevator.kElevatorKd, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      // Feedforward Constants
      .withFeedforward(new ElevatorFeedforward(Constants.elevator.kElevatorkS,
          Constants.elevator.kElevatorkG, Constants.elevator.kElevatorkV))
      .withSimFeedforward(new ElevatorFeedforward(Constants.elevator.kElevatorkS,
          Constants.elevator.kElevatorkG, Constants.elevator.kElevatorkV))
      // Telemetry name and verbosity level
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the
      // gearbox attached to your motor.
      .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(3, 3).div(2)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false).withIdleMode(MotorMode.BRAKE).withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25)).withOpenLoopRampRate(Seconds.of(0.25))
      .withFollowers(Pair.of(m_motorFollower, true));

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(m_motorLeader, DCMotor.getNEO(1), smcConfig);

  private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
      .withStartingHeight(Constants.elevator.kMinElevatorHeightMeters)
      .withHardLimits(Meters.of(0), Meters.of(3)).withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Constants.elevator.kCarriageMass);

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

  private final RelativeEncoder m_encoder = m_motorLeader.getEncoder();
  DigitalInput input = new DigitalInput(Constants.elevator.kLimitSwitchPort);

  public ElevatorSubsystem() {
    ElevatorLevelSub = ElevatorLevelTopic.subscribe(1);
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim


    // motor2config.apply(motor1config);
    // motor2config.inverted(false);
    if (instance != null) {
      throw new IllegalStateException("Cannot create new instance of singleton class");
    }
    instance = this;
    // this.setDefaultCommand(elevatorStop());
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulat ion of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // Next, we update it. The standard loop time is 20ms.

    // We set the simulated motor voltage and current draw
    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage

    // SimBattery estimates loaded battery voltages

  }

  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goalMeters the position to maintain
   */
  private double currentGoalRotations;

  public Command setHeight(Distance goalMeters) {
    System.out.print("commanded height: ");
    System.out.println(goalMeters);
    return elevator.setHeight(goalMeters);
    // m_controller2.setReference(currentGoalRotations,
    // ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    // With the setpoint value we run PID control like normal
  }

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() {
    return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }


  /**
   * Move the elevator up and down.
   * 
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    return elevator.set(dutycycle);
  }


  /** Stop the control loop and motor output. */
  public void stop() {
    // m_controller.setReference(0.0, ControlType.kVoltage);
    // m_controller2.setReference(0.0, ControlType.kVoltage);
    // m_motor2.set(0.0);
  }

  public boolean isAtBottom() {
    return input.get();
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
  }

  // Commands for Elevator setpoints

  public Command elevatorLevel1Command() {
    return setHeight(Constants.LEVEL_1);
  }

  public Command elevatorLevel2Command() {
    return setHeight(Constants.LEVEL_2);
  }

  public Command elevatorLevel3Command() {
    return setHeight(Constants.LEVEL_3);
  }

  public Command elevatorLevel4Command() {
    return setHeight(Constants.LEVEL_4);
  }

  public BooleanSupplier elevatorAtLevel = (() -> java.lang.Math.abs(m_encoder.getPosition()
      - currentGoalRotations) < Constants.elevator.kElevatorPositionTolerance.baseUnitMagnitude());

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("SmartDashboard");

  IntegerTopic ElevatorLevelTopic = table.getIntegerTopic("ElevatorLevel");
  IntegerSubscriber ElevatorLevelSub;

  public Command elevatorleveldataCommand() {
    int ElevatorLevelNum = (int) ElevatorLevelSub.get();
    Distance ElevatorLevel = Constants.LEVEL_1;
    switch (ElevatorLevelNum) {
      case 1:
        ElevatorLevel = Constants.LEVEL_1;
        break;
      case 2:
        ElevatorLevel = Constants.LEVEL_2;
        break;
      case 3:
        ElevatorLevel = Constants.LEVEL_3;
        break;
      case 4:
        ElevatorLevel = Constants.LEVEL_4;
        break;
      default:
        ElevatorLevel = Constants.LEVEL_1;
    }
    return setHeight(ElevatorLevel);
  }

  public Command elevatorUp() {
    return set(0.3);
  }

  public Command elevatorDown() {
    return set(-0.3);
  }

  public Command elevatorStop() {
    return set(0);
  }

  @Override
  public void close() {}
}
