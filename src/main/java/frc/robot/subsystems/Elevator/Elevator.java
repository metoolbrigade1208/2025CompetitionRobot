// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

@Logged
public class Elevator extends SubsystemBase implements AutoCloseable {

  // Singleton stuff
  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Instance not created yet");
    }
    return instance;
  }

  // This gearbox represents a gearbox containing 2 NEO motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

  // Standard classes for controlling our elevator

  private final SparkMax m_motor =
      new SparkMax(Constants.elevator.kMotorPort, MotorType.kBrushless);
  private final SparkMax m_motor2 =
      new SparkMax(Constants.elevator.kMotorPort2, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final SparkClosedLoopController m_controller2 = m_motor2.getClosedLoopController();

  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final RelativeEncoder m_encoder2 = m_motor2.getEncoder();

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(m_elevatorGearbox, Constants.elevator.kElevatorGearing,
          Constants.elevator.kCarriageMass, Constants.elevator.kElevatorDrumRadius,
          Constants.elevator.kMinElevatorHeightMeters, Constants.elevator.kMaxElevatorHeightMeters,
          true, Constants.elevator.kMinElevatorHeightMeters, 0.01, 0.0);

  private final SparkRelativeEncoderSim m_encoderSim = new SparkRelativeEncoderSim(m_motor);
  private final SparkRelativeEncoderSim m_encoderSim2 = new SparkRelativeEncoderSim(m_motor2);
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);
  private final SparkMaxSim m_motorSim2 = new SparkMaxSim(m_motor2, m_elevatorGearbox);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot
      .append(new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  DigitalInput input = new DigitalInput(Constants.elevator.kLimitSwitchPort);

  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(Constants.elevator.kElevatorkS, Constants.elevator.kElevatorkG,
          Constants.elevator.kElevatorkV, Constants.elevator.kElevatorkA);

  // keep max and min for this a bit away from the actual ends.
  public final Trigger atMin = new Trigger(() -> MathUtil.isNear(getHeightMeters(),
      Constants.elevator.kMinElevatorHeightMeters + 0.1, Inches.of(1).in(Meters)));
  public final Trigger atMax = new Trigger(() -> MathUtil.isNear(getHeightMeters(),
      Constants.elevator.kMaxElevatorHeightMeters - 0.1, Inches.of(1).in(Meters)));
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutVoltage m_appliedVoltage2 = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutDistance m_distance2 = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final MutLinearVelocity m_velocity2 = MetersPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(Volts.per(Second).of(2), Volts.of(2), Seconds.of(30)),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motor(s).
          this::setMotorVolts,
          // Tell SysId how to record a frame of data for each motor on the mechanism being
          // characterized.
          log -> {
            // Record a frame for the elevator motor.
            log.motor("elevator1")
                .voltage(m_appliedVoltage.mut_replace(
                    m_motor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getHeightMeters(m_encoder), Meters))
                // Records Height in Meters via SysIdRoutineLog.linearPosition
                .linearVelocity(
                    m_velocity.mut_replace(getVelocityMetersPerSecond(m_encoder), MetersPerSecond));
            log.motor("elevator2")
                .voltage(m_appliedVoltage2.mut_replace(
                    m_motor2.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance2.mut_replace(getHeightMeters(m_encoder2), Meters))
                // Records Height in Meters via SysIdRoutineLog.linearPosition
                .linearVelocity(m_velocity2.mut_replace(getVelocityMetersPerSecond(m_encoder2),
                    MetersPerSecond));
            // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
          }, this));

  public Elevator() {
    ElevatorLevelSub = ElevatorLevelTopic.subscribe(1);
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SparkMaxConfig motor1config = new SparkMaxConfig();
    motor1config.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode();
    double limitDistInRot =
        Units.inchesToMeters(28.5 * 2) / Constants.elevator.kPositionConversionFactor;
    motor1config.softLimit.forwardSoftLimit(limitDistInRot).reverseSoftLimit(0.0)
        .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    motor1config.closedLoop
        .pid(Constants.elevator.kElevatorKp, Constants.elevator.kElevatorKi,
            Constants.elevator.kElevatorKd, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0).maxMotion
            .maxVelocity(5000, ClosedLoopSlot.kSlot0).maxAcceleration(8000, ClosedLoopSlot.kSlot0);

    m_motor.configure(motor1config, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    SparkMaxConfig motor2config = new SparkMaxConfig().apply(motor1config);
    motor2config.inverted(false);
    // motor2config.follow(m_motor, true);
    m_motor2.configure(motor2config, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    if (instance != null) {
      throw new IllegalStateException("Cannot create new instance of singleton class");
    }
    instance = this;
    // this.setDefaultCommand(elevatorStop());
  }

  private double getVelocityMetersPerSecond(RelativeEncoder encoder) {
    return encoder.getVelocity() * Constants.elevator.kVelocityConversionFactor;
  }

  private double getVelocityMetersPerSecond() {
    return getVelocityMetersPerSecond(m_encoder);
  }

  private double getHeightMeters() {
    return getHeightMeters(m_encoder);
  }

  private double getHeightMeters(RelativeEncoder encoder) {
    return encoder.getPosition() * Constants.elevator.kPositionConversionFactor;
  }

  private void setMotorVolts(Voltage volts) {
    m_motor.setVoltage(volts);
    m_motor2.setVoltage(volts);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulat ion of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    m_motorSim.iterate(
        m_elevatorSim.getVelocityMetersPerSecond() / Constants.elevator.kVelocityConversionFactor,
        RoboRioSim.getVInVoltage(), 0.020);
    m_motorSim2.iterate(
        m_elevatorSim.getVelocityMetersPerSecond() / Constants.elevator.kVelocityConversionFactor,
        RoboRioSim.getVInVoltage(), 0.020);
    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // We set the simulated motor voltage and current draw
    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_encoderSim.setPosition(
        m_elevatorSim.getPositionMeters() / Constants.elevator.kPositionConversionFactor);
    m_encoderSim2.setPosition(
        m_elevatorSim.getPositionMeters() / Constants.elevator.kPositionConversionFactor);
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps() * 2));

  }

  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
    if (isAtBottom()) {
      m_encoder.setPosition(0);
      m_encoder2.setPosition(0);
      if (currentGoalRotations == 0) {
        m_motor.set(0);
        m_motor2.set(0);
      }
    }
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goalMeters the position to maintain
   */
  private double currentGoalRotations;

  public void reachGoal(double goalMeters) {

    goalMeters = goalMeters - Constants.LEVEL_1;
    currentGoalRotations = goalMeters / Constants.elevator.kPositionConversionFactor;
    System.out.print("goal Rot: ");
    System.out.println(currentGoalRotations);
    m_controller.setReference(currentGoalRotations, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0); // feedfowards here?
    m_controller2.setReference(currentGoalRotations, ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);// feedfowards here?
    // With the setpoint value we run PID control like normal
  }

  public void setVelocity(double velocity) {
    m_controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    m_controller2.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    // m_controller.setReference(0.0, ControlType.kVoltage);
    // m_controller2.setReference(0.0, ControlType.kVoltage);
    m_motor.set(0.0);
    m_motor2.set(0.0);
  }

  public boolean isAtBottom() {
    return input.get();
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getPosition());
    SmartDashboard.putNumber("motor1encoder", m_encoder.getPosition());
  }

  // Commands for Elevator setpoints
  public Command elevatorLevelIntakeCommand() {
    return runOnce(() -> reachGoal(Constants.LEVEL_Intake));
  }

  public Command elevatorLevel1Command() {
    return runOnce(() -> reachGoal(Constants.LEVEL_1));
  }

  public Command elevatorLevel2Command() {
    return runOnce(() -> reachGoal(Constants.LEVEL_2));
  }

  public Command elevatorLevel3Command() {
    return runOnce(() -> reachGoal(Constants.LEVEL_3));
  }

  public Command elevatorLevel4Command() {
    return runOnce(() -> reachGoal(Constants.LEVEL_4));
  }

  public BooleanSupplier elevatorAtLevel = (() -> java.lang.Math.abs(m_encoder.getPosition()
      - currentGoalRotations) < Constants.elevator.kElevatorPositionTolerance);

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("SmartDashboard");

  IntegerTopic ElevatorLevelTopic = table.getIntegerTopic("ElevatorLevel");
  IntegerSubscriber ElevatorLevelSub;

  public Command elevatorleveldataCommand() {
    int ElevatorLevelNum = (int) ElevatorLevelSub.get();
    switch (ElevatorLevelNum) {
      case 1:
        return elevatorLevel1Command();
      case 2:
        return elevatorLevel2Command();
      case 3:
        return elevatorLevel3Command();
      case 4:
        return elevatorLevel4Command();
      default:
        return elevatorLevel1Command();
    }
  }

  public Command elevatorUp() {
    return runOnce(() -> {
      m_motor.set(.1);
      m_motor2.set(.1);
    });
  }

  public Command elevatorDown() {
    return runOnce(() -> {
      m_motor.set(-.1);
      m_motor2.set(-.1);
    });
  }

  public Command elevatorStop() {
    return runOnce(() -> {
      stop();
    });
  }

  @Override
  public void close() {
    m_motor.close();
    m_motor2.close();
    m_mech2d.close();
  }

  /**
   * Runs the SysId routine to tune the Arm
   *
   * @return SysId Routine command
   */
  public Command runSysIdRoutine() {
    return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin));
  }
}
