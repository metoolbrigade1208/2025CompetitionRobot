// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

  // Standard classes for controlling our elevator
  private final SparkMax m_motor = new SparkMax(Constants.Elevator.kMotorPort, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();

  private final RelativeEncoder m_encoder;
  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      m_elevatorGearbox,
      Constants.Elevator.kElevatorGearing,
      Constants.Elevator.kCarriageMass,
      Constants.Elevator.kElevatorDrumRadius,
      Constants.Elevator.kMinElevatorHeightMeters,
      Constants.Elevator.kMaxElevatorHeightMeters,
      true,
      0.5,
      0.01,
      0.0);

  // private final SparkRelativeEncoderSim m_encoderSim = new
  // SparkRelativeEncoderSim(m_motor);
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 1);
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Subsystem constructor. */
  public Elevator() {

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SparkMaxConfig motor1config = new SparkMaxConfig();
    motor1config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .closedLoopRampRate(.5);
    motor1config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.Elevator.kElevatorKp, Constants.Elevator.kElevatorKi,
            Constants.Elevator.kElevatorKd)
        .outputRange(-1, 1);

    motor1config.encoder.positionConversionFactor(Constants.Elevator.kPositionConversionFactor)
        .velocityConversionFactor(Constants.Elevator.kVelocityConversionFactor);

    m_motor.configure(motor1config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_encoder = m_motor.getEncoder();
    reachGoal(0.7);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    final double loopTime = .02;
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    double elevVolts = m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    m_elevatorSim.setInput(elevVolts);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(loopTime);
    m_motorSim.iterate(
        Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
            .per(Second).in(RPM),
        RoboRioSim.getVInVoltage(), loopTime);

    // We set the simulated motor voltage and current draw
    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    double positionMeters = m_elevatorSim.getPositionMeters();
    m_encoder.setPosition(positionMeters);

    SmartDashboard.putNumber("ElevatorSimPosition", positionMeters);
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    // i feel like we need to add something to tell the sim to go to the goal; no
    // idea what though
    m_controller.setReference(goal, ControlType.kPosition);
    // With the setpoint value we run PID control like normal
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setReference(0.0, ControlType.kVoltage);
    m_motor.set(0.0);
  }

  public static Angle convertDistanceToRotations(Distance distance) {
    return Rotations.of(distance.in(Meters) /
        (Constants.Elevator.kElevatorDrumRadius * 2 * Math.PI) *
        Constants.Elevator.kElevatorGearing);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getPosition());
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

  @Override
  public void close() {
    m_motor.close();
    ;
    m_mech2d.close();
  }
}