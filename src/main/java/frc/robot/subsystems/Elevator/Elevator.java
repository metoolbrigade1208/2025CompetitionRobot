// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

  // Standard classes for controlling our elevator

  private final SparkMax m_motor = new SparkMax(Constants.elevatorConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMax m_motor2 = new SparkMax(Constants.elevatorConstants.kMotorPort2, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();

  private final RelativeEncoder m_encoder = m_motor.getAlternateEncoder();
  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      m_elevatorGearbox,
      Constants.elevatorConstants.kElevatorGearing,
      Constants.elevatorConstants.kCarriageMass,
      Constants.elevatorConstants.kElevatorDrumRadius,
      Constants.elevatorConstants.kMinElevatorHeightMeters,
      Constants.elevatorConstants.kMaxElevatorHeightMeters,
      true,
      0,
      0.01,
      0.0);

  private final SparkRelativeEncoderSim m_encoderSim = new SparkRelativeEncoderSim(m_motor);
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Subsystem constructor. */
  public Elevator() {

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SparkMaxConfig motor1config = new SparkMaxConfig();
    motor1config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    motor1config.closedLoop
        .pid(Constants.elevatorConstants.kElevatorKp, Constants.elevatorConstants.kElevatorKi,
            Constants.elevatorConstants.kElevatorKd, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0).maxMotion
        .maxVelocity(2500, ClosedLoopSlot.kSlot0)
        .maxAcceleration(5000, ClosedLoopSlot.kSlot0);
    motor1config.closedLoop
        .pid(Constants.elevatorConstants.kElevatorKp, Constants.elevatorConstants.kElevatorKi,
            Constants.elevatorConstants.kElevatorKd, ClosedLoopSlot.kSlot1)
        .velocityFF(1 / Constants.elevatorConstants.kElevatorkV, ClosedLoopSlot.kSlot1).maxMotion
        .maxAcceleration(5000, ClosedLoopSlot.kSlot1); // no max velocity, because it's in velocity control mode for this,
                                                    // not position control
/*     motor1config.limitSwitch.setSparkMaxDataPortConfig()
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen); */

    m_motor.configure(motor1config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    SparkMaxConfig motor2config = new SparkMaxConfig();
    motor2config.follow(m_motor);
    m_motor2.configure(motor2config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond() / Constants.elevatorConstants.kVelocityConversionFactor, RoboRioSim.getVInVoltage(), 0.020);
    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    
    // We set the simulated motor voltage and current draw
    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_encoderSim.setPosition(m_elevatorSim.getPositionMeters());

    SmartDashboard.putNumber("ElevatorSimPosition", m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
        
   
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goalMeters the position to maintain
   */
  public void reachGoal(double goalMeters) {
    m_controller.setReference(goalMeters / Constants.elevatorConstants.kPositionConversionFactor, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // With the setpoint value we run PID control like normal
  }

  public void setVelocity(double velocity) {
    m_controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setReference(0.0, ControlType.kVoltage);
    m_motor.set(0.0);
  }

  public boolean isForwardLimitSwitchPressed() {
    return m_motor.getForwardLimitSwitch().isPressed();
  }

  public boolean isReverseLimitSwitchPressed() {
    return m_motor.getReverseLimitSwitch().isPressed();
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

  // command for manual override
  public Command elevatorManualOverideCommand(XboxController opXboxController) {
    return new FunctionalCommand(
        () -> {
        },
        () -> setVelocity(opXboxController.getLeftX() * Constants.elevatorConstants.kVelocityMultiplier),
        (done) -> stop(),
        () -> isForwardLimitSwitchPressed() || isReverseLimitSwitchPressed(),
        this);
  }

  @Override
  public void close() {
    m_motor.close();
    m_motor2.close();
    m_mech2d.close();
  }
}