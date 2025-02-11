// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.IntakeSimulation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements AutoCloseable {
  // The P gain for the PID controller that drives this arm.
  private double m_armKp = Constants.IntakeConstants.kArmKp;
  private double m_armSetpointDegrees = Constants.IntakeConstants.kDefaultArmSetpointDegrees;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getNEO(1); 

  // Motor and encoder for deploying arm.
  private final SparkMax m_armMotor = new SparkMax(Constants.IntakeConstants.kArmMotorPort, MotorType.kBrushless);
  // Standard classes for controlling our arm
  private final SparkClosedLoopController m_controller = m_armMotor.getClosedLoopController();
  // Motor and IR sensor for intake.
  private final SparkMax m_intakeMotor = new SparkMax(Constants.IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
  private final DigitalInput m_coraldetect = new DigitalInput(Constants.IntakeConstants.kIRsensorport);

  private IntakeSimulation m_IntakeSim;

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down
  // front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
      m_armGearbox,
      Constants.IntakeConstants.kArmReduction,
      SingleJointedArmSim.estimateMOI(Constants.IntakeConstants.kArmLength, Constants.IntakeConstants.kArmMass),
      Constants.IntakeConstants.kArmLength,
      Constants.IntakeConstants.kMinAngleRads,
      Constants.IntakeConstants.kMaxAngleRads,
      true,
      0,
      Constants.IntakeConstants.kArmEncoderDistPerPulse,
      0.0 // Add noise with a std-dev of 1 tick
  );
  

  private final SparkAbsoluteEncoderSim m_encoderSim = new SparkAbsoluteEncoderSim(m_armMotor);
  private final SparkMaxSim m_armMotorSim = new SparkMaxSim(m_armMotor, m_armGearbox);


  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm = m_armPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(m_armSim.getAngleRads()),
          6,
          new Color8Bit(Color.kYellow)));

  /** Subsystem constructor. */
  public Intake(SwerveDrive drivetrain) {
    // m_encoder.setDistancePerPulse(Constants.IntakeConstants.kArmEncoderDistPerPulse);
    m_IntakeSim = IntakeSimulation.OverTheBumperIntake("Coral", drivetrain.getMapleSimDrive().get(), Inches.of(28),
        Inches.of(8), IntakeSimulation.IntakeSide.FRONT, 1);
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // Configure the arm motor
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    armMotorConfig.closedLoop
        .pid(Constants.IntakeConstants.kArmKp,
            Constants.IntakeConstants.kArmKi,
            Constants.IntakeConstants.kArmKd, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armMotorConfig.closedLoop.maxMotion
        .maxAcceleration(Constants.IntakeConstants.kArmMaxAcceleration)
        .maxVelocity(Constants.IntakeConstants.kArmMaxSpeed)
        .allowedClosedLoopError(Constants.IntakeConstants.kArmMaxError);

    armMotorConfig.encoder.positionConversionFactor(360.0); // degrees
    m_armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure the intake motor
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

        intakeMotorConfig.closedLoop
            .pidf(Constants.IntakeConstants.kIntakeKp,
            Constants.IntakeConstants.kIntakeKi, Constants.IntakeConstants.kIntakeKd, 1.0/Constants.IntakeConstants.kIntakeKv, ClosedLoopSlot.kSlot0);

    // Set the Arm position setpoint and P constant to Preferences if the keys don't
    // already exist
    Preferences.initDouble(Constants.IntakeConstants.kArmPositionKey, m_armSetpointDegrees);
    Preferences.initDouble(Constants.IntakeConstants.kArmPKey, m_armKp);
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_armMotor.get() * RobotController.getBatteryVoltage());

    m_armMotorSim.iterate(m_armSim.getVelocityRadPerSec() / Constants.IntakeConstants.kArmPositionConversionFactor,RoboRioSim.getVInVoltage(), 0.020);
    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_encoderSim.setPosition(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpointDegrees = Preferences.getDouble(Constants.IntakeConstants.kArmPositionKey, m_armSetpointDegrees);
  }

  /**
   * Run the control loop to reach and maintain the setpoint from the preferences.
   */
  public void reachSetpoint() {
    m_controller.setReference(m_armSetpointDegrees, ControlType.kMAXMotionPositionControl);
  }

  public void stoparm() {
    m_armMotor.set(0.0);
  }

  // sets intake speed
  public void setintakespeed(Double speed) {
    m_intakeMotor.set(speed);
    m_IntakeSim.startIntake();
  }

  // stops intake
  public void stopintake() {
    m_intakeMotor.set(0.0);
    m_IntakeSim.stopIntake();
  }

  // gets IR sensor output as a boolean
  public boolean IsDetected() {
    if (Robot.isSimulation()) {
      return m_IntakeSim.getGamePiecesAmount() > 0;
    }
    return m_coraldetect.get();
  }

  // Commands for arm
  public Command armDownCommand() {
    return runOnce(() -> this.m_armSetpointDegrees = Constants.IntakeConstants.kArmDownPosition);
  }

  public Command armUpCommand() {
    return runOnce(() -> this.m_armSetpointDegrees = Constants.IntakeConstants.kArmUpPosition);
  }

  // Commands for Intake
  public Command startIntakeCommand() {
    return new FunctionalCommand(
        () -> this.setintakespeed(Constants.IntakeConstants.kIntakeRunSpeed), // on Start
        () -> {
        }, // do nothing while running
        interrupted -> this.stopintake(), // on stop
        () -> this.IsDetected(), // stop when coral detected
        this);
  }

  // @Override
  public void close() {
    m_armMotor.close();
    // m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_arm.close();
  }
}