// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.SimulatedArena;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import java.util.Optional;
import static edu.wpi.first.units.Units.Degrees;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

// import org.ironmaple.simulation.OutputSimulation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.RelativeEncoder;

public class Output extends SubsystemBase implements AutoCloseable {

  // declares the motor on the output device
  private final SparkMax m_OutputMotor =
      new SparkMax(Constants.OutputConstants.kOutputMotorPort, MotorType.kBrushless);

  // declares the moter gearbox
  private final DCMotor m_OutputGearbox = DCMotor.getNEO(1);

  // declares IR Sensor
  private final DigitalInput m_coraldetect =
      new DigitalInput(Constants.OutputConstants.kIRsensorport);


  // declares the controller for the output motor
  // private final SparkClosedLoopController m_Outputcontroller =
  // m_OutputMotor.getClosedLoopController();
  // private final RelativeEncoder m_OutputEncoder = m_OutputMotor.getAlternateEncoder();

  public Output(SwerveDrive drivetrain) {
    m_OutputSim = new OutputSimulation(m_OutputGearbox);

    // Initialize Drive Train Simulation
    m_DriveSimulation = drivetrain.getMapleSimDrive().get();
    // Configure the output motor
    SparkMaxConfig OutputMotorConfig = new SparkMaxConfig();
    OutputMotorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    OutputMotorConfig.closedLoop.pidf(Constants.OutputConstants.kOutputKp,
        Constants.OutputConstants.kOutputKi, Constants.OutputConstants.kOutputKd,
        1.0 / Constants.OutputConstants.kOutputKv, ClosedLoopSlot.kSlot0);
  }

  // declares the simulation for the output
  public OutputSimulation m_OutputSim;
  private final SwerveDriveSimulation m_DriveSimulation;

  // Creates Output Simulation
  public class OutputSimulation {
    private final DCMotor m_OutputGearbox;


    public OutputSimulation(DCMotor OutputGearbox) {
      m_OutputGearbox = OutputGearbox;
    }

    public void startOutput() {
      // Check if the robot is in the specific area
      Translation2d robotPosition = m_DriveSimulation.getSimulatedDriveTrainPose().getTranslation();
      if (isInSourceArea(robotPosition)) {
        // If the robot is in the specific area, start the output
        runmotor();
        // check if the coral is detected
        if (m_coraldetect.get()) {
          // run the motor to grip the coral
          runmotoronce();
        }
      }
    }

    private boolean isInSourceArea(Translation2d position) {
      Rectangle2d sourceArea = new Rectangle2d(
          new Pose2d(Inches.of(0), Inches.of(0), new Rotation2d()), Inches.of(10), Inches.of(10));
    }


  }

  public void runmotor() {
    m_OutputMotor.set(1);
  }

  public void runmotorOnce() {
    SparkClosedLoopController outputController = m_OutputMotor.getClosedLoopController();
    outputController.setReference(2, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void stopOutput() {
    m_OutputMotor.set(0);
  }

  public void addGamePieceProjectile(SwerveDriveSimulation driveSimulation, double height) {
    SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.35, 0),
        // Obtain robot speed from drive simulation
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        driveSimulation.getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        Inches.of(height),
        // The initial speed of the coral
        InchesPerSecond.of(10),
        // The coral is ejected at a 35-degree slope
        Degrees.of(-35)));
  }

  public void OutputSimulation(DCMotor OutputGearbox, int motorPort, double moi, boolean flag,
      double radians, double encoderDistPerPulse, double noise) {
    // Initialize the fields or add the logic for the constructor
  }

  // public int getGamePiecesAmount() {
  // int gamePiecesAmount = 0;

  // Optional<ReefscapeReefSimulation> reefSimulation =
  // SimulatedArena.getInstance().getGamePiecesByType(ReefscapeReefSimulation.class);
  // if (reefSimulation.isPresent()) {
  // gamePiecesAmount = reefSimulation.get().getTotalGamePieces();
  // }
  // return gamePiecesAmount;

  // }



  public void periodic() {

  }

  // runs motor
  public void runmotor() {
    m_OutputMotor.set(1.0);
  }

  // grips onto coral after detection
  public void runmotoronce() {
    SparkClosedLoopController outputController = m_OutputMotor.getClosedLoopController();
    outputController.setReference(2, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // starts output
  public void setoutputspeed(Double speed) {

  }

  // runs motor again to grip coral
  public Command gripCoralCommand() {
    return new FunctionalCommand(() -> {
    }, () -> runmotor(), (x) -> runmotoronce(), () -> IsDetected(), this);

  }

  // shoots coral onto reef
  public Command ejectCoralCommand() {
    return new FunctionalCommand(() -> {
    }, () -> runmotor(), (x) -> runmotoronce(), () -> !IsDetected(), this);
  }



  // stops output
  public void stopoutput() {

  }

  // gets IR sensor output as a boolean
  public boolean IsDetected() {
    if (Robot.isSimulation()) {
      return (m_OutputSim.getGamePiecesAmount() > 0);
    }
    return m_coraldetect.get();
  }


  @Override
  public void close() {
    m_OutputMotor.close();
    // m_encoder.close();
  }
}
