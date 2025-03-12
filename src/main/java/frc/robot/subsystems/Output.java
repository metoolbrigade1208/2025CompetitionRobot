// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;



public class Output extends SubsystemBase implements AutoCloseable {
  // singleton instance
  private static Output instance;

  public static Output getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Instance not created yet");
    }
    return instance;
  }

  private OutputSimulation m_OutputSim;
  private LocationService m_LocationService;
  private SwerveDrive m_Drive;
  // declares the motor on the output device
  private final SparkMax m_OutputMotor =
      new SparkMax(Constants.OutputConstants.kOutputMotorPort, MotorType.kBrushless);

  // declares the moter gearbox
  private DCMotor m_OutputGearbox = DCMotor.getNEO(1);

  // declares IR Sensor
  private final DigitalInput m_coraldetect =
      new DigitalInput(Constants.OutputConstants.kIRsensorport);


  // declares the controller for the output motor
  // private final SparkClosedLoopController m_Outputcontroller =
  // m_OutputMotor.getClosedLoopController();
  // private final RelativeEncoder m_OutputEncoder = m_OutputMotor.getAlternateEncoder();
  public Output() {

    // Configure the output motor
    SparkMaxConfig OutputMotorConfig = new SparkMaxConfig();
    OutputMotorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    OutputMotorConfig.closedLoop.pidf(Constants.OutputConstants.kOutputKp,
        Constants.OutputConstants.kOutputKi, Constants.OutputConstants.kOutputKd,
        1.0 / Constants.OutputConstants.kOutputKv, ClosedLoopSlot.kSlot0);

    // Initialize the blank final fields
    m_OutputSim = new OutputSimulation(m_OutputGearbox);
    m_LocationService = LocationService.getInstance();
    m_Drive = SwerveSubsystem.getInstance().getSwerveDrive();

    if (instance != null) {
      throw new IllegalStateException("Cannot create new instance of singleton class");
    }
    instance = this;
  }

  // Creates Output Simulation

  public class OutputSimulation {
    private boolean havePiece = false;

    public OutputSimulation(DCMotor OutputGearbox) {
      m_OutputGearbox = OutputGearbox;
    }

    public boolean getGamePieceInBot() {
      return havePiece;
    }

    public void periodic() {
      if (m_LocationService.inSourceRegion()) {
        havePiece = true;
      }
    }

    // function to addprojectile, called by ejectCoralCommand in sim mode
    public void ejectCoralSim() {
      m_OutputSim.addGamePieceProjectile(m_Drive.getMapleSimDrive().get(), 37.5);
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

  }

  public void periodic() {
    // if in simulation, call the Sim's periodic function
    if (Robot.isSimulation()) {
      m_OutputSim.periodic();
    }
    System.out.print("output status: ");
    System.out.println(IsDetected());
  }

  // runs motor
  public void runmotor() {
    m_OutputMotor.set(-.1);
  }

  // stops motor
  public void stopmotor() {
    m_OutputMotor.set(0.0);
  }

  // grips onto coral after detection
  public void runmotoronce() {
    SparkClosedLoopController outputController = m_OutputMotor.getClosedLoopController();
    outputController.setReference(2, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }


  // runs motor again to grip coral
  public Command gripCoralCommand() {
    return new FunctionalCommand(() -> {
    }, this::runmotor, (x) -> runmotoronce(), this::IsDetected, this);

  }

  // shoots coral onto reef
  public Command ejectCoralCommand() {
    return new FunctionalCommand(() -> {
    }, this::runmotor, (x) -> runmotoronce(), () -> !IsDetected(), this);
  }

  public Command runOutputMotor() {
    return new StartEndCommand(this::runmotor, this::stopmotor, this);
  }

  public Trigger clearOutput() {
    return new Trigger(this::IsDetected);
  }

  // gets IR sensor output as a boolean
  public boolean IsDetected() {
    if (Robot.isSimulation()) {
      return (m_OutputSim.getGamePieceInBot());
    }
    return m_coraldetect.get();
  }


  @Override
  public void close() {
    m_OutputMotor.close();
    // m_encoder.close();
  }
}

