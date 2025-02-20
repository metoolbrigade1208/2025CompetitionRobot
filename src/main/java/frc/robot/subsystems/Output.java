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

import com.revrobotics.RelativeEncoder;

public class Output extends SubsystemBase implements AutoCloseable {

  // declares the motor on the output device
  private final SparkMax m_OutputMotor =
      new SparkMax(Constants.elevator.kMotorPort, MotorType.kBrushless);

  // declares the moter gearbox
  private final DCMotor m_OutputGearbox = DCMotor.getNEO(1);

  // declares IR Sensor
  private final DigitalInput m_coraldetect =
      new DigitalInput(Constants.IntakeConstants.kIRsensorport);
  private final DigitalInput input = new DigitalInput(1);
  DigitalInput input2 = new DigitalInput(1);

  // declares the controller for the output motor
  private final SparkClosedLoopController m_Outputcontroller =
      m_OutputMotor.getClosedLoopController();
  private final RelativeEncoder m_OutputEncoder = m_OutputMotor.getAlternateEncoder();


  public void periodic() {

  public Output(SwerveDrive drivetrain) {
    // m_encoder.setDistancePerPulse(Constants.IntakeConstants.kArmEncoderDistPerPulse);
    m_OutputSim = IntakeSimulation.OverTheBumperIntake("Coral", drivetrain.getMapleSimDrive().get(),
        Inches.of(28), Inches.of(8), IntakeSimulation.IntakeSide.FRONT, 1);
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  /** Update the simulation model. */


  }

  }

  public Command isCoralDetected() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return input.get();
      }
    };
  }



  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }
}
