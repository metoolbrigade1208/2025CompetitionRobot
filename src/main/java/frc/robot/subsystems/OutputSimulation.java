
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

public class OutputSimulation {

    // Define the estimateMOI method
    public static double estimateMOI(int outputGearbox, int IRsensorport) {
        // Add the logic to estimate MOI
            return 0.0; // Placeholder return value
        }

    private final OutputSimulation m_OutputSim;

    public OutputSimulation(Object m_OutputGearbox) {
        m_OutputSim = new OutputSimulation(m_OutputGearbox,
                Constants.OutputConstants.kOutputMotorPort,
                OutputSimulation.estimateMOI(Constants.OutputConstants.kOutputGearbox,
                        Constants.OutputConstants.kIRsensorport),
                true, Units.degreesToRadians(0), Constants.IntakeConstants.kArmEncoderDistPerPulse,
                0.0 // Add noise with a std-dev of 1 tick
        );
    }
    
    // Define the constructor with the required parameters
    public OutputSimulation(Object m_OutputGearbox,
            int motorPort,
            double moi, 
            boolean flag, 
            double radians, 
            double encoderDistPerPulse, 
            double noise) {
        // Initialize the fields or add the logic for the constructor
    }

}

}

}
