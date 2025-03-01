
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
import org.ironmaple.simulation.gamepieces.GamePieceProjectile; // Ensure this is the correct import
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.SimulatedArena;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class OutputSimulation {

    // Define the estimateMOI method
    public static double estimateMOI(int outputGearbox, int IRsensorport) {
        // Add the logic to estimate MOI
        return 0.0; // Placeholder return value
    }

    private final OutputSimulation m_OutputSim;
    private final SwerveDriveSimulation m_DriveSimulation;

    public OutputSimulation(Object m_OutputGearbox) {

    }

    public void startOutput() {

    }

    public void stopOutput() {

    }

    public void addGamePieceProjectile(SwerveDriveSimulation driveSimulation, double height) {
        // Level 3 projectile scoring simulation
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


    // Define the constructor with the required parameters
    public OutputSimulation(Object m_OutputGearbox, int motorPort, double moi, boolean flag,
            double radians, double encoderDistPerPulse, double noise) {
        // Initialize the fields or add the logic for the constructor
    }

    public void runmotorSim() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runmotorSim'");
    }

    public void runmotorOnceSim() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runmotorOnceSim'");
    }

}
