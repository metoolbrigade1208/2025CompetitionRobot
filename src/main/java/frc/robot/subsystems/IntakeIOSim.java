package frc.robot.subsystems;

import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.util.Units;

public class IntakeIOSim implements IntakeIO {
    private IntakeSimulation intakeSimulation;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        this.intakeSimulation = new IntakeSimulation("Coral", driveTrain,
                new Rectangle(Units.inchesToMeters(30), Units.inchesToMeters(8)), 1);
    }

    // Intake Functions\\
    @Override // Defined by IntakeIO
    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation.startIntake();
        else
            intakeSimulation.stopIntake();
    }

    @Override
    public boolean isCoralInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0;
    }
}
