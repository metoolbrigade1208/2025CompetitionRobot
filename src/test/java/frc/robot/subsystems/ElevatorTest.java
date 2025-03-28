package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.Elevator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorTest {
    private Elevator elevator;
    private ElevatorSim elevatorSim;
    private DIOSim limitSwitchSim;

    @BeforeEach
    void setup() {
        HAL.initialize(500, 0);
        elevator = new Elevator();
        elevatorSim = new ElevatorSim(
            elevator.m_elevatorGearbox,
            Constants.elevator.kElevatorGearing,
            Constants.elevator.kCarriageMass,
            Constants.elevator.kElevatorDrumRadius,
            Constants.elevator.kMinElevatorHeightMeters,
            Constants.elevator.kMaxElevatorHeightMeters,
            true,
            Constants.elevator.kMinElevatorHeightMeters
        );
        limitSwitchSim = new DIOSim(elevator.input);
    }

    @AfterEach
    void shutdown() throws Exception {
        elevator.close();
        CommandScheduler.getInstance().cancelAll();
    }

    @Test
    void testInitialPosition() {
        assertEquals(0, elevator.m_encoder.getPosition(), 0.01);
    }

    @Test
    void testReachGoal() {
        double testHeight = Units.inchesToMeters(10);
        elevator.reachGoal(testHeight);
        assertEquals(
            (testHeight - Constants.LEVEL_1) / Constants.elevator.kPositionConversionFactor,
            elevator.currentGoalRotations,
            0.01
        );
    }

    @Test
    void testLimitSwitch() {
        // Test limit switch pressed
        limitSwitchSim.setValue(true);
        elevator.periodic();
        assertEquals(0, elevator.m_encoder.getPosition(), 0.01);

        // Test limit switch released
        limitSwitchSim.setValue(false);
        elevator.periodic();
        assertNotEquals(0, elevator.m_encoder.getPosition());
    }

    @Test
    void testLevelCommands() {
        // Test level 1 command
        elevator.elevatorLevel1Command().schedule();
        CommandScheduler.getInstance().run();
        assertEquals(
            (Constants.LEVEL_1 - Constants.LEVEL_1) / Constants.elevator.kPositionConversionFactor,
            elevator.currentGoalRotations,
            0.01
        );

        // Test level 2 command
        elevator.elevatorLevel2Command().schedule();
        CommandScheduler.getInstance().run();
        assertEquals(
            (Constants.LEVEL_2 - Constants.LEVEL_1) / Constants.elevator.kPositionConversionFactor,
            elevator.currentGoalRotations,
            0.01
        );
    }

    @Test
    void testStopCommand() {
        elevator.elevatorUp().schedule();
        CommandScheduler.getInstance().run();
        assertNotEquals(0, elevator.m_motor.get());

        elevator.elevatorStop().schedule();
        CommandScheduler.getInstance().run();
        assertEquals(0, elevator.m_motor.get(), 0.01);
    }
}
