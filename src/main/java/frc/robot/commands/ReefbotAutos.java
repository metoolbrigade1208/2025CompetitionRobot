package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.LocationService;
import frc.robot.subsystems.Output;
import frc.robot.subsystems.Elevator.Elevator;

public class ReefbotAutos {
    public static Command AutoOutput() {
        LocationService locationService = LocationService.getInstance();
        Output output = Output.getInstance();
        Elevator elevator = Elevator.getInstance();
        /*
         * Assume driving automatedly, then within 3ft, raise elevator to selected level once
         * elevator reaches its set position and within 2in of tagpose, run ejectCoralCommand wait
         * for half a second lower elevator to lvl1 when clearOutput
         */


        Command elevatorCommand =
                Commands.defer(elevator::elevatorleveldataCommand, Set.of(elevator));

        return new WaitUntilCommand(locationService.nearAutoPose()).andThen(elevatorCommand)
                .andThen(new WaitUntilCommand(locationService.atAutoPose()))
                .andThen(new WaitUntilCommand(elevator.elevatorAtLevel))
                .andThen(output.ejectCoralCommand())
                .andThen(new WaitUntilCommand(() -> !output.IsDetected()))
                .andThen(new WaitCommand(0.5)).andThen(elevator::elevatorLevel1Command);
    }
}
