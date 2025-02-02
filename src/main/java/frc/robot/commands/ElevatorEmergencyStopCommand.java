// EmergencyStopElevatorCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorEmergencyStopCommand extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorEmergencyStopCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}