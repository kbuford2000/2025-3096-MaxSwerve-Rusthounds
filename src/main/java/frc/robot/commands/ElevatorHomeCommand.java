// HomeElevatorCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorHomeCommand extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorHomeCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(ElevatorSubsystem.HOME_POSITION);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stop();
        }
    }
}