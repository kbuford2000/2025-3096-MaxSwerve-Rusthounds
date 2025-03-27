// ResetElevatorEncoderCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorResetEncoderCommand extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorResetEncoderCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
