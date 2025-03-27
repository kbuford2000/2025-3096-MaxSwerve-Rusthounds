package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.AutonomousRoutines;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.autonomous.*;


import java.util.List;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    // Autonomous routines
    private final AutonomousRoutines autonomousRoutines = new AutonomousRoutines(
        m_robotDrive,
        elevatorSubsystem,
        armSubsystem
        );

    // Driver's controller (Commonized)
    private final CommandXboxController driverController =
            new CommandXboxController(OIConstants.kDriverControllerPort);

    /** Constructor: Initializes and binds controls */
    public RobotContainer() {
        configureButtonBindings();

        // Default driving command using lambda for cleaner code
        m_robotDrive.setDefaultCommand(
                new RunCommand(() -> m_robotDrive.drive(
                        -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                        true),
                        m_robotDrive));
    }

    /** Define button mappings for controller input */
    private void configureButtonBindings() {
        // Mode selection triggers
        //Trigger elevatorMode = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
        //Trigger armMode = new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);

        
        Trigger armDown = new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);

        // Drive mode toggle
        //driverController.rightBumper().whileTrue(new RunCommand(m_robotDrive::setX, m_robotDrive));

       
        // Elevator position controls without Trigger 
        (driverController.povUp()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_4));
        (driverController.povRight()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_3));
        (driverController.povDown()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_2));
        (driverController.povLeft()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_1));
        //(driverController.back()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_PickupCoral));
        (driverController.start()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_SwingArmWithCoral));
  
        // Manual elevator adjustments
        //driverController.leftTrigger(0.5).onTrue(new ElevatorManualCommand(elevatorSubsystem, () -> -0.3));  // Down
        //driverController.leftBumper().whileTrue(new ElevatorManualCommand(elevatorSubsystem, () -> 0.3));    // Up

       //Elevator and Arm Coordinated Movement

       (driverController.back()).onTrue(
    new SequentialCommandGroup(
       // Then execute the "a" button functionality
         new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_3),
         // First execute the "back" button functionality
       new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_PickupCoral).withTimeout(3)
    )
);

(driverController.start()).onTrue(
        new SequentialCommandGroup(
           // Then execute the "a" button functionality
        new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_SwingArmWithCoral).withTimeout(3),
             // First execute the "back" button functionality

      new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_1)
      
           )
    );
        // Arm position controls without Trigger(Comment out until positions confirmed)rmSetPositionCommand(armSubsystem, ArmSubsystem.HOME_POSITION));
        (driverController.x()).onTrue(new ArmSetPositionCommand(armSubsystem, ArmSubsystem.HOME_POSITION));
        (driverController.y()).onTrue(new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_1));
        (driverController.b()).onTrue(new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_2));
        (driverController.a()).onTrue(new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_3));
        
        // Manual arm adjustments
        //driverController.rightTrigger(0.5).onTrue(new ArmManualCommand(armSubsystem, () -> 0.3));  // Down
        //driverController.leftBumper().whileTrue(new ArmManualCommand(armSubsystem, () -> 0.3));    // Up
        //driverController.rightBumper().whileTrue(new ArmManualCommand(armSubsystem, () -> -0.3));  // Up

        // Universal controls
       // driverController.start().onTrue(
                //new ElevatorHomeCommand(elevatorSubsystem)
                       // .alongWith(new ArmHomeCommand(armSubsystem)));

      //  driverController.back().onTrue(
              //  new ElevatorEmergencyStopCommand(elevatorSubsystem)
                      //  .alongWith(new ArmEmergencyStopCommand(armSubsystem)));
    }

    /** Provides the autonomous command for the robot */
       public Command getAutonomousCommand() {
        // Reset robot pose based on selected starting position
        autonomousRoutines.resetRobotPose();
        
        // Return the selected autonomous command
        return autonomousRoutines.getSelectedAutonomousCommand();
    }

    /**
     * Returns the currently selected starting position for autonomous
     */
   
}

        //(driverController.x()).onTrue(new A