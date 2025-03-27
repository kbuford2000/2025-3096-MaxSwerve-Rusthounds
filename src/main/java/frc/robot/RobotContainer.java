package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.List;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    // Driver's controller (Commonized)
    private final CommandXboxController driverController =
            new CommandXboxController(OIConstants.kDriverControllerPort);

    /** Constructor: Initializes and binds controls */
    public RobotContainer() {
        configureButtonBindings();

        // Default driving command using lambda for cleaner code
       // m_robotDrive.setDefaultCommand(
               // new RunCommand(() -> m_robotDrive.drive(
                       // -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                       // -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                       // -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                       // true),
                       // m_robotDrive));
    }

    /** Define button mappings for controller input */
    private void configureButtonBindings() {
        // Mode selection triggers
        Trigger elevatorMode = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
        Trigger armMode = new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);

        // Drive mode toggle
        //driverController.rightBumper().whileTrue(new RunCommand(m_robotDrive::setX, m_robotDrive));

        // Elevator position controls (when left trigger is held)
        elevatorMode.and(driverController.povUp()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_4));
        elevatorMode.and(driverController.povRight()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_3));
        elevatorMode.and(driverController.povDown()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_2));
        elevatorMode.and(driverController.povLeft()).onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_1));

        // Manual elevator adjustments
        elevatorMode.and(driverController.rightBumper()).whileTrue(new ElevatorManualCommand(elevatorSubsystem, () -> 0.3));  // Up
        elevatorMode.and(driverController.leftBumper()).whileTrue(new ElevatorManualCommand(elevatorSubsystem, () -> -0.3)); // Down

        // Arm position controls (when right trigger is held)
        armMode.and(driverController.a()).onTrue(new ArmSetPositionCommand(armSubsystem, ArmSubsystem.HOME_POSITION));
        armMode.and(driverController.b()).onTrue(new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_1));
        armMode.and(driverController.x()).onTrue(new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_2));

        // Manual arm adjustments
        armMode.and(driverController.rightBumper()).whileTrue(new ArmManualCommand(armSubsystem, () -> 0.3));  // Forward
        armMode.and(driverController.leftBumper()).whileTrue(new ArmManualCommand(armSubsystem, () -> -0.3));  // Backward

        // Universal controls
        driverController.start().onTrue(
                new ElevatorHomeCommand(elevatorSubsystem)
                        .alongWith(new ArmHomeCommand(armSubsystem)));

        driverController.back().onTrue(
                new ElevatorEmergencyStopCommand(elevatorSubsystem)
                        .alongWith(new ArmEmergencyStopCommand(armSubsystem)));
    }

    /** Provides the autonomous command for the robot */
    public Command getAutonomousCommand() {
        // Configure trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // Example trajectory
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        // PID controllers
        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry before running the trajectory
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}
