package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
//import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax masterMotor;
    private final SparkMax followerMotor;
    private final SparkMaxConfig masterMotorConfig;
    private final SparkMaxConfig followerMotorConfig;
    private final RelativeEncoder masterEncoder;
    private final RelativeEncoder followerEncoder;

    private final PIDController pidController;

    // Define elevator positions in encoder counts
    private static final double MAX_HEIGHT = 50.0; // Adjust based on your elevator
    private static final double MIN_HEIGHT = 0.0;

    public static final double HOME_POSITION = 0.0;
    public static final double POSITION_1 = 7.0;
    public static final double POSITION_PickupCoral = 12.4;
    public static final double POSITION_2 = 15.0;
    public static final double POSITION_SwingArmWithCoral = 16.9;
    public static final double POSITION_3 = 25.0;
    public static final double POSITION_4 = 45.0;

    // PID constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Motor constants

    private static final int MASTER_MOTOR_ID = 21;
    private static final int FOLLOWER_MOTOR_ID = 22;
    private static final double MAX_OUTPUT = 0.5;
    private static final double MIN_OUTPUT = -0.5;

    private static final double RAMP_RATE = 0.25;

    private double setPoint = HOME_POSITION;

    public ElevatorSubsystem() {
        // Initialize master motor
        masterMotor = new SparkMax(MASTER_MOTOR_ID, MotorType.kBrushless);
        masterMotorConfig = new SparkMaxConfig();
        masterMotorConfig.idleMode(IdleMode.kBrake);
        // Set motor output limits and current limits
        masterMotorConfig.smartCurrentLimit(40);
        // set ramp rate in seconds from neutral to full output
        masterMotorConfig.closedLoopRampRate(RAMP_RATE);
        masterMotor.configure(masterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        
        // Initialize follower motor
        followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        followerMotorConfig = new SparkMaxConfig();
        followerMotorConfig.idleMode(IdleMode.kBrake);        
        // Set motor output limits and current limits
        followerMotorConfig.smartCurrentLimit(40);
        // set ramp rate in seconds from neutral to full output
        followerMotorConfig.closedLoopRampRate(RAMP_RATE);
        // Configure follower motor to follow master but inverted
        followerMotorConfig.follow(MASTER_MOTOR_ID, true);
        followerMotor.configure(followerMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        

        // Initialize encoder (only need master's encoder since they're mechanically linked)
        masterEncoder = masterMotor.getEncoder();
        masterEncoder.setPosition(0);

        // Initialize follower encoder 
        followerEncoder = followerMotor.getEncoder();
        followerEncoder.setPosition(0);

        // Initialize PID controller
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(0.5);

      

     
    }

    @Override
    public void periodic() {
        // Calculate PID output
        double output = pidController.calculate(masterEncoder.getPosition(), setPoint);
        output = Math.max(MIN_OUTPUT, Math.min(output, MAX_OUTPUT));
        
        masterMotor.set(output);
        // No need to set follower motor as it's following the master

        // Update dashboard with telemetry
        SmartDashboard.putNumber("Elevator Position", masterEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Follower Position", followerEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Target", setPoint);
        SmartDashboard.putNumber("Elevator Output", output);
        SmartDashboard.putNumber("Master Motor Current", masterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Follower Motor Current", followerMotor.getOutputCurrent());
    }

    public void setPosition(double position) {
        setPoint = position;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void resetEncoder() {
        masterEncoder.setPosition(0);
        followerEncoder.setPosition(0);

    }

    public double getCurrentPosition() {
        followerEncoder.getPosition();
        return masterEncoder.getPosition();
    }

    public void checkMotorSync() {
        // Log if motor currents are significantly different, which could indicate mechanical issues
        double currentDifference = Math.abs(masterMotor.getOutputCurrent() - followerMotor.getOutputCurrent());
        if (currentDifference > 10.0) {  // 10 amp threshold, adjust as needed
            System.out.println("Warning: Motor current mismatch detected");
            SmartDashboard.putBoolean("Elevator Motor Sync Warning", true);
        } else {
            SmartDashboard.putBoolean("Elevator Motor Sync Warning", false);
        }
    }

    public void stop() {
        masterMotor.set(0);
        setPoint = getCurrentPosition(); // Maintain current position after stop
    }

    public void setManualSpeed(double speed) {
        // Limit manual speed and check soft limits
        double limitedSpeed = Math.max(-MAX_OUTPUT, Math.min(speed, MAX_OUTPUT));
        
        // Check soft limits
        if ((getCurrentPosition() >= MAX_HEIGHT && limitedSpeed > 0) ||
            (getCurrentPosition() <= MIN_HEIGHT && limitedSpeed < 0)) {
            limitedSpeed = 0;
        }
        
        masterMotor.set(limitedSpeed);
    }

  

    public boolean isAtUpperLimit() {
        return getCurrentPosition() >= MAX_HEIGHT;
    }

    public boolean isAtLowerLimit() {
        return getCurrentPosition() <= MIN_HEIGHT;
    }
}