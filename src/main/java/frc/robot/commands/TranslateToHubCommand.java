package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class TranslateToHubCommand extends Command{

    private final PIDController xController;

    private final double positionTolerance = 0.05; // Meters (5 cm)
    private double targetOffsetX; 
    private double targetSetpointX; 
    private double targetX;
    private double targetErrorX;
    private double pidOutX; 
    private double xSpeed; 
    private double maxSpeedMultiplier;
    
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PIDController thetaController;

    private final double angleTolerance = 1;    // Degree 
    private double targetSetpointTheta; 
    private double targetTheta;
    private double pidOutTheta;
    private double thetaSpeed;
    
    public TranslateToHubCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(driveSubsystem, visionSubsystem, 0.0, 0.51);
        this.maxSpeedMultiplier = 1;
        
        addRequirements(driveSubsystem, visionSubsystem);
    }

    public TranslateToHubCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, 
        double targetOffsetTheta, double targetOffsetX) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetX = targetOffsetX;
        this.maxSpeedMultiplier = 1;

        this.xController = new PIDController(.90, 0, 1e-4);
        xController.setTolerance(positionTolerance);

        this.thetaController = new PIDController(1, 0.0, 0.0);
        thetaController.enableContinuousInput(-180,180);
        thetaController.setTolerance(angleTolerance);

        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {

        xController.reset();
        targetErrorX = 0.0;
        pidOutX = 0.0;
        targetSetpointX = 0.0;
        xSpeed = 0.0;

        thetaController.reset();
        pidOutTheta = 0.0;
        targetSetpointTheta = -10;
        thetaSpeed = 0.0;

        // SmartDashboard.putNumber("TS Theta", targetSetpointTheta);
        // SmartDashboard.putNumber("Target Theta", targetTheta);
        // SmartDashboard.putNumber("Target Error", targetErrorTheta);
        // SmartDashboard.putNumber("PID Output", pidOutTheta);
        SmartDashboard.putNumber("Theta Speed", thetaSpeed);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {

            targetSetpointX = targetOffsetX;
            targetX = VisionSubsystem.getTarget_x();
            pidOutX = xController.calculate(targetX, targetSetpointX);
            targetErrorX = xController.getError();

            xSpeed = Math.max(-0.35*maxSpeedMultiplier, Math.min(0.35*maxSpeedMultiplier, pidOutX));

            targetTheta = VisionSubsystem.getTarget_rawYaw();
            pidOutTheta = thetaController.calculate(targetTheta, targetSetpointTheta);

            thetaSpeed = Math.max(-.2, Math.min(.2, pidOutTheta)); // degrees/sec


            driveSubsystem.drive(-xSpeed, 0, thetaSpeed, false, false);    // try fieldrelative true?

        } else {
            pidOutX = 0.0;
            xSpeed = 0.0;
            targetErrorX = 0.0;
            pidOutTheta = 0.0;
            thetaSpeed = 0.0;
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }

        // SmartDashboard.putNumber("TS Theta", targetSetpointTheta);
        // SmartDashboard.putNumber("Target Theta", targetTheta);
        // SmartDashboard.putNumber("Target Error", targetErrorTheta);
        // SmartDashboard.putNumber("PID Output", pidOutTheta);
        SmartDashboard.putNumber("Theta Speed", thetaSpeed);
    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasTarget() || (thetaController.atSetpoint() && xController.atSetpoint())) {
            return true; 
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }

}