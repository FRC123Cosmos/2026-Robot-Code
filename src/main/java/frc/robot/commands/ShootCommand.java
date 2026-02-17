package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double shootSpeed;
    private boolean stow;

    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double shootSpeed, boolean stowIntake) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shootSpeed = shootSpeed;
        this.stow = stowIntake;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        // SmartDashboard.putNumber("Input ShooterRPM", 0);
        shooterSubsystem.setShooterVelocity(shootSpeed);
    }

    @Override
    public void execute() {
        // shootSpeed = SmartDashboard.getNumber("Input ShooterRPM", 0.0);
        // shootSpeed = shooterSubsystem.inputShootVelocity;

        intakeSubsystem.setIndexer(IntakeConstants.indexerSpeed);
        shooterSubsystem.kickFuel(true);
        // intakeSubsystem.agitateFuel(true);
    }

    @Override
    public void end(boolean canceled) {
        shooterSubsystem.stopShooterSystem();
        intakeSubsystem.setIndexer(0.0);
        // intakeSubsystem.setIntakeRoller(0.0);
        // if (stow) {
        //     intakeSubsystem.setIntakePosition(5);
        // }
    }
}