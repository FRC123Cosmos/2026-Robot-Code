package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class UnjamHopperCommand extends Command{
    
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    
    public UnjamHopperCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        LedSubsystem.blinkYellowFast();
    }

    @Override
    public void execute() {
        intakeSubsystem.setIndexer(-0.15);
        shooterSubsystem.setKickerRollers(-0.1);
    }

    @Override
    public void end(boolean canceled) {
        intakeSubsystem.setIndexer(0.0);
        shooterSubsystem.setKickerRollers(0.0);
    }

}
