package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class UnjamHopperCommand extends Command{
    
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private boolean isIntakeExtending;
    
    public UnjamHopperCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, boolean isIntakeExtending) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.isIntakeExtending = isIntakeExtending;
        
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        if(isIntakeExtending){
            intakeSubsystem.setIntakePosition(IntakeConstants.kIntakeFinalPosition - 7);
        }
        LedSubsystem.blinkOrangeFast();
    }

    @Override
    public void execute() {
        intakeSubsystem.setIndexer(-0.15);
        shooterSubsystem.setKickerRPM(-550); // - 0.1
    }

    @Override
    public void end(boolean canceled) {
        intakeSubsystem.setIndexer(0.0);
        shooterSubsystem.stopKicker();
        LedSubsystem.setAllianceSolid();
    }

}
