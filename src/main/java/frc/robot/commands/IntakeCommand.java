package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{

    private IntakeSubsystem intakeSubsystem;
    private double targetPosition;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.targetPosition = IntakeConstants.kIntakeFinalPosition - 12;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakePosition(targetPosition);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeRoller(IntakeConstants.intakeSpeed);
        intakeSubsystem.setIndexer(IntakeConstants.indexerSpeed);

    }

    @Override
    public void end(boolean canceled) {
        intakeSubsystem.setIntakeRoller(0.0);
        intakeSubsystem.setIndexer(0.0);
    }
}