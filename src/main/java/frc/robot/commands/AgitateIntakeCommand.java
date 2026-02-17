package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class AgitateIntakeCommand extends Command{

    private IntakeSubsystem intakeSubsystem;

    private final double highPos = 2 * (IntakeConstants.kIntakeFinalPosition - 12) / 3;
    private final double lowPos = IntakeConstants.kIntakeFinalPosition - 12;

    private boolean goingHigh = true;
    private boolean wasAtTargetPos = false;

    public AgitateIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        goingHigh = true;
        wasAtTargetPos = false;
        intakeSubsystem.setIntakePosition(highPos);
    }

    @Override
    public void execute() {
        boolean atTargetPos = intakeSubsystem.intakeAtPosition();

        if (atTargetPos && !wasAtTargetPos) {
            goingHigh = !goingHigh;

            if (goingHigh){
                intakeSubsystem.setIntakePosition(highPos);
            } else {
                intakeSubsystem.setIntakePosition(lowPos);
            }
        }

        wasAtTargetPos = atTargetPos;
    }

    @Override
    public void end(boolean canceled) {
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}