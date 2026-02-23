package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class OscillateIntakeCommand extends Command{

    private IntakeSubsystem intakeSubsystem;

    private final double highPos = 60;
    private final double lowPos = 140;

    private boolean goingHigh = true;
    private boolean wasAtTargetPos = false;

    public OscillateIntakeCommand(IntakeSubsystem intakeSubsystem) {
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
        boolean atTargetPos = Math.abs(
            (intakeSubsystem.getIntakePos()) - intakeSubsystem.getTargetPosition()) < 40;

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
        intakeSubsystem.setIntakePosition(100);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}