package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class OscillatedPulseAndIntakeCommand extends Command{
    
    private IntakeSubsystem intakeSubsystem;
    private double targetPosition = IntakeConstants.kIntakeFinalPosition - 7;

    private final double pulseDutyCycle = 0.5;
    private final double pulseDuration = 0.4;
    private final double restDuration = 0.15;

    private final Timer timer = new Timer();

    private boolean isPulsing = true;
    private double lastSpeed = Double.NaN;

    private final double highPos = 60;
    private final double lowPos = 140;

    private boolean goingHigh = true;
    private boolean wasAtTargetPos = false;

    public OscillatedPulseAndIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakePosition(targetPosition);
        timer.reset();
        timer.start();

        goingHigh = true;
        wasAtTargetPos = false;
        intakeSubsystem.setIntakePosition(highPos);

        isPulsing = true;
        intakeSubsystem.setIndexer(pulseDutyCycle);
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


        intakeSubsystem.setIntakeRoller(IntakeConstants.intakeSpeed);

        double elapsedTime = timer.get();
        double speed;

        if (isPulsing && elapsedTime >= pulseDuration) {
            speed = -0.15;
            isPulsing = false;
            timer.reset();
        } else if (!isPulsing && elapsedTime >= restDuration) {
            speed = pulseDutyCycle;
            isPulsing = true;
            timer.reset();
        } else {
            return;
        }

        if (speed != lastSpeed) {
            intakeSubsystem.setIndexer(speed);
            lastSpeed = speed;
        }
    }

    @Override
    public void end(boolean canceled) {
        timer.stop();

        intakeSubsystem.setIntakePosition(100);

        intakeSubsystem.setIntakeRoller(0.0);
        intakeSubsystem.setIndexer(0.0);
    }

}
