package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PulseIndexerCommand extends Command{

    private IntakeSubsystem intakeSubsystem;

    private final double pulseDutyCycle = 0.8;
    private final double pulseDuration = 0.4;
    private final double restDuration = 0.1;

    private final Timer timer = new Timer();

    private boolean isPulsing = true;
    private double lastSpeed = Double.NaN;
    
    public PulseIndexerCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        isPulsing = true;
        intakeSubsystem.setIndexer(pulseDutyCycle);
    }

    @Override
    public void execute() {
        double elapsedTime = timer.get();
        double speed;

        if (isPulsing && elapsedTime >= pulseDuration) {
            speed = 0.0;
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
        intakeSubsystem.setIndexer(0.0);
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}