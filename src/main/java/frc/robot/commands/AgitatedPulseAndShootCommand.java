package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AgitatedPulseAndShootCommand extends Command{

    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private HoodSubsystem hoodSubsystem;
    private double shootSpeed;
    private boolean stow;

    private final double pulseDutyCycle = 0.6;
    private final double pulseDuration = 0.5;
    private final double restDuration = 0.15;

    private final double intakeHighPos = 10;
    private final double intakeLowPos = 130;

    private final Timer timer = new Timer();

    private boolean isPulsing = true;
    private double lastSpeed = Double.NaN;

    private boolean goingHigh = true;
    private boolean wasAtTargetPos = false;
    private boolean isFar;

    
    public AgitatedPulseAndShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double shootSpeed, 
        boolean stowIntake, HoodSubsystem hoodSubsystem, boolean isFar) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.shootSpeed = shootSpeed;
        this.stow = stowIntake;
        this.isFar = isFar;
        
        addRequirements(shooterSubsystem, intakeSubsystem, hoodSubsystem);
    }

    @Override
    public void initialize() {
        
        timer.reset();
        timer.start();

        isPulsing = true;

        goingHigh = true;
        wasAtTargetPos = false;
        intakeSubsystem.setIntakePosition(intakeHighPos);

        intakeSubsystem.setIndexer(pulseDutyCycle);
        shooterSubsystem.setShooterVelocity(shootSpeed);

        if (isFar) {
            hoodSubsystem.setHoodPosition(3.4);
        }
    }

    @Override
    public void execute() {
        double elapsedTime = timer.get();
        double speed;

        boolean atTargetPos = Math.abs(
            (intakeSubsystem.getIntakePos()) - intakeSubsystem.getTargetPosition()) < 30;

        shooterSubsystem.kickFuel(true);


        if (atTargetPos && !wasAtTargetPos) {
            goingHigh = !goingHigh;

            if (goingHigh){
                intakeSubsystem.setIntakePosition(intakeHighPos);
            } else {
                intakeSubsystem.setIntakePosition(intakeLowPos);
            }
        }

        wasAtTargetPos = atTargetPos;
        
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
        shooterSubsystem.stopShooterSystem();
        
        intakeSubsystem.setIndexer(0.0);
        timer.stop();

        hoodSubsystem.setHoodPosition(0.00);

        if (stow) {
            intakeSubsystem.setIntakePosition(10);
        } else {
            intakeSubsystem.setIntakePosition(100);
        }
    }

}
