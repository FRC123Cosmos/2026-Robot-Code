package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AgitatedPulseAndShootCommand extends Command{

    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private HoodSubsystem hoodSubsystem;
    private double shootSpeed;
    private boolean stow;

    private final double pulseDutyCycle = 0.725; // .725
    private final double pulseDuration = 0.5; // .5
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
            LedSubsystem.blinkAllianceSolidSlow();
            hoodSubsystem.setHoodPosition(3.35); // 3.4
        } else {
            hoodSubsystem.setHoodPosition(-0.11);
            LedSubsystem.blinkAllianceSolidFast();
        }
    }

    @Override
    public void execute() {
        double elapsedTime = timer.get();
        double speed;

        boolean atTargetPos = Math.abs(
            (intakeSubsystem.getIntakePos()) - intakeSubsystem.getTargetPosition()) < 30;

        shooterSubsystem.kickFuelRPM(true);

        if (atTargetPos && !wasAtTargetPos) {
            goingHigh = !goingHigh;

            if (goingHigh){
                intakeSubsystem.setIntakePosition(intakeHighPos);
            } else {
                intakeSubsystem.setIntakePosition(intakeLowPos);
            }
        }

        wasAtTargetPos = atTargetPos;
        intakeSubsystem.setIntakeRoller(0.65);
        
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
        intakeSubsystem.setIntakeRoller(0.0);
        intakeSubsystem.setIndexer(0.0);
        timer.stop();

        LedSubsystem.setAllianceSolid();
        // hoodSubsystem.setHoodPosition(-0.1);

        if (stow) {
            intakeSubsystem.setIntakePosition(10);
        } else {
            intakeSubsystem.setIntakePosition(100);
        }
    }

}
