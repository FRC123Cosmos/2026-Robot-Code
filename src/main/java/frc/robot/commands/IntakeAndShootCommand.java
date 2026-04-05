package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeAndShootCommand extends Command{
    
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private HoodSubsystem hoodSubsystem;

    private final double pulseDutyCycle = 0.55;
    private final double pulseDuration = 0.45;
    private final double restDuration = 0.125;

    private final Timer timer = new Timer();

    private boolean isPulsing = true;
    private double lastSpeed = Double.NaN;

    private final double highPos = 60;
    private final double lowPos = 140;

    private boolean goingHigh = true;
    private boolean wasAtTargetPos = false;

    public IntakeAndShootCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(intakeSubsystem, shooterSubsystem, hoodSubsystem);
    }

    @Override
    public void initialize() {
        LedSubsystem.blinkPurpleFast();
        timer.reset();
        timer.start();

        goingHigh = true;
        wasAtTargetPos = false;
        intakeSubsystem.setIntakePosition(highPos);

        isPulsing = true;
        intakeSubsystem.setIndexer(pulseDutyCycle);
        shooterSubsystem.setShooterVelocity(ShooterConstants.kShooterPassVelocityRPM);

        hoodSubsystem.setHoodPosition(3.0);
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

        shooterSubsystem.kickFuelRPM(true);

        intakeSubsystem.setIntakeRoller(IntakeConstants.intakeSpeed);

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
        LedSubsystem.setAllianceSolid();
        timer.stop();

        shooterSubsystem.stopShooterSystem();

        intakeSubsystem.setIntakePosition(25);

        intakeSubsystem.setIntakeRoller(0.0);
        intakeSubsystem.setIndexer(0.0);
    }

}
