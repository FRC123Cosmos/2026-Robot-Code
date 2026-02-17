package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PulseAndShootCommand extends Command{

    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double shootSpeed;
    private boolean stow;

    private final double pulseDutyCycle = 0.8;
    private final double pulseDuration = 0.4;
    private final double restDuration = 0.1;

    private final Timer timer = new Timer();

    private boolean isPulsing = true;
    private double lastSpeed = Double.NaN;

    private boolean agitateHigh = false;
    private boolean wasAtPosition = false;
    private double IntakeTargetPos = IntakeConstants.kIntakeFinalPosition - 12;

    
    public PulseAndShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double shootSpeed, boolean stowIntake) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shootSpeed = shootSpeed;
        this.stow = stowIntake;
        
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        
        timer.reset();
        timer.start();

        isPulsing = true;
        intakeSubsystem.setIntakeRoller(IntakeConstants.intakeRollerAgitationSpeed);
        intakeSubsystem.setIndexer(pulseDutyCycle);
        shooterSubsystem.setShooterVelocity(shootSpeed);
    }

    @Override
    public void execute() {
        double elapsedTime = timer.get();
        double speed;

        shooterSubsystem.kickFuel(true);
        
        boolean atPos = intakeSubsystem.intakeAtPosition();

        if (atPos && !wasAtPosition) {
            IntakeTargetPos = getNextAgitatePosition();
            intakeSubsystem.setIntakePosition(IntakeTargetPos);
        }

        wasAtPosition = atPos;

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
        intakeSubsystem.setIntakeRoller(0.0);
        timer.stop();
        if (stow) {
            intakeSubsystem.setIntakePosition(5);
        }
    }

    private double getNextAgitatePosition() {
        agitateHigh = !agitateHigh;

        return agitateHigh
            ? IntakeConstants.kIntakeFinalPosition - 12
            : ((IntakeConstants.kIntakeFinalPosition - 12) / 2) - 5;
    }
}