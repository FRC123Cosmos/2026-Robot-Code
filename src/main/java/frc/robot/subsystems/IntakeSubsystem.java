package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax kIndexerMax;
    private final SparkMax kIntakeMax;
    private final SparkMax kIntakeRollerMax;

    private final SparkClosedLoopController kIndexerController;
    private final SparkClosedLoopController kIntakeController;
    private final SparkClosedLoopController kIntakeRollerController;

    private final AbsoluteEncoder intakeEncoder;

    private double targetPosition = 5;



    public IntakeSubsystem(){
        kIndexerMax = new SparkMax(IntakeConstants.kIndexerCANID, MotorType.kBrushless);
        kIntakeMax = new SparkMax(IntakeConstants.kIntakeCANID, MotorType.kBrushless);
        kIntakeRollerMax = new SparkMax(IntakeConstants.kIntakeRollerCANID, MotorType.kBrushless);

        kIndexerMax.configure(Configs.IntakeConfigs.indexerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kIntakeMax.configure(Configs.IntakeConfigs.intakeConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kIntakeRollerMax.configure(Configs.IntakeConfigs.intakeRollerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        intakeEncoder = kIntakeMax.getAbsoluteEncoder();


        kIndexerController = kIndexerMax.getClosedLoopController();
        kIntakeController = kIntakeMax.getClosedLoopController();
        kIntakeRollerController = kIntakeRollerMax.getClosedLoopController();
    }

    public void setIndexer(double setPoint){
        kIndexerController.setSetpoint(setPoint, ControlType.kDutyCycle);
    }

    public void setIntakeRoller(double setPoint){
        kIntakeRollerController.setSetpoint(setPoint, ControlType.kDutyCycle);
    }

    public boolean intakeAtPosition(){
        return Math.abs((intakeEncoder.getPosition()) - targetPosition) < IntakeConstants.kIntakePositionDeadband;
    }

    public double getIntakePos(){
        return intakeEncoder.getPosition();
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public void setIntakePosition(double position){
        targetPosition = position;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Target Intake Pos", targetPosition);
        SmartDashboard.putNumber("Intake Pos", getIntakePos());

        kIntakeController.setSetpoint(targetPosition, ControlType.kPosition);
    }

}
