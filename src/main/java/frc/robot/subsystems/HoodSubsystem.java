package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase{
    
    private final SparkMax kHoodMax;

    private final SparkClosedLoopController kHoodController;

    private final RelativeEncoder hoodEncoder;

    private double targetPosition;

    public HoodSubsystem(){
        kHoodMax = new SparkMax(HoodConstants.kHoodCANID, MotorType.kBrushless);

        kHoodMax.configure(Configs.HoodConfigs.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodEncoder = kHoodMax.getEncoder();
        hoodEncoder.setPosition(0.0);

        kHoodController = kHoodMax.getClosedLoopController();

        targetPosition = 0;
    }

    public boolean intakeAtPosition(){
        return Math.abs((hoodEncoder.getPosition()) - targetPosition) < HoodConstants.kHoodPositionDeadband;
    }

    public double getHoodPos(){
        return hoodEncoder.getPosition();
    }

    public void setHoodPosition(double position){
        targetPosition = position;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hood Pos", getHoodPos());

        kHoodController.setSetpoint(targetPosition, ControlType.kPosition);
    }

}
