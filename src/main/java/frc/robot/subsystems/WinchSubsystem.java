package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase{
    private final SparkFlex winchFlex;
    private final SparkFlex winchFollowerFlex;

    private final SparkClosedLoopController kWinchController;

    private final RelativeEncoder winchEncoder;

    private double targetPosition; 

    public WinchSubsystem(){

        winchFlex = new SparkFlex(WinchConstants.kWinchCANID, MotorType.kBrushless);
        winchFollowerFlex = new SparkFlex(WinchConstants.kWinchFollowerCANID, MotorType.kBrushless);

        winchFlex.configure(Configs.WinchConfigs.winchConfig, 
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchFollowerFlex.configure(Configs.WinchConfigs.winchFollowerConfig, 
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kWinchController = winchFlex.getClosedLoopController();

        winchEncoder = winchFlex.getEncoder();

        targetPosition = 0; 
    }

    public void setWinchPos(double position) {
        targetPosition = position;
    }

    public double getPos(){
        return winchEncoder.getPosition();
    }

    public boolean atPosition(){
        return Math.abs(getPos() - targetPosition) < WinchConstants.kWinchPositionDeadband;
    }


    public void incremPos(){
        if (targetPosition > WinchConstants.kWinchTopPosition) {
            targetPosition = targetPosition;
        }
        else{
            targetPosition += 0.5;
        }
    }

    public void decremPos(){
        if (targetPosition < WinchConstants.kWinchBottomPosition) {
            targetPosition = targetPosition;
        }
        targetPosition -= 0.5;
    }

    @Override
    public void periodic(){
        // if (!atPosition()) {
        //     kWinchController.setSetpoint(targetPosition, ControlType.kPosition);
        // }

        SmartDashboard.putNumber("Winch Pos", getPos());
    }

}
