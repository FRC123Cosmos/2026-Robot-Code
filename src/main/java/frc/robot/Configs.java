package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WinchConstants;


public final class Configs {
    public static final class MAXSwereveModule{
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

        static{
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedMps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V1 (use REV_ThroughBoreEncoderV2 for V2):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class ShooterMaxConfig {

        public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
        public static final SparkMaxConfig follwerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig kickerConfig = new SparkMaxConfig();

        static{
                shooterConfig
                        .idleMode(IdleMode.kCoast)
                        .inverted(true)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                shooterConfig.closedLoop.feedForward
                        .kV(2e-4);
                shooterConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(1.25e-4, 0e-4, 0)
                        .iZone(0.2)
                        .outputRange(-1, 1);

                follwerConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12)
                        .follow(ShooterConstants.kShooterCANID, true);
                follwerConfig.closedLoop.feedForward
                        .kV(2e-4);
                follwerConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(1.25e-4, 0, 0)
                        .iZone(0.2)
                        .outputRange(-1, 1);
                
                kickerConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                kickerConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.7, 0, 0)
                        .outputRange(-0.7, 0.7);
                }
        }

        public static final class IntakeConfigs {
        
                public static final SparkMaxConfig indexerConfigs = new SparkMaxConfig();
                public static final SparkMaxConfig intakeConfigs = new SparkMaxConfig();
                public static final SparkMaxConfig intakeRollerConfigs = new SparkMaxConfig();

                static {
                        indexerConfigs
                                .idleMode(IdleMode.kCoast)
                                .inverted(true)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12);
                        indexerConfigs.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.8, 0, 0)
                                .outputRange(-1, 1);

                        intakeConfigs
                                .idleMode(IdleMode.kBrake)
                                .inverted(true)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12);
                        intakeConfigs.absoluteEncoder
                                .positionConversionFactor(360)
                                .velocityConversionFactor(360 / 60.0)
                                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
                        intakeConfigs.softLimit
                                .forwardSoftLimit(IntakeConstants.kIntakeFinalPosition)
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(IntakeConstants.kIntakeInitialPosition)
                                .reverseSoftLimitEnabled(true);
                        // intakeConfigs.closedLoop.maxMotion
                        //         .cruiseVelocity(180)
                        //         .maxAcceleration(100)
                        //         .allowedProfileError(1);
                        // intakeConfigs.closedLoop.feedForward
                        //         .kV(0.0005);
                                // .kG(0.9); // 2.0
                                // .kCos();
                        intakeConfigs.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                // .pid(0.0015, 0e-5, 0, ClosedLoopSlot.kSlot0)
                                // .iZone(0.1, ClosedLoopSlot.kSlot0)
                                // .pid(0.0015, 0e-5, 0)
                                .pid(0.002, 1e-5, 0)
                                .iZone(0.1)
                                // .outputRange(-0.8, 0.8);
                                .outputRange(-0.1, 0.1);

                        intakeRollerConfigs
                                .idleMode(IdleMode.kCoast)
                                .smartCurrentLimit(30)
                                .voltageCompensation(12);
                        intakeRollerConfigs.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.8, 0, 0)
                                .outputRange(-1, 1);
                }
        }

        public static final class WinchConfigs {
        
                public static final SparkFlexConfig winchConfig = new SparkFlexConfig();
                public static final SparkFlexConfig winchFollowerConfig = new SparkFlexConfig();

                static {
                        winchConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                        // winchConfig.softLimit
                        // .forwardSoftLimit(WinchConstants.kWinchTopPosition)
                        // .forwardSoftLimitEnabled(false)
                        // .reverseSoftLimit(WinchConstants.kWinchBottomPosition)
                        // .reverseSoftLimitEnabled(false);
                        // winchConfig.encoder
                        // .positionConversionFactor((1 / WinchConstants.kWinchGearReduction))
                        // .velocityConversionFactor((1 / WinchConstants.kWinchGearReduction) / 60);
                        winchConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.05, 2e-5, 0)
                        .iZone(0.1)
                        .outputRange(-0.75, 0.75);

                        winchFollowerConfig
                        .idleMode(IdleMode.kCoast)
                        .follow(WinchConstants.kWinchCANID, true)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                }
        }

        public static final class HoodConfigs {
        
                public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

                static {
                        hoodConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(20)
                        .voltageCompensation(12);
                        hoodConfig.encoder
                        .positionConversionFactor(2.67 / HoodConstants.kHoodGearReduction)
                        .velocityConversionFactor((2.67 / HoodConstants.kHoodGearReduction)/60);
                        hoodConfig.softLimit
                        .forwardSoftLimit(HoodConstants.kHoodFinalPosition)
                        .forwardSoftLimitEnabled(false)
                        .reverseSoftLimit(HoodConstants.kHoodInitialPosition)
                        .reverseSoftLimitEnabled(false);
                        hoodConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(.05, 0, 0)
                        .iZone(0.1)
                        .outputRange(-0.4, 0.4);
                }
        }
}
