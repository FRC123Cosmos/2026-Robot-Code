package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AgitatedPulseAndShootCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.OscillateIntakeCommand;
import frc.robot.commands.PulseAndIntakeCommand;
import frc.robot.commands.PulseIndexerCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;

public class RobotContainer {

  private final DriveSubsystem robotDrive = new DriveSubsystem();
  // private final VisionSubsystem vision = new VisionSubsystem();
  private final IntakeSubsystem intake = new  IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  // private final WinchSubsystem winch = new WinchSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();


  private final CommandXboxController driverControllerCommand =
    new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController coPilotControllerCommand = 
    new CommandXboxController(OIConstants.kCoPilotControllerPort);

  // private final SendableChooser<Command> autoChooser;

  private final SendableChooser<AutoPos> autoPosition;

  private double autoDelay;

  
  public RobotContainer() {
    configureBindings();

    autoPosition = new SendableChooser<AutoPos>();
    autoPosition.addOption("Left", AutoPos.Left);
    autoPosition.addOption("Center", AutoPos.Center);
    autoPosition.addOption("Right", AutoPos.Right);
    autoPosition.setDefaultOption("Center", AutoPos.Center);
    SmartDashboard.putData("Auto Pos", autoPosition);

    // autoChooser = AutoBuilder.buildAutoChooser("Example_Auto");
    // SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putNumber("Auto Delay", 0);

  }

  private void configureBindings() {

    // -------------------------------- Driver Controller Binding ----------------------------------- //
    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));

    // driverControllerCommand.a().whileTrue(new AlignToTagCommand(robotDrive, vision));
    driverControllerCommand.y().whileTrue(new RunCommand(() -> robotDrive.setX()));
    driverControllerCommand.start().onTrue(new InstantCommand(() -> robotDrive.zeroHeading(), robotDrive));

    // -------------------------------- CoPilot Controller Binding ----------------------------------- //
    coPilotControllerCommand.leftTrigger().whileTrue(new PulseAndIntakeCommand(intake));
    coPilotControllerCommand.leftBumper().whileTrue( new StartEndCommand(
      () -> intake.setIndexer(-0.3), 
      () -> intake.setIndexer(0.0)));
    // coPilotControllerCommand.rightBumper().whileTrue(
    //   new ParallelCommandGroup(
    //     new PulseAndShootCommand(shooter, intake, 3000, false),
    //     new AgitateIntakeCommand(intake)));

    coPilotControllerCommand.rightBumper().whileTrue(new OscillateIntakeCommand(intake));

    coPilotControllerCommand.rightTrigger().whileTrue(
      new AgitatedPulseAndShootCommand(shooter, intake, 3000, true));

    // coPilotControllerCommand.rightTrigger().whileTrue(new ShootCommand(shooter, intake, 3000, false));
    // controllerOne.leftBumper().whileTrue(new IntakeCommand(intake, false));
    // controllerOne.rightBumper().whileTrue(new ParallelCommandGroup(
    //     // new RunCommand(() -> intake.agitateFuel(true), intake), 
    //     new InstantCommand(() -> shooter.setShooterVelocity(ShooterConstants.kTestShooterVelocityRPM)), 
    //     new RunCommand(() -> shooter.kickFuel(true), shooter)
    //   ));
    coPilotControllerCommand.a().onTrue(new InstantCommand(() -> intake.setIntakePosition(45)));
    coPilotControllerCommand.b().onTrue(new InstantCommand(() -> intake.setIntakePosition(100)));
    coPilotControllerCommand.y().onTrue(new InstantCommand(() -> intake.setIntakePosition(10)));
    // coPilotControllerCommand.a().whileTrue(new ShootCommand(shooter, intake, 2850, false));
    // coPilotControllerCommand.b().whileTrue(new ShootCommand(shooter, intake, 2775, false));
    // coPilotControllerCommand.y().whileTrue(new ShootCommand(shooter, intake, 2700, false));
  }

  // private boolean leftTrigger() {
  //   return copilotController.getRawAxis(2) > 0.75;
  // }
  // private boolean rightTrigger() {
  //   return copilotController.getRawAxis(3) > 0.75;
  // }
  // private boolean R1Down() {
  //   return copilotController.getRawAxis(5) > 0.75;
  // }
  // private boolean R1Up() {
  //   return copilotController.getRawAxis(5) < -0.75;
  // }
  // private boolean R1Left(){
  //   return copilotController.getRawAxis(4) < -0.75;
  // }
  // private boolean R1Right(){
  //   return copilotController.getRawAxis(4) > 0.75;
  // }
  // private boolean L1Down() {
  //   return copilotController.getRawAxis(1) > 0.75;
  // }
  // private boolean L1Up() {
  //   return copilotController.getRawAxis(1) < -0.75;
  // }


  public Command getAutonomousCommand() {

    autoDelay = SmartDashboard.getNumber("Auto Delay", 0);

    robotDrive.zeroHeading();
    if (autoPosition.getSelected() == AutoPos.Center) {
      robotDrive.setFieldRelativeOffset(180);
    }
    else if (autoPosition.getSelected() == AutoPos.Left) {
      robotDrive.setFieldRelativeOffset(-135);
    }
    else if (autoPosition.getSelected() == AutoPos.Right) {
      robotDrive.setFieldRelativeOffset(135);
    }
    // return new WaitCommand(autoDelay).andThen(autoChooser.getSelected());
    return null;
  }


  public enum AutoPos{
    Left, Center, Right
  }

}
