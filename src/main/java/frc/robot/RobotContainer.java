package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

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
import frc.robot.commands.FaceTagCommand;
import frc.robot.commands.OscillatedPulseAndIntakeCommand;
import frc.robot.commands.PulseAndShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WinchSubsystem;

public class RobotContainer {

  private final DriveSubsystem robotDrive = new DriveSubsystem();
  // private final VisionSubsystem vision = new VisionSubsystem();
  private final IntakeSubsystem intake = new  IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final WinchSubsystem winch = new WinchSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();


  private final CommandXboxController driverControllerCommand =
    new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController coPilotControllerCommand = 
    new CommandXboxController(OIConstants.kCoPilotControllerPort);

  private final SendableChooser<Command> autoChooser;

  private final SendableChooser<AutoPos> autoPosition;

  private double autoDelay;

  
  public RobotContainer() {

    /* ---------------------------------------------PathPlanner Commands---------------------------------------- */

    NamedCommands.registerCommand("OscillateIntake", new OscillatedPulseAndIntakeCommand(intake));
    NamedCommands.registerCommand("ShootFar", new PulseAndShootCommand(shooter, intake, 3000, hood, true));
    NamedCommands.registerCommand("ShootClose", new PulseAndShootCommand(shooter, intake, 2700, hood, false));
    NamedCommands.registerCommand("AgitatedShootFar", 
      new AgitatedPulseAndShootCommand(shooter, intake, 3000, true, hood, false));
    NamedCommands.registerCommand("lowerWinch", new InstantCommand(() -> winch.setWinchPos(0.0)));

    /* ----------------------------------------------PathPlanner Events----------------------------------------- */

    new EventTrigger("raiseWinch").onTrue(new InstantCommand(() -> winch.setWinchPos(5)));
    new EventTrigger("intakeOut").onTrue(new InstantCommand(() -> intake.setIntakePosition(100)));
    new EventTrigger("intakeIn").onTrue(new InstantCommand(() -> intake.setIntakePosition(10)));

    /* -------------------------------------------End of PathPlanner Block--------------------------------------- */

    configureBindings();

    autoPosition = new SendableChooser<AutoPos>();
    autoPosition.addOption("Left(Trench)", AutoPos.LeftT_NZ);
    autoPosition.addOption("Center Left", AutoPos.CenterL);
    autoPosition.addOption("Center Right", AutoPos.CenterR);
    autoPosition.addOption("Right(Trench)", AutoPos.RightT_NZ);
    autoPosition.setDefaultOption("Center Left", AutoPos.CenterL);
    SmartDashboard.putData("Auto Pos", autoPosition);

    autoChooser = AutoBuilder.buildAutoChooser("Test_Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putNumber("Auto Delay", 0);
  }

  private void configureBindings() {

    // -------------------------------- Driver Controller Binding ----------------------------------- //
    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));

    // driverControllerCommand.a().whileTrue(new FaceTagCommand(robotDrive, vision, 0.0));
    // driverControllerCommand.a().whileTrue(new AlignToTagCommand(robotDrive, vision));
    driverControllerCommand.y().whileTrue(new RunCommand(() -> robotDrive.setX()));
    driverControllerCommand.start().onTrue(new InstantCommand(() -> robotDrive.zeroHeading(), robotDrive));

    // -------------------------------- CoPilot Controller Binding ----------------------------------- //
    coPilotControllerCommand.leftTrigger().whileTrue(new OscillatedPulseAndIntakeCommand(intake));
    coPilotControllerCommand.leftBumper().whileTrue( new StartEndCommand(
      () -> intake.setIndexer(-0.3), 
      () -> intake.setIndexer(0.0)));

    coPilotControllerCommand.rightBumper().whileTrue(
      new AgitatedPulseAndShootCommand(shooter, intake, 2700, true, hood, false));

    // coPilotControllerCommand.rightBumper().whileTrue(new StartEndCommand(
    //   () -> intake.setIndexer(0.8), 
    //   () -> intake.setIndexer(0.0)));


    // coPilotControllerCommand.rightBumper().whileTrue(
    //   new PulseAndShootCommand(shooter, intake, 3000));
    coPilotControllerCommand.rightTrigger().whileTrue(
      new AgitatedPulseAndShootCommand(shooter, intake, 3000, true, hood, true));


    coPilotControllerCommand.povUp().onTrue(new InstantCommand(() -> winch.setWinchPos(5)));
    coPilotControllerCommand.povDown().onTrue(new InstantCommand(() -> winch.setWinchPos(0.0)));

    // coPilotControllerCommand.rightTrigger().whileTrue(new ShootCommand(shooter, intake, 3000, false));
    // controllerOne.leftBumper().whileTrue(new IntakeCommand(intake, false));
    // controllerOne.rightBumper().whileTrue(new ParallelCommandGroup(
    //     // new RunCommand(() -> intake.agitateFuel(true), intake), 
    //     new InstantCommand(() -> shooter.setShooterVelocity(ShooterConstants.kTestShooterVelocityRPM)), 
    //     new RunCommand(() -> shooter.kickFuel(true), shooter)
    //   ));

    coPilotControllerCommand.start().onTrue(new InstantCommand(() -> intake.setIntakePosition(10)));

    coPilotControllerCommand.a().onTrue(new InstantCommand(() -> intake.setIntakePosition(10)));
    // // coPilotControllerCommand.x().onTrue(new InstantCommand(() -> intake.setIntakePosition(45)));
    // // coPilotControllerCommand.b().onTrue(new InstantCommand(() -> intake.setIntakePosition(100)));
    // coPilotControllerCommand.y().onTrue(new InstantCommand(() -> hood.setHoodPosition(3.4)));
    // // coPilotControllerCommand.a().whileTrue(new ShootCommand(shooter, intake, 2850, false));
    // // coPilotControllerCommand.b().whileTrue(new ShootCommand(shooter, intake, 2775, false));
    // // coPilotControllerCommand.y().whileTrue(new ShootCommand(shooter, intake, 2700, false));
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

    // autoDelay = SmartDashboard.getNumber("Auto Delay", 0);

    // robotDrive.zeroHeading();
    // if (autoPosition.getSelected() == AutoPos.LeftT_NZ) {
    //   robotDrive.setFieldRelativeOffset(-80);
    // }
    // else if (autoPosition.getSelected() == AutoPos.RightT_NZ) {
    //   robotDrive.setFieldRelativeOffset(80);
    // }
    // else {
    //   robotDrive.setFieldRelativeOffset(0);
    // }
    return autoChooser.getSelected();
    // return null;
  }


  public enum AutoPos{
    LeftT_NZ, CenterL, CenterR, RightT_NZ
  }

}
