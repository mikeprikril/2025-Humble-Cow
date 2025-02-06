// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmReady;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.BumpDown;
import frc.robot.commands.ChangePipeline;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.TransferPosition;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Joysticks
  final CommandXboxController driverXbox = new CommandXboxController(Constants.OperatorConstants.DriverUSBPort);
  final CommandXboxController operatorXbox = new CommandXboxController(Constants.OperatorConstants.OperatorUSBPort);
  final Joystick panel = new Joystick(Constants.OperatorConstants.PanelUSBPort);

  //Auto Mode Chooser
  private final SendableChooser<Command> autoChooser;

  // Subsystems
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsytem arm = new ArmSubsytem();
  private final Limelight limelight = new Limelight();

  //Commands
  private final ManualElevatorCommand manualElevator;
  private final ManualArmCommand manualArm;
  private final AutoElevatorCommand autoElevator;;
  private final TransferPosition transfer;
  private final BumpDown bumpDown;
  private final ArmReady armReady;
  private final ChangePipeline changePipeline;

  private final SequentialCommandGroup autoTransfer;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
  
    //PathPlanner Named Commands
    //NamedCommands.registerCommand("Fire From Subwoofer", new FireFromSubwoofer(m_arm, m_shooter));

    //Commands
    manualElevator = new ManualElevatorCommand(elevator, operatorXbox);
    manualArm = new ManualArmCommand(arm, operatorXbox);
    autoElevator = new AutoElevatorCommand(elevator, operatorXbox);
    transfer = new TransferPosition(elevator, arm, operatorXbox);
    bumpDown = new BumpDown(elevator, arm, operatorXbox);
    armReady = new ArmReady(elevator, arm, operatorXbox);
    changePipeline = new ChangePipeline(limelight, driverXbox);

    autoTransfer = new SequentialCommandGroup(bumpDown, armReady); //sequential command group for auto transfer

    //Default Commands

    //Automode Chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode Choice", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    
    //Default Commands for each subsystem
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    elevator.setDefaultCommand(manualElevator);
    arm.setDefaultCommand(manualArm);
    limelight.setDefaultCommand(changePipeline);
  }

  private void configureBindings()
  {
    //Buttons   
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //new JoystickButton(driverXbox, 8).onTrue(new InstantCommand(drivebase::zeroGyro));
      driverXbox.back().onTrue(Commands.none());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());

      //operatorXbox.back().onTrue(autoTransfer); //run sequence of 
      operatorXbox.x().onTrue(transfer); //move to transfer position (human loading) when holding X
      operatorXbox.y().onTrue(bumpDown); //grab coral from trough when holding Y
      operatorXbox.a().onTrue(armReady); //move arm and elevator to scoring position when holding A

      //new JoystickButton(panel, 1).onTrue(transfer); //use this when we get the panel control working


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
