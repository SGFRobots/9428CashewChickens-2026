// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// Auto
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// Controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Subsystems
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Driving.SpeedControl;
import frc.robot.commands.Driving.ResetRotations;
import frc.robot.commands.Driving.SwerveJoystick;
import frc.robot.commands.ShooterControl;
import frc.robot.commands.Auto.IntakeAuto;
import frc.robot.commands.Auto.ShooterAuto;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OpenIntakeCommand;

public class RobotContainer {

  // Controllers
  public static final GenericHID mDroneComtroller = new GenericHID(Constants.Controllers.DrivingControllerPort);
  public static final GenericHID mRoninController = new GenericHID(Constants.Controllers.RoninControllerPort);

  // auto
  private final SendableChooser<Command> autoChooser;

  // Subsystems
  public final SwerveSubsystem mSwerveSubsystem;
  private final SpeedControl mSpeedControl;
  private final Shooter mShooter;
  private final Intake mIntake;
  private final Limelight mLimelight;

  // Commands
  private final ResetRotations mResetRotations;
  public final OpenIntakeCommand mOpenIntake;
  public final IntakeAuto mIntakeAuto;
  private final ShooterAuto mShooterAuto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Driving
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mDroneComtroller));
    mResetRotations = new ResetRotations(mSwerveSubsystem);
    mSpeedControl = new SpeedControl(mSwerveSubsystem, mDroneComtroller);

    // Limelight
    mLimelight = new Limelight();

    // Shooter
    mShooter = new Shooter();
    mShooter.setDefaultCommand(new ShooterControl(mShooter, mRoninController, mLimelight));
    
    // Intake
    mIntake = new Intake();
    mIntake.setDefaultCommand(new IntakeCommand(mIntake, mDroneComtroller));
    mOpenIntake = new OpenIntakeCommand(mIntake);
    
    // Auto Commands
    mIntakeAuto = new IntakeAuto(mIntake);
    mShooterAuto = new ShooterAuto(mShooter);

    // Auto
    setUpAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    // new JoystickButton(mDroneComtroller, Constants.Controllers.DrivingController.LeftButton).onTrue(mShooter.getDefaultCommand());
    new JoystickButton(mDroneComtroller, Constants.Controllers.DrivingController.LeftButton).onTrue(new InstantCommand(() -> mSwerveSubsystem.resetYaw(), mSwerveSubsystem));
  }

  // Set up auto commands
  private void setUpAuto() {
    NamedCommands.registerCommand("Intake", mIntakeAuto);
    NamedCommands.registerCommand("Shoot", mShooterAuto);
  }

  // get selected auto
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return mAutoPath;
  }

  public Command getSpeedControlCommand() {
    return mSpeedControl;
  }

  public void displayAligningState(){
    SmartDashboard.putBoolean("Aligning", mSwerveSubsystem.getFindingPos());
  }

  public void resetRotations(){
    mResetRotations.schedule();
  }

  public static boolean driveControllerMoving() {
    boolean leftX = Math.abs(mDroneComtroller.getRawAxis(Constants.Controllers.DrivingController.LeftXPort)) >= Constants.Mechanical.kDeadzone;
    boolean leftY = Math.abs(mDroneComtroller.getRawAxis(Constants.Controllers.DrivingController.LeftYPort)) >= Constants.Mechanical.kDeadzone;
    boolean rightX = Math.abs(mDroneComtroller.getRawAxis(Constants.Controllers.DrivingController.RightXPort)) >= Constants.Mechanical.kDeadzone;

    return leftX || leftY || rightX;
  }

  public void resetHeading() {
    mSwerveSubsystem.zeroHeading();
  }

}
