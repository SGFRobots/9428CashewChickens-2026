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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Subsystems
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Driving.SpeedControl;
import frc.robot.commands.Driving.ResetRotations;
import frc.robot.commands.Driving.SwerveJoystick;
import frc.robot.commands.Limelight.AprilTagAlign;
import frc.robot.commands.Limelight.LimeLightControl;

public class RobotContainer {

  // Controllers
  public static final GenericHID mDroneComtroller = new GenericHID(Constants.Controllers.DrivingControllerPort);
  public static final GenericHID mXBoxController = new GenericHID(Constants.Controllers.XBoxControllerPort);

  // auto
  private final SendableChooser<Command> autoChooser;

  // Subsystems
  private final SwerveSubsystem mSwerveSubsystem;
  private final SpeedControl mSpeedControl;
  private final Limelight mLeftLimelight;
  private final Limelight mRightLimelight;

  // Commands
  private final ResetRotations mResetRotations;
  private final AprilTagAlign mTestAlign;
  private final AprilTagAlign mAprilTagLockLeft;
  private final AprilTagAlign mAprilTagLockRight;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Driving
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mDroneComtroller));
    mResetRotations = new ResetRotations(mSwerveSubsystem);
    mSpeedControl = new SpeedControl(mSwerveSubsystem, mDroneComtroller);
    
    // Limelight
    mLeftLimelight = new Limelight(Constants.MotorPorts.kLeftLimelightKey);
    mLeftLimelight.setDefaultCommand(new LimeLightControl(mLeftLimelight));
    mRightLimelight = new Limelight(Constants.MotorPorts.kRightLimelightKey);
    mRightLimelight.setDefaultCommand(new LimeLightControl(mRightLimelight));

    // Auto Alignment
    mAprilTagLockLeft = new AprilTagAlign(mSwerveSubsystem, mRightLimelight, Constants.AprilTags.leftCoral[0], Constants.AprilTags.leftCoral[1], Constants.AprilTags.leftCoral[2]);
    mAprilTagLockRight = new AprilTagAlign(mSwerveSubsystem, mLeftLimelight, Constants.AprilTags.rightCoral[0], Constants.AprilTags.rightCoral[1], Constants.AprilTags.rightCoral[2]);
    // mAprilTagLockLeft = new AprilTagAlign(mSwerveSubsystem, mRightLimelight, -0.04, 0.34, 0);
    // mAprilTagLockRight = new AprilTagAlign(mSwerveSubsystem, mLeftLimelight, 0, 0.3, 0);
    mTestAlign = new AprilTagAlign(mSwerveSubsystem, mLeftLimelight, Constants.AprilTags.rightCoral[0], Constants.AprilTags.rightCoral[1], Constants.AprilTags.rightCoral[2]);
    
    // Auto
    setUpAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonAPort).onTrue(mResetRotations);
  }

  // Set up auto commands
  private void setUpAuto() {
    NamedCommands.registerCommand("reefAlignLeft", mAprilTagLockLeft);
    NamedCommands.registerCommand("reefAlignRight", mAprilTagLockRight);
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
