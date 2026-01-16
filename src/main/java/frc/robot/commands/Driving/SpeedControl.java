package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

public class SpeedControl extends Command{
    private final SwerveSubsystem mSubsystem;
    private final GenericHID mController;

    public SpeedControl(SwerveSubsystem subsystem, GenericHID pController) {
        mController = pController;
        mSubsystem = subsystem;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {

        // Normal mode when aligning
        if (mSubsystem.getFindingPos()) {
            mSubsystem.toggleFastMode(false);
            mSubsystem.toggleSlowMode(false);

        } else if (Robot.stage.equals("auto")) {
            // Normal mode always during auto
            mSubsystem.toggleFastMode(false);
            mSubsystem.toggleSlowMode(false);

        } else if (Robot.stage.equals("teleOp")) {
            boolean fast = (mController.getRawAxis(Constants.Controllers.DrivingController.RightSwtich) == -1);
            mSubsystem.toggleFastMode(fast);
            boolean slow =(mController.getRawAxis(Constants.Controllers.DrivingController.RightSwtich) == 1);
            mSubsystem.toggleSlowMode(slow);
        }

        SmartDashboard.putBoolean("FastMode", mSubsystem.getFastMode());
        SmartDashboard.putBoolean("SlowMode", mSubsystem.getSlowMode());
        SmartDashboard.putBoolean("NormalMode", !mSubsystem.getSlowMode() && !mSubsystem.getFastMode());
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    public void fastModeOn() {
        mSubsystem.toggleFastMode(true);
        mSubsystem.toggleSlowMode(false);
        System.out.println("fast");
    }

    public void slowModeOn() {
        mSubsystem.toggleFastMode(false);
        mSubsystem.toggleSlowMode(true);
        System.out.println("slow");
    }

    public void normalModeOn() {
        mSubsystem.toggleFastMode(false);
        mSubsystem.toggleSlowMode(false);
        System.out.println("normal");
    }
}
