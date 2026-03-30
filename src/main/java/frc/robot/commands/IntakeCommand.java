package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeCommand extends Command{
    private final Intake mIntake;
    private final GenericHID mController;
    private final OpenIntakeCommand mOpenIntake;
    private final CloseIntakeCommand mCloseIntake;
    private boolean IntakeDown;
    private final double deadzone = 0.5;
    private boolean upping;

    public IntakeCommand(Intake pIntake, GenericHID pController, OpenIntakeCommand pOpenIntake, CloseIntakeCommand pCloseIntake){
        mIntake = pIntake;
        mController = pController;
        mOpenIntake = pOpenIntake;
        mCloseIntake = pCloseIntake;
        IntakeDown = true;
        upping = false;

        addRequirements(mIntake);
    }
    
    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()){
            double leftButtonPressed = mController.getRawAxis(Constants.Controllers.DrivingController.LeftHoldBtn);
            if(leftButtonPressed==1){
                mIntake.setPowerSpinny(0.3);
            }
            else {
                mIntake.stopSpinny();
            }

            // double rightButtonPressed = mController.getRawAxis(Constants.Controllers.DrivingController.RightHoldBtn);
            // if((rightButtonPressed==1) && (!IntakeDown)){
            //     CommandScheduler.getInstance().schedule(mOpenIntake);
            //     IntakeDown = true;
            // }
            // else {
            //     if (IntakeDown) {
            //         CommandScheduler.getInstance().schedule(mCloseIntake);
            //         IntakeDown = false;
            //     }
            // }

            // double intakeSwitch = mController.getRawAxis(Constants.Controllers.DrivingController.LeftSwitch);
            // if (intakeSwitch == -1) {
            //     // go down
            //     if (mIntake.getPos() < mIntake.getDownPos() - deadzone) {
            //         double dist = Math.abs(mIntake.getDownPos() - mIntake.getPos());
            //         mIntake.setPowerUppyDowney(dist/6.64 * 0.4);
            //     }
            // } else if (intakeSwitch == 0) {
            //     if (upping) {
            //         double dist = Math.abs(mIntake.getMidPos() - mIntake.getPos());
            //         mIntake.setPowerUppyDowney(-dist/6.64 * 0.4);
            //         if (mIntake.getPos() <= mIntake.getMidPos() - deadzone) {
            //             upping = false;
            //         }
            //     } else {
            //         if (mIntake.getPos() < mIntake.getDownPos() - deadzone) {
            //             double dist = Math.abs(mIntake.getDownPos() - mIntake.getPos());
            //             mIntake.setPowerUppyDowney(dist/6.64 * 0.4);
            //         } else {
            //             upping = true;
            //         }

            //     }
            // } else if (intakeSwitch == 1) {
            //     // go up
            //     if (mIntake.getPos() < mIntake.getZeroPos() - deadzone) {
            //         double dist = Math.abs(mIntake.getZeroPos() - mIntake.getPos());
            //         mIntake.setPowerUppyDowney(dist/6.64 * 0.4);
            //     } else if (mIntake.getPos() > mIntake.getZeroPos() + deadzone) {
            //         double dist = Math.abs(mIntake.getDownPos() - mIntake.getPos());
            //         mIntake.setPowerUppyDowney(-dist/6.64 * 0.4);
            //     }
            // }

            // double rightY = mController.getRawAxis(Constants.Controllers.DrivingController.RightYPort);
            // if (mIntake.checkPos()) {
            //     rightY = (rightY <= 0.1) ? 0 : rightY;
            //     mIntake.setPowerUppyDowney(rightY);
            // } else {
            //     mIntake.stopUppyDowney();
            // }
        }

        SmartDashboard.putNumber("Intake0", mIntake.getZeroPos());
        SmartDashboard.putNumber("IntakeLimit", mIntake.getDownPos());
        SmartDashboard.putNumber("IntakePos", mIntake.getPos());

        
    }

    @Override
    public void end(boolean isFinished) {
        mIntake.stopSpinny();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

