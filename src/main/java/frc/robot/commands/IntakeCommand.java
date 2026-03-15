package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

import java.util.ResourceBundle.Control;

import javax.naming.TimeLimitExceededException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeCommand extends Command{
    private final Intake mIntake;
    private final GenericHID mController;

    public IntakeCommand(Intake pIntake, GenericHID pController){
        mIntake = pIntake;
        mController = pController;

        addRequirements(mIntake);
    }
    
    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()){
            double leftButtonPressed = mController.getRawAxis(Constants.Controllers.DrivingController.LeftHoldBtn);
            if(leftButtonPressed==1){
                mIntake.setPowerSpinny(0.4);
            }
            else {
                mIntake.stopSpinny();
            }

            double rightY = mController.getRawAxis(Constants.Controllers.DrivingController.RightYPort);
            if (mIntake.checkPos()) {
                rightY = (rightY <= 0.1) ? 0 : rightY;
                mIntake.setPowerUppyDowney(rightY);
            } else {
                mIntake.stopUppyDowney();
            }
        }

        
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

