package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeCommand extends Command{
    private final Intake mIntake;
    private final GenericHID mController;
    private boolean IntakeDown;
    private final double deadzone = 0.5;
    private boolean upping;

    public IntakeCommand(Intake pIntake, GenericHID pController, OpenIntakeCommand pOpenIntake){
        mIntake = pIntake;
        mController = pController;
        IntakeDown = true;
        upping = false;

        addRequirements(mIntake);
    }
    
    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()){
            // Set Power spinny
            double leftButtonPressed = mController.getRawAxis(Constants.Controllers.DrivingController.LeftHoldBtn);
            if ((leftButtonPressed==1) && (mIntake.getPosHalf() == -1)){
                mIntake.setPowerSpinny(0.7);
            }
            else {
                mIntake.stopSpinny();
            }

            // Set Power uppy downey
            // double rightButtonPressed = mController.getRawAxis(Constants.Controllers.DrivingController.RightHoldBtn);
            // if ((rightButtonPressed == 1) && (!(mIntake.getPos() >= mIntake.getDownPos() - deadzone))) {
            //     mIntake.setPowerUppyDowney(0.3);
            // } else if ((rightButtonPressed == 0) && (!(mIntake.getPos() <= mIntake.getUpPos() + deadzone))) {
            //     mIntake.setPowerUppyDowney(-0.3);
            // } else {
            //     mIntake.stopUppyDowney();
            // }
        }

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

