package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterControl extends Command {
    private final Shooter mShooter;
    private final GenericHID mController;
    

    public ShooterControl(Shooter pShooter, GenericHID pController) {
        mShooter = pShooter;
        mController = pController;

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (Robot.stage.equals("teleOp")){
            Boolean buttonPressed = mController.getRawButton(Constants.Controllers.DrivingController.LeftButton);
            if(buttonPressed){
                mShooter.setPower(1);
            }
            else{
                mShooter.stop();
            }
        }
        // mShooter.setPower(0.3);
    }

    @Override
    public void end(boolean isFinished) {
        mShooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
