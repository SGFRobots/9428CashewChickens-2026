package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterControl extends Command {
    private final Shooter mShooter;
    private final GenericHID mController;
    private boolean shooterRunning;
    private int shooterCounter;

    public ShooterControl(Shooter pShooter, GenericHID pController) {
        mShooter = pShooter;
        mController = pController;
        shooterRunning = false;
        shooterCounter = 0;

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (Robot.stage.equals("teleOp")){
            Boolean buttonPressed = mController.getRawButton(Constants.Controllers.DrivingController.LeftButton);
            Boolean newlyPressed = mController.getRawButtonPressed(Constants.Controllers.DrivingController.LeftButton);
            if (newlyPressed) {
                shooterRunning = true;
            } else if (shooterRunning) {
                shooterCounter++;
            }

            if (shooterCounter >= 10) {
                mShooter.runKicker(0.5);                
            }
            
            if(buttonPressed){
                mShooter.setPower(1);
            }
            else{
                mShooter.stop();
                shooterCounter = 0;
                shooterRunning = false;
            }
        }
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
