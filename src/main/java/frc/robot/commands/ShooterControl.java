package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterControl extends Command {
    private final Shooter mShooter;
    private final GenericHID mController;
    private boolean shooterRunning;
    private int shooterCounter;
    private Pose3d LLData;
    public final PIDController turretPID;
    public final PIDController resetPID;
    public boolean resetting;

    public ShooterControl(Shooter pShooter, GenericHID pController) {
        mShooter = pShooter;
        mController = pController;
        shooterRunning = false;
        shooterCounter = 0;
        resetting = false;

        turretPID = new PIDController(2, 0, 0);
        resetPID = new PIDController(0.02, 0, 0);

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        LLData = mShooter.getLLData3d();

        if (DriverStation.isTeleop()){
            double buttonPressed = mController.getRawAxis(Constants.Controllers.DrivingController.RightHoldBtn);
            if ((buttonPressed == 1) && !shooterRunning) {
                shooterRunning = true;
            } else if (shooterRunning) {
                shooterCounter++;
            }

            if (shooterCounter >= 10) {
                mShooter.runKicker(0.75);                
            }
            
            if(buttonPressed == 1){
                mShooter.setPower(1);
            }
            else{
                mShooter.stop();
                shooterCounter = 0;
                shooterRunning = false;
            }
        }

        // Auto Aim
        double x_val = LLData.getX();
        if (Math.abs(mShooter.getRelativePos()) < 70) {
            if((Math.abs(x_val) >= 0.05)){
                // double turretpower = x_val * 0.05;
                double turretpower = turretPID.calculate(x_val, 0);
                mShooter.turn(turretpower);
                SmartDashboard.putNumber("Turret Power", turretpower);
                
            }
            else{
                mShooter.stopTurret();
            }
        } else {
            resetting = true;
        }

        if (resetting) {
            if (Math.abs(mShooter.getRelativePos()) >= 5) {
                double power = resetPID.calculate(mShooter.getRelativePos(), 0);
                mShooter.turn(power);
            } else {
                resetting = false;
            }          
        }

        SmartDashboard.putNumber("turret position", mShooter.getRelativePos());
        SmartDashboard.putBoolean("turret reset", resetting);

        
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
