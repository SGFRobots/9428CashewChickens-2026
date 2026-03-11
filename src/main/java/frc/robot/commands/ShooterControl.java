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
    private double[] LLData;
    private Pose3d LLData2;
    public final PIDController turretPID;

    public ShooterControl(Shooter pShooter, GenericHID pController) {
        mShooter = pShooter;
        mController = pController;
        shooterRunning = false;
        shooterCounter = 0;

        turretPID = new PIDController(0.31, 0, 0);

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        LLData = mShooter.getLLData();
        LLData2 = mShooter.getLLData2();

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
        // SmartDashboard.putNumberArray("LLData",LLData);
        // long x_val = Math.round(LLData[0] * 100);
        // SmartDashboard.putNumber("camera_x", x_val);
        // double angle_val = LLData[3];
        // SmartDashboard.putNumber("camera_yaw", angle_val);
        double x_val = LLData2.getX();
        if(Math.abs(x_val) >= 0.00){
            double turretpower = turretPID.calculate(x_val, 0);
            mShooter.turn(turretpower);
            SmartDashboard.putNumber("Turret Power", turretpower);

        }
        else{
            mShooter.stopTurret();
        }

        
        // if (Math.abs(x_val) >= 1) {
        //     double turretpower = -turretPID.calculate(x_val, 0);
        //     // double turretpower *= (LLData[0] > 0) ? 0.05 : -0.05;
        //     // System.out.println(turretpower);
        //     // turretpower = Math.abs(turretpower) > 0.08 ? turretpower : 0.0;
        //     mShooter.turn(turretpower);
        // } else {
        //     mShooter.stopTurret();
        // }
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
