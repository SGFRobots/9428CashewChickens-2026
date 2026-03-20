package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterControl extends Command {
    private final Shooter mShooter;
    private final GenericHID mController;
    public final PIDController turretPID;
    public final PIDController resetPID;
    public boolean resetting;

    public ShooterControl(Shooter pShooter, GenericHID pController) {
        mShooter = pShooter;
        mController = pController;
        resetting = false;

        turretPID = new PIDController(0.2, 0, 0);
        resetPID = new PIDController(0.02, 0, 0);

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()){
            double buttonPressed = mController.getRawAxis(Constants.Controllers.DrivingController.RightHoldBtn);
            if (buttonPressed == 1) {
                mShooter.shoot(0.6);                
            } else {
                mShooter.stop();
            }

                // They call me doctor worm.
                // Good morning how are you?
                // I'm Doctor Worm.
                // I'm interested in things
                // I'm not a real doctor 
                // But I am a real worm
                // I am an actual worm
                // I live like a worm
                // I like to play the drums
                // I think I'm getting good
                // But I can handle criticism
                // I'll show you what I know
                // And you can tell me if you think
                // I'm getting better on the drums
                // I'll leave the front unlocked
                // 'Cause I can't hear the doorbell
                // When I get into it, I can't tell if you are
                // Watching me twirling the stick
                // When I give the signal, my friend
                // Rabbi Vole will play the solo
                // Someday, somebody else besides me will
                // Call me by my stage name, they will
                // Call me Doctor Worm
                // Good morning, how are you?
                // I'm Doctor Worm
                // I'm interested in things
                // I'm not a real doctor
                // But I am a real worm
                // I am an actual worm
                // I live like a worm
                // And I like to play the drums
                // I think I'm getting good
                // But I can handle criticism
                // I'll show you what I know
                // And you can tell me if you think
                // I'm getting better on the drums
                // I'm not a real doctor
                // But they call me Doctor Worm
            
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
