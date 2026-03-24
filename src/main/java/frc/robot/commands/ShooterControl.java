package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterControl extends Command {
    private final Shooter mShooter;
    private final GenericHID roninController;
    public final PIDController turretPID;
    public final PIDController resetPID;

    public ShooterControl(Shooter pShooter, GenericHID rController) {
        mShooter = pShooter;
        roninController = rController; 

        turretPID = new PIDController(0.2, 0, 0);
        resetPID = new PIDController(0.02, 0, 0);

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()){
            double revButtonPressed = roninController.getRawAxis(Constants.Controllers.RoninController.reversyPort);
            boolean shooting = false;

            double roninPower = (roninController.getRawAxis(Constants.Controllers.RoninController.PowerDialPort) + 1) /2;
            roninPower = (roninPower>=0.7) ? 0.7 : roninPower;
            SmartDashboard.putNumber("SHOOTER POWER",roninPower);
            
            if (revButtonPressed == 1) {
                mShooter.shoot(-0.7);
            } else {
                double buttonPressed = roninController.getRawAxis(Constants.Controllers.RoninController.ShootyPort);
                if (buttonPressed == 1) {
                    mShooter.shoot(roninPower); 
                    shooting = true; 
                } else {
                    // mShooter.shoot(0.1);
                    mShooter.stop();
                    shooting = false;
                }
            }
            SmartDashboard.putBoolean("Shooting", shooting);

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
