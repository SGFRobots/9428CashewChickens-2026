package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class ShooterControl extends Command {
    private final Shooter mShooter;
    private final GenericHID roninController;
    public final PIDController turretPID;
    public final PIDController resetPID;
    private final Limelight mLimelight;
    private final Timer mTimer;
    private final double delay = 1;
    private boolean shooterRunning = false;

    public ShooterControl(Shooter pShooter, GenericHID rController, Limelight pLimelight) {
        mShooter = pShooter;
        roninController = rController; 
        mLimelight = pLimelight;

        turretPID = new PIDController(0.2, 0, 0);
        resetPID = new PIDController(0.02, 0, 0);

        mTimer = new Timer();

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {
            double revButtonPressed = roninController.getRawAxis(Constants.Controllers.RoninController.reversyPort);
            boolean shooting = false;
            double roninPower = (roninController.getRawAxis(Constants.Controllers.RoninController.PowerDialPort) + 1) /2;
            // roninPower = (roninPo7) : roninPower;
            
            Pose3d LLdata = mLimelight.getLL3d("limelight-front");
            double dist = LLdata.getZ();
            double power = dist / 4.035;
            boolean autoPower = roninController.getRawAxis(Constants.Controllers.RoninController.leftSwitch) == -1;
            
            SmartDashboard.putNumber("SHOOTER POWER", roninPower);
            
            if (revButtonPressed == 1) {
                mShooter.shoot(0.7);
            } else {
                // mShooter.shoot(roninPower);
                double buttonPressed = roninController.getRawAxis(Constants.Controllers.RoninController.ShootyPort);
                if (buttonPressed == 1) {
                    // mShooter.spinKicker(1);

                    if (!shooterRunning) {
                        shooterRunning = true;
                        mTimer.restart();
                    }
                    mShooter.shoot(autoPower? -power : -roninPower); // UNCOMMENT IF WANT AUTO POWER
                    if ((shooterRunning) && (mTimer.get() > delay)) {
                        // mShooter.shoot(-roninPower); 
                        mShooter.spinKicker(0.7);


                        // mShooter.setServo(Constants.Mechanical.shooterGateDown);
                        // mShooter.lowerGate();
                        shooting = true; 
                    } else {
                        mShooter.stopKicker();
                    }
                    
                } else {
                    // mShooter.shoot(0.1);
                    mShooter.stop();
                    mShooter.stopKicker();
                    // mShooter.setServo(Constants.Mechanical.shooterGateUp);
                    // mShooter.raiseGate();
                    shooting = false;
                    shooterRunning = false;
                }
            }
            SmartDashboard.putBoolean("Shooting", shooting);
            // SmartDashboard.putNumber("servoangle", mShooter.getServoAngle());
            // SmartDashboard.putNumber("GatePos", mShooter.getGatePos());
            SmartDashboard.putNumber("DISTANCE", dist);
            SmartDashboard.putNumber("Auto Power", power);
            SmartDashboard.putBoolean("Auto Powering", autoPower);

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
