package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberControl extends Command {
    private final Climber mClimber;
    private final GenericHID mRoninController;
    private final double deadzone = 2;

    public ClimberControl(Climber pClimber, GenericHID rController) {
        mClimber = pClimber;
        mRoninController = rController;

        addRequirements(pClimber);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {
            double roninY = mRoninController.getRawAxis(1);
            // mClimber.setPower(roninY);
            SmartDashboard.putNumber("ClimberPos", mClimber.getAbsPos());
        }
        // double buttonPressed = mRoninController.getRawAxis(Constants.Controllers.RoninController.ClimberPort);
        // double ClimberPos = mClimber.getAbsPos();
        // if (buttonPressed == 0) { // button down = climber up
        //     if (ClimberPos < (mClimber.getUppyLimit() - deadzone)) {
        //         mClimber.setPower(1);
        //     } else {
        //         mClimber.stop();
        //     }
        // } else { // button up = climber down
        //     if (ClimberPos > (mClimber.getZero() + deadzone)) {
        //         mClimber.setPower(-1);
        //     } else {
        //         mClimber.stop();
        //     }
        // }
    }

    @Override
    public void end(boolean isFinished) {
        mClimber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
