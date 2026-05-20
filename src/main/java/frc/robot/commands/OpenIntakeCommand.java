package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OpenIntakeCommand extends Command{
    private final Intake mIntake;
    private double direction;
    private final double deadzone;

    public OpenIntakeCommand(Intake pIntake) {
        mIntake = pIntake;
        deadzone = 0.7;
    }

    @Override
    public void initialize() {
        // if (mIntake.getPos() > (mIntake.getMidPoint())) {
        //     // currently down, needs to go up
        //     direction = -1;
        // } else {
        //     direction = 1;
        // }
        // if (mIntake.getPosHalf() == 1) {
        //     direction = 1;
        // } else {
        //     direction = 0;
        // }
        direction = mIntake.getPosHalf() * -1;
    }

    @Override 
    public void execute() {
        mIntake.setPowerUppyDowney(0.1 * direction);
    }

    @Override
    public void end(boolean isFinished) {
        mIntake.stopUppyDowney();
    }

    @Override
    public boolean isFinished() {
        // return mIntake.getPos() >= mIntake.getDownPos();
        if (direction == -1) {
            return mIntake.getPos() <= mIntake.getDownPos() + deadzone;
        } else {
            return mIntake.getPos() >= mIntake.getUpPos() - deadzone;
        }
    }    
}
