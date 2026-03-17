package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OpenIntakeCommand extends Command{
    private final Intake mIntake;

    public OpenIntakeCommand(Intake pIntake) {
        mIntake = pIntake;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        mIntake.setPowerUppyDowney(0.4);
    }

    @Override
    public void end(boolean isFinished) {
        mIntake.stopUppyDowney();
    }

    @Override
    public boolean isFinished() {
        return mIntake.getPos() >= mIntake.getDownPos();
    }    
}
