package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAuto extends Command {
    private final Intake mIntake;

    public IntakeAuto(Intake pIntake) {
        mIntake = pIntake;
    }

    @Override
    public void initialize() {}
    
    @Override 
    public void execute() {
        mIntake.setPowerSpinny(0.4);
    }

    @Override
    public void end(boolean isFinished) {
        mIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isAutonomous();
    }
}
