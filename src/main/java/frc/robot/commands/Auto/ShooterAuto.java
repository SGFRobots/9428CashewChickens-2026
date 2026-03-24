package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterAuto extends Command {
    public final Shooter mShooter;
    private final Timer timer;
    private final double timeRunning;

    public ShooterAuto(Shooter pShooter) {
        mShooter = pShooter;
        timer = new Timer();
        timeRunning = 5;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override 
    public void execute() {
        mShooter.shoot(0.7);
    }

    @Override
    public void end(boolean isFinished) {
        mShooter.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= timeRunning;
    }
}
