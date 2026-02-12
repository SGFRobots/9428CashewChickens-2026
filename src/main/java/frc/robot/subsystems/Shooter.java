package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public Shooter() {
        leftMotor = new TalonFX(Constants.MotorPorts.kLeftShooterID);
        rightMotor = new TalonFX(Constants.MotorPorts.kRightShooterID);
    }

    public void setPower(double power) {
        leftMotor.set(power);
        rightMotor.set(-power);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
