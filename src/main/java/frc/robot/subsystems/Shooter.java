package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Shooter extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final SparkMax kickerMotor;

    public Shooter() {
        leftMotor = new TalonFX(Constants.MotorPorts.kLeftShooterID);
        rightMotor = new TalonFX(Constants.MotorPorts.kRightShooterID);
        kickerMotor = new SparkMax(Constants.MotorPorts.kKickerID, MotorType.kBrushless);
    }

    public void setPower(double power) {
        leftMotor.set(power);
        rightMotor.set(-power);
    }

    public void stopShooter() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void runKicker(double power) {
        kickerMotor.set(power);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public void stop() {
        stopShooter();
        stopKicker();
    }
}
