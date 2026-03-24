package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private final SparkMax mMotor;
    private double zero;
    private double uppyLimit;

    public Climber() {
        mMotor = new SparkMax(Constants.MotorPorts.kClimberID, MotorType.kBrushless);
        resetPos();
    }

    public void resetPos() {
        zero = mMotor.getEncoder().getPosition();
        uppyLimit = zero + Constants.Mechanical.climberUppy;
    }

    public double getAbsPos() {
        return mMotor.getEncoder().getPosition();
    }

    public double getRelPos() {
        return getAbsPos() - zero;
    }

    public double getUppyLimit() {
        return uppyLimit;
    }

    public double getZero() {
        return zero;
    }

    public void setPower(double power) {
        mMotor.set(power);
    }

    public void stop() {
        mMotor.stopMotor();
    }
}
