package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    // Motors
    private final SparkMax mMotor;

    public Shooter() {
        mMotor = new SparkMax(Constants.MotorPorts.kKickerID, MotorType.kBrushless);

    }

    public void shoot(double power) {
        mMotor.set(power);
    }

    public void stop() {
        mMotor.stopMotor();
    }

}
