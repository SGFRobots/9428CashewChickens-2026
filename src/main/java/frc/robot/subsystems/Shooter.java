package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    // Motors
    private final SparkMax mMotorLeft;
    private final SparkMax mMotorRight;
    // private final Servo mServo;
    private final SparkMax mGateMotor;
    private double gateZero;
    private double gateLimit;
    private final double deadzone = 2;

    public Shooter() {
        mMotorLeft = new SparkMax(Constants.MotorPorts.kLeftShooterID, MotorType.kBrushless);
        mMotorRight = new SparkMax(Constants.MotorPorts.kRightShooterID, MotorType.kBrushless);
        // mServo = new Servo(0);
        mGateMotor = new SparkMax(Constants.MotorPorts.kGateID, MotorType.kBrushless);
        zeroGate();
    }

    public void shoot(double power) {
        mMotorLeft.set(power);
        mMotorRight.set(power);
    }

    public void stop() {
        mMotorLeft.stopMotor();
        mMotorRight.stopMotor();
    }

    public void setGatePower(double power) {
        mGateMotor.set(power);
    }

    public void stopGate() {
        mGateMotor.stopMotor();
    }

    public double getGatePos() {
        return mGateMotor.getEncoder().getPosition();
    }

    public void zeroGate() {
        gateZero = getGatePos();
        gateLimit = gateZero - Constants.Mechanical.shooterGateDown;
    }

    public void lowerGate() {
        if (getGatePos() > gateLimit + deadzone) {
            setGatePower(-0.05);
        } else {
            stopGate();   
        }
    }

    public void raiseGate() {
        if (getGatePos() < gateZero - deadzone) {
            setGatePower(0.05);
        } else {
            stopGate();
        }
    }

}
