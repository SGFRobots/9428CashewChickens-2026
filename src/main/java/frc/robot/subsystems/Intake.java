package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Intake extends SubsystemBase{
    private final SparkMax uppyDownyMotor;
    private final SparkMax spinnyMotor;
    private double uppyZero;
    private double uppyDown;

    public Intake(){
        uppyDownyMotor = new SparkMax(Constants.MotorPorts.kUpDownID, MotorType.kBrushless);
        spinnyMotor = new SparkMax(Constants.MotorPorts.kSpinnyID, MotorType.kBrushless);

        zeroPos();
    }

    public void setPowerSpinny(double power){
        spinnyMotor.set(power);
    }

    public void setPowerUppyDowney(double power){
        uppyDownyMotor.set(power);
    }

    public void stopUppyDowney(){
        uppyDownyMotor.stopMotor();
    }

    public void stopSpinny(){
        spinnyMotor.stopMotor();
    }

    public void stop() {
        stopSpinny();
        stopUppyDowney();
    }

    public double getPos() {
        return uppyDownyMotor.getEncoder().getPosition();
    }

    public double getZeroPos() {
        return uppyZero;
    }

    public double getDownPos() {
        return uppyDown;
    }


    public double getRelativePos() {
        return uppyZero + getPos();
    }

    public void zeroPos() {
        uppyZero = getPos();
        uppyDown = uppyZero + Constants.Mechanical.intakeDownLimit;
    }

    public boolean checkPos() {
        return (getPos() > uppyZero) && (getPos() < uppyDown);
    }

}
