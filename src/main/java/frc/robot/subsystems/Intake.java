package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private final SparkMax uppyDownyMotor;
    private final SparkMax spinnyMotor;
    private final SparkMax uppyDownyLeftMotor;
    private double uppyUp;
    private double uppyDown;
    private int position;

    public Intake(){
        uppyDownyMotor = new SparkMax(Constants.MotorPorts.kUpDownID, MotorType.kBrushless);
        uppyDownyLeftMotor = new SparkMax(Constants.MotorPorts.kUpDownTwoID, MotorType.kBrushless);
        spinnyMotor = new SparkMax(Constants.MotorPorts.kSpinnyID, MotorType.kBrushless);

        zeroPos();
    }

    public void setPowerSpinny(double power){
        spinnyMotor.set(power);
    }

    public void setPowerUppyDowney(double power){
        uppyDownyMotor.set(power);
        uppyDownyLeftMotor.set(power);
    }

    public void stopUppyDowney(){
        uppyDownyMotor.stopMotor();
        uppyDownyLeftMotor.stopMotor();
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

    public double getDownPos() {
        return uppyDown;
    }

    public double getUpPos() {
        return uppyUp;
    }

    public double getMidPoint() {
        return (uppyDown + uppyUp) / 2.0;
    }

    public void periodic() {
        if (getPos() < (getMidPoint())) {
            // currently down
            position = -1;
        } else {
            // currently up
            position = 1;
        }
        SmartDashboard.putBoolean("IntakeDown", position==-1);
    }

    public int getPosHalf() {
        return position;
    }

    public void zeroPos() {
        // if (getPos() < 2.0) {
            uppyUp = getPos();
            uppyDown = uppyUp + Constants.Mechanical.intakeDownLimit;
            position = 1;
        // } else {
        //     uppyDown = getPos();
        //     uppyUp = uppyDown - Constants.Mechanical.intakeDownLimit;
        //     position = -1;
        // }
    }
}
