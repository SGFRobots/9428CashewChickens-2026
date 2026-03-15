package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Limelight.LimelightHelpers;


public class Shooter extends SubsystemBase{
    // Motors
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final SparkMax kickerMotor;
    private final SparkMax turretMotor;
    private double zeroPos;

    public Shooter() {
        leftMotor = new TalonFX(Constants.MotorPorts.kLeftShooterID);
        rightMotor = new TalonFX(Constants.MotorPorts.kRightShooterID);
        kickerMotor = new SparkMax(Constants.MotorPorts.kKickerID, MotorType.kBrushless);
        turretMotor = new SparkMax(Constants.MotorPorts.kTurretID, MotorType.kBrushless);

        setZeroPos();

        LimelightHelpers.setPipelineIndex("limelight-turret", 0);
        LimelightHelpers.SetIMUMode("limelight-turret", 2);
        LimelightHelpers.setPriorityTagID("limelight-turret", 10);

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

    public void turn(double power) {
        // Positive = Left
        turretMotor.set(power);
    }

    public void stopTurret() {
        turretMotor.stopMotor();
    }

    public void stop() {
        stopShooter();
        stopKicker();
    }

    public double getTurretPos() {
        return turretMotor.getEncoder().getPosition();
    }

    public double getRelativePos() {
        return getTurretPos() - zeroPos;
    }

    public double getZeroPos() {
        return zeroPos;
    }

    public void setZeroPos() {
        zeroPos = getTurretPos();
    }

    public Pose3d getLLData3d() {
        Pose3d dataPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-turret");
        SmartDashboard.putNumber("llheartbeat", LimelightHelpers.getHeartbeat("limelight-turret"));
        SmartDashboard.putString("cameralimelight", dataPose.toString());
        return dataPose;
    }

}
