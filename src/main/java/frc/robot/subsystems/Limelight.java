package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Limelight.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;

public class Limelight extends SubsystemBase{
    // Limelight name
    private final String frontLL = "limelight-front";
    private final String backLL = "limelight-back";

    public Limelight() {
        // Set up limelight
        LimelightHelpers.setPipelineIndex(frontLL, 0);
        LimelightHelpers.setPipelineIndex(backLL, 0);
    }

    public Pose3d getLL3d(String LLName) {
        return LimelightHelpers.getTargetPose3d_CameraSpace(LLName);
    }
}
