package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LimelightConfig;

import java.util.List;

@TeleOp(name = "Show AprilTag Distance Only", group = "Vision")
public class ShowAprilTagDistanceOnly extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, LimelightConfig.LIMELIGHT_NAME);

        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(5);//pipeline 5
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<FiducialResult> tags = result.getFiducialResults();
                for (FiducialResult tag : tags) {
                    if (tag.getFiducialId() == LimelightConfig.TARGET_APRILTAG_ID) {
                        double ty = tag.getTargetYDegrees();  // vertical angle offset
                        double totalAngleDeg = LimelightConfig.CAMERA_TILT_DEGREES + ty;
                        double totalAngleRad = Math.toRadians(totalAngleDeg);

                        double distanceInches = (LimelightConfig.TAG_HEIGHT_INCHES - LimelightConfig.CAMERA_HEIGHT_INCHES)
                                / Math.tan(totalAngleRad);

                        telemetry.addLine("AprilTag Detected");
                        telemetry.addData("ID", tag.getFiducialId());
                        telemetry.addData("Vertical Angle (ty)", "%.2f°", ty);
                        telemetry.addData("Distance to Tag", "%.2f in", distanceInches);
                        break;
                    }
                }
            } else {
                telemetry.addLine("No valid AprilTag detected.");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
