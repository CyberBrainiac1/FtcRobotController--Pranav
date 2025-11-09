package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name="AprilTag Align TeleOp", group="TeleOp")
public class AutoAlign extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Motor directions
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoders + braking
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, LimelightConfig.LIMELIGHT_NAME);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(5);
        limelight.setPollRateHz(100);

        waitForStart();
        limelight.start();

        boolean aligning = false;
        boolean xButtonPrev = false;

        while (opModeIsActive()) {
            if (gamepad1.x && !xButtonPrev) aligning = true;
            xButtonPrev = gamepad1.x;

            LLResult result = limelight.getLatestResult();
            double tx = 0, ty = 0;
            boolean targetVisible = false;

            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();
                for (FiducialResult fid : fiducials) {
                    if (fid.getFiducialId() == LimelightConfig.TARGET_APRILTAG_ID) {
                        tx = fid.getTargetXDegrees();
                        ty = fid.getTargetYDegrees();
                        targetVisible = true;
                        break;
                    }
                }
            }

            if (aligning) {
                if (!targetVisible) {
                    double spinPower = LimelightConfig.SPIN_SEARCH_POWER;
                    leftFront.setPower(spinPower);
                    leftBack.setPower(spinPower);
                    rightFront.setPower(-spinPower);
                    rightBack.setPower(-spinPower);
                } else {
                    double turn = Range.clip(LimelightConfig.KP_TURN * tx,
                            -LimelightConfig.MAX_TURN_POWER,
                            LimelightConfig.MAX_TURN_POWER);
                    leftFront.setPower(turn);
                    leftBack.setPower(turn);
                    rightFront.setPower(-turn);
                    rightBack.setPower(-turn);

                    if (Math.abs(tx) < LimelightConfig.TURN_DEADZONE_DEGREES) {
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                        aligning = false;
                    }
                }
            } else {
                double drive  = -gamepad1.left_stick_y;
                double strafe =  gamepad1.left_stick_x;
                double rotate =  gamepad1.right_stick_x;

                double fl = drive + strafe + rotate;
                double bl = drive - strafe + rotate;
                double fr = drive - strafe - rotate;
                double br = drive + strafe - rotate;

                double max = Math.max(1.0, Math.max(Math.abs(fl),
                        Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
                leftFront.setPower(fl / max);
                leftBack.setPower(bl / max);
                rightFront.setPower(fr / max);
                rightBack.setPower(br / max);
            }

            if (targetVisible) {
                double angleToTarget = LimelightConfig.CAMERA_TILT_DEGREES + ty;
                double distance = (LimelightConfig.TAG_HEIGHT_INCHES - LimelightConfig.CAMERA_HEIGHT_INCHES)
                        / Math.tan(Math.toRadians(angleToTarget));
                telemetry.addData("Distance (in)", distance);
            } else {
                telemetry.addData("Distance (in)", "Unknown");
            }

            telemetry.update();
        }
    }
}
