package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp
public class VisionOnlyAutoAlign extends OpMode {

    private DcMotor leftBack, leftFront, rightBack, rightFront;
    private DcMotor shooter;
    private CRServo geckoLeft, geckoRight;
    private CRServo intake;
    private Limelight3A limelight;

    private boolean autoAlign = false;  // Auto‑align mode flag

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // shooter = hardwareMap.get(DcMotor.class, "shooter");
        // geckoLeft  = hardwareMap.get(CRServo.class, "geckoLeft");
        // geckoRight = hardwareMap.get(CRServo.class, "geckoRight");
        // intake     = hardwareMap.get(CRServo.class, "intake");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // **Set zero‑power behavior to BRAKE** so wheels stop firmly when power = 0
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4); // Use pipeline 4 (you mentioned 5 but code had 4) for AprilTag detection
        limelight.start();

        // shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        // geckoLeft.setPower(0);
        // geckoRight.setPower(0);
        // intake.setPower(0);
        // shooter.setPower(0);
    }

    @Override
    public void loop() {
        // Toggle auto‑align mode
        if (gamepad2.x) autoAlign = true;
        if (gamepad2.b) autoAlign = false;

        if (!autoAlign) {
            // Normal operation: stop drive motors immediately (since BRAKE mode is set)
            leftFront .setPower(0);
            leftBack  .setPower(0);
            rightFront.setPower(0);
            rightBack .setPower(0);

            // The rest remains commented (shooter, intake, etc)
            /*
            double shooterPower = gamepad1.left_trigger * 0.67;
            shooter.setPower(shooterPower);
            if (gamepad1.left_bumper) shooter.setPower(-0.67);

            if (gamepad1.b) {
                intake.setPower(-1);
                geckoLeft.setPower(1);
                geckoRight.setPower(-1);
            } else if (gamepad1.right_trigger > 0.1) {
                geckoRight.setPower(1);
                geckoLeft.setPower(-1);
                intake.setPower(1);
            } else if (gamepad1.a) {
                intake.setPower(0);
                geckoLeft.setPower(-1);
                geckoRight.setPower(1);
            } else if (gamepad1.dpad_up) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
                geckoLeft.setPower(0);
                geckoRight.setPower(0);
            }
            */
        } else {
            // === Auto‑Align Logic ===
            LLResult result = limelight.getLatestResult();
            boolean tagDetected = false;
            double tx = 0;

            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();
                for (FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == 20) {
                        tagDetected = true;
                        tx = fr.getTargetXDegrees();
                        break;
                    }
                }
            }

            if (tagDetected) {
                telemetry.addLine("Tag20 Detected");
                double turnPower = 0.1;  // you can tune this
                if (Math.abs(tx) > 2) {   // threshold to decide alignment complete
                    // rotate to align
                    if (tx > 2) {
                        leftFront .setPower( turnPower);
                        leftBack  .setPower( turnPower);
                        rightFront.setPower(-turnPower);
                        rightBack .setPower(-turnPower);
                    } else if (tx < -2) {
                        leftFront .setPower(-turnPower);
                        leftBack  .setPower(-turnPower);
                        rightFront.setPower( turnPower);
                        rightBack .setPower( turnPower);
                    }
                    telemetry.addData("Aligning", "TX = %.2f", tx);
                } else {
                    // within threshold: aligned
                    leftFront .setPower(0);
                    leftBack  .setPower(0);
                    rightFront.setPower(0);
                    rightBack .setPower(0);
                    telemetry.addLine("Aligned with Tag20!");

                    // Exit auto‐align mode now that alignment is done
                    autoAlign = false;
                }
            } else {
                // Spin in place to search for tag
                leftFront .setPower(LimelightConfig.MAX_TURN_POWER);
                leftBack  .setPower(LimelightConfig.MAX_TURN_POWER);
                rightFront.setPower(-LimelightConfig.MAX_TURN_POWER);
                rightBack .setPower(-LimelightConfig.MAX_TURN_POWER);
                telemetry.addLine("Searching for Tag20...");
            }
        }

        telemetry.update();
    }
}
