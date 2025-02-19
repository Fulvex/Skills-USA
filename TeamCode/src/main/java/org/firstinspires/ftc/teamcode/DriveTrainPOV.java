package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo; // Continuous Rotation Servo
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Refined Servo Control", group = "TeleOp")
public class DriveTrainPOV extends LinearOpMode {
    // Declare hardware components
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, armMotor;
    private Servo clawServo;
    private CRServo servo1, servo2; // Continuous Rotation Servos

    // Define continuous servo speed ranges
    private static final double SERVO_SPEED = 0.2; // Speed factor for CRServos
    private static final double HOLD_POWER = 0.02; // Small power to hold servos in place

    // Arm motor hold power
    private static final double ARM_HOLD_POWER = 0.025;

    // Webcam
    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        clawServo = hardwareMap.servo.get("clawServo");
        servo1 = hardwareMap.get(CRServo.class, "lMiniArm");  // Continuous servo
        servo2 = hardwareMap.get(CRServo.class, "rMiniArm"); // Continuous servo

        // Reverse right motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        // Configure arm motor
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial servo speeds to 0 (or HOLD_POWER for holding)
        servo1.setPower(HOLD_POWER);
        servo2.setPower(HOLD_POWER);

        servo2.setDirection(CRServo.Direction.REVERSE); // Reverse servo2 direction if needed

        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera error!");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Read joystick inputs
            double forward = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            // Compute motor power
            double leftPower = forward + turn;
            double rightPower = forward - turn;

            // Normalize power values
            double maxPower = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftPower /= maxPower;
            rightPower /= maxPower;

            // Apply power to motors
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            // Control claw
            if (gamepad1.a) {
                clawServo.setPosition(0.8); // Open
            } else if (gamepad1.b) {
                clawServo.setPosition(0.5); // Closed
            }

            // Arm motor control with hold position
            if (gamepad1.left_trigger > 0.5) {
                armMotor.setPower(0.5);
            } else if (gamepad1.right_trigger > 0.5) {
                armMotor.setPower(-0.5);
            } else {
                armMotor.setPower(ARM_HOLD_POWER);
            }

            // Control Continuous Rotation Servos (servo1 and servo2)
            if (gamepad1.right_bumper) {
                // Move both servos forward
                servo1.setPower(SERVO_SPEED);  // Forward movement for servo1
                servo2.setPower(SERVO_SPEED); // Opposite direction for servo2
            } else if (gamepad1.left_bumper) {
                // Move both servos backward
                servo1.setPower(-SERVO_SPEED); // Backward movement for servo1
                servo2.setPower(-SERVO_SPEED);  // Opposite direction for servo2
            } else {
                // Hold position with a small power when no input
                servo1.setPower(HOLD_POWER);  // Hold servo1 in place
                servo2.setPower(HOLD_POWER);  // Hold servo2 in place
            }

            // Debug telemetry
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.addData("Arm Power", armMotor.getPower());
            telemetry.addData("Servo1 Power", servo1.getPower());
            telemetry.addData("Servo2 Power", servo2.getPower());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Frame Time (ms)", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline Time (ms)", webcam.getPipelineTimeMs());
            telemetry.update();
        }
    }

    // Sample OpenCV pipeline
    class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            return input;
        }
    }
}
