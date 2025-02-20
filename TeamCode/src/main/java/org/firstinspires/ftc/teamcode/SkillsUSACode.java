package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@TeleOp(name = "SkillsUSATeleop", group = "TeleOp")
public class SkillsUSACode extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = -0.1;

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    // Declare hardware components
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, armMotor;
    private Servo clawServo;
    private Servo servo1, servo2; // Changed from CRServo to Servo

    // Define positional servo limits
    private static final double MIN_SERVO_POS = 0.0;
    private static final double MAX_SERVO_POS = 1.0;
    private double servo1Pos = 0.7; // Default to midpoint
    private double servo2Pos = 0.7;

    // Arm motor hold power
    private static final double ARM_HOLD_POWER = 0.025;

    // Webcam
    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        clawServo = hardwareMap.servo.get("clawServo");
        servo1 = hardwareMap.get(Servo.class, "lMiniArm");  // Changed to positional servo
        servo2 = hardwareMap.get(Servo.class, "rMiniArm");  // Changed to positional servo

        // Reverse right motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Configure arm motor
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial servo positions
        servo1.setPosition(servo1Pos);
        servo2.setPosition(servo2Pos);

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
            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            armMotor.setPower(power);

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

            // Control Positional Servos (servo1 and servo2)
            if (gamepad1.right_bumper) {
                servo1Pos = Math.min(MAX_SERVO_POS, servo1Pos + 0.01); // Increase position
                servo2Pos = Math.min(MAX_SERVO_POS, servo2Pos - 0.01);
            } else if (gamepad1.left_bumper) {
                servo1Pos = Math.max(MIN_SERVO_POS, servo1Pos - 0.01); // Decrease position
                servo2Pos = Math.max(MIN_SERVO_POS, servo2Pos + 0.01);
            }

            // Set the updated servo positions
            servo1.setPosition(servo1Pos);
            servo2.setPosition(servo2Pos);

            // Debug telemetry
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.addData("Arm Power", armMotor.getPower());
            telemetry.addData("Servo1 Position", servo1.getPosition());
            telemetry.addData("Servo2 Position", servo2.getPosition());
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
