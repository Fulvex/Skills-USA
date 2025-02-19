package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Skills USA TeleOP", group = "TeleOp")
public class SkillsUSACode extends LinearOpMode {

    // Declare motors and servo
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo clawServo;

    // Define servo positions
    private static final double CLAW_OPEN = 0.1;
    private static final double CLAW_CLOSED = 0.0;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Reverse right motors to ensure correct movement
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servo position
        clawServo.setPosition(CLAW_CLOSED);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joystick inputs
            double forward = gamepad1.left_stick_y;  // Forward/Backward
            double turn = -gamepad1.right_stick_x;   // Turning

            // Compute motor power values
            double leftPower = forward + turn;
            double rightPower = forward - turn;

            // Normalize power values to keep them within the -1 to 1 range
            double maxPower = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftPower /= maxPower;
            rightPower /= maxPower;

            // Apply power to motors
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            // Control servo claw with triggers
            if (gamepad1.right_trigger > 0.5) {
                clawServo.setPosition(CLAW_OPEN);
            } else if (gamepad1.left_trigger > 0.5) {
                clawServo.setPosition(CLAW_CLOSED);
            }

            // Telemetry for debugging
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.update();
        }
    }
}
