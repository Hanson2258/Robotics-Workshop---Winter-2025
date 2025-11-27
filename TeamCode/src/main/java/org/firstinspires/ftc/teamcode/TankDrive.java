package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp with Set and Variable controls", group="Robot Control")
public class TankDrive extends OpMode {
    //// Hardware Declaration
    // Motors
    private DcMotorEx leftDrive   = null;
    private DcMotorEx rightDrive  = null;
    private DcMotorEx shooter     = null;

    // Servos
    private Servo servo     = null;
    private CRServo crServo = null;

    //// Constants Declaration
    private final double MAX_SLOW_SPEED     = 0.5;
    private final double STARTING_SERVO_POS = 0.5;

    // Shooter
    private final double SHOOTER_TICKS_PER_SEC = 28 * 4; // GOBILDA 5203-2402-0001 PPR is 28
    private final double SHOOTER_STOPPED       =    0;
    private final double MIN_SHOOTER_RPM       = 1500;
    private final double MID_SHOOTER_RPM       = 3000;
    private final double MAX_SHOOTER_RPM       = 6000;
    private final double SHOOTER_INC_AND_DEC   =    5;

    // Servo Pos
    private final double SERVO_START_POS   = 0.0;
    private final double SERVO_MID_POS     = 0.5;
    private final double SERVO_END_POS     = 1.0;
    private final double SERVO_INC_AND_DEC = 0.01;

    // Servo Speed
    private final double CRSERVO_REVERSE_SPEED = -1.0;
    private final double CRSERVO_STOP_SPEED    =  0.0;
    private final double CRSERVO_FORWARD_SPEED =  1.0;
    private final double CRSERVO_INC_AND_DEC   =  0.05;

    //// Variables
    private boolean fastModeEnabled   = false;
    private double targetShooterRPM   = SHOOTER_STOPPED;
    private double targetServoPos     = SERVO_MID_POS;
    private double targetCRServoSpeed = CRSERVO_STOP_SPEED;


    @Override
    public void init() {
        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Motor Direction
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);

        // Motor Mode
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Motor Encode Mode
        leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        servo   = hardwareMap.get(Servo.class, "servo");
        crServo = hardwareMap.get(CRServo.class, "CRServo");
        servo.setPosition(STARTING_SERVO_POS);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready. Press START.");
    }

    @Override
    public void loop() {
        // Enable/Disable Fast/Slow drive mode
        fastModeEnabled = gamepad1.right_bumper || gamepad2.right_bumper;

        // Drive Input (duplicated for Gamepad 1 and 2)
        if (!gamepad1.atRest()) {
            arcadeDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x);
        }
        else {
            arcadeDrive(-gamepad2.right_stick_y, gamepad2.right_stick_x);
        }

        /// Gamepad 1 - Specific values
        // Motor to set velocity
        if (gamepad1.dpad_up) {
            targetShooterRPM = MAX_SHOOTER_RPM;
        }
        else if (gamepad1.dpad_right) {
            targetShooterRPM = MID_SHOOTER_RPM;
        }
        else if (gamepad1.dpad_down) {
            targetShooterRPM = MIN_SHOOTER_RPM;
        }
        else if (gamepad1.dpad_left) {
            targetShooterRPM = SHOOTER_STOPPED;
        }

        // Servo Pos Control
        if (gamepad1.x) {
            targetServoPos = SERVO_START_POS;
        }
        else if (gamepad1.y) {
            targetServoPos = SERVO_MID_POS;
        }
        else if (gamepad1.b) {
            targetServoPos = SERVO_END_POS;
        }

        // Servo Speed Control
        if (gamepad1.left_trigger > 0.5) {
            targetCRServoSpeed = CRSERVO_FORWARD_SPEED;
        }
        else if (gamepad1.right_trigger > 0.5) {
            targetCRServoSpeed = CRSERVO_REVERSE_SPEED;
        }
        else if (gamepad1.left_bumper) {
            targetCRServoSpeed = CRSERVO_STOP_SPEED;
        }

        /// Gamepad 2 - Variable values
        // Motor to set velocity
        if (gamepad2.dpad_up) {
            targetShooterRPM += SHOOTER_INC_AND_DEC;
            targetShooterRPM = Range.clip(targetShooterRPM, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);
        }
        else if (gamepad2.dpad_right) {
            targetShooterRPM = MID_SHOOTER_RPM;
        }
        else if (gamepad2.dpad_down) {
            targetShooterRPM -= SHOOTER_INC_AND_DEC;
            targetShooterRPM = Range.clip(targetShooterRPM, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);
        }
        else if (gamepad2.dpad_left) {
            targetShooterRPM = SHOOTER_STOPPED;
        }

        // Servo Pos Control
        if (gamepad2.x) {
            targetServoPos -= SERVO_INC_AND_DEC;
            targetServoPos = Range.clip(targetServoPos, SERVO_START_POS, SERVO_END_POS);
        }
        else if (gamepad2.y) {
            targetServoPos = SERVO_MID_POS;
        }
        else if (gamepad2.b) {
            targetServoPos += SERVO_INC_AND_DEC;
            targetServoPos = Range.clip(targetServoPos, SERVO_START_POS, SERVO_END_POS);
        }

        // Servo Speed Control
        if (gamepad2.left_trigger > 0.5) {
            targetCRServoSpeed -= CRSERVO_INC_AND_DEC;
            targetCRServoSpeed = Range.clip(targetCRServoSpeed, CRSERVO_REVERSE_SPEED, CRSERVO_FORWARD_SPEED);
        }
        else if (gamepad2.right_trigger > 0.5) {
            targetCRServoSpeed += CRSERVO_INC_AND_DEC;
            targetCRServoSpeed = Range.clip(targetCRServoSpeed, CRSERVO_REVERSE_SPEED, CRSERVO_FORWARD_SPEED);
        }
        else if (gamepad2.left_bumper) {
            targetCRServoSpeed = CRSERVO_STOP_SPEED;
        }
        
        /// Telemetry Update
        telemetry.addData("Drive Left Power", leftDrive.getPower());
        telemetry.addData("Drive Right Power", rightDrive.getPower());
        telemetry.addData("Target Shooter RPM", targetShooterRPM);
        telemetry.addData("Target Servo Pos", targetServoPos);
        telemetry.addData("Target CR Servo Speed", targetCRServoSpeed);
        telemetry.update();


        /// Setting Motor and Servo positions/speeds
        double targetTicksPerSec = (targetShooterRPM / 60.0) * SHOOTER_TICKS_PER_SEC;
        shooter.setVelocity(targetTicksPerSec);
        servo.setPosition(targetServoPos);
        crServo.setPower(targetCRServoSpeed);

    }

    /**
     * Arcade drive on one stick.
     *
     * @param throttle Forward/ Backwards input
     * @param rotational Spin input
     */
    private void arcadeDrive(double throttle, double rotational) {
        // Calculate raw motor powers
        double leftPower  = throttle + rotational;
        double rightPower = throttle - rotational;

        // Squaring input for smoother control
        leftPower  = Math.copySign(leftPower * leftPower, leftPower);
        rightPower = Math.copySign(rightPower * rightPower, rightPower);

        // Clip the values to the range [-MAX_SLOW_SPEED, MAX_SLOW_SPEED] or [-1, 1], depending
        // on if fast mode is enabled or not.
        if (!fastModeEnabled) {
            leftPower  = Math.max(-MAX_SLOW_SPEED, Math.min(MAX_SLOW_SPEED, leftPower));
            rightPower = Math.max(-MAX_SLOW_SPEED, Math.min(MAX_SLOW_SPEED, rightPower));
        }
        else {
            leftPower  = Math.max(-1.0, Math.min(1.0, leftPower));
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower));
        }

        // Send power to motors
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}
