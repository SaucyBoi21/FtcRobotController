package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Sleeve;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlankAuto", group="Robot")
public class BlankAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor LeftBack = null;
    private DcMotor LeftFront = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor Lift = null;
    private Servo RightServo = null;
    private Servo LeftServo = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;
    static final double LIFT_UP = 5;
    static final double LIFT_DOWN = -5;
    static final double OPEN = 0.1;
    static final double CLOSED = 0.85;

    @Override
    public void runOpMode() {


        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // speed,  LF, RB, RF, time, lift
//        Lift.setPower(0.5);
//        encoderDrive(DRIVE_SPEED, 0, 57, 57, 1.0);  // go to pole off start
//        encoderDrive(TURN_SPEED,  -1, 1,  0, 1.0);  // turn towards pole
//        // drop cone
//        RightServo.setPosition(OPEN);
//        LeftServo.setPosition(OPEN);
//
//
//
//        encoderDrive(TURN_SPEED, 4, -4, 0 , 1.0);  // turn to cones
//        Lift.setPower(-0.3);
//        encoderDrive(TURN_SPEED,   0, 33, 33 , 1.0);  // go to cones
//        // get cone
//        RightServo.setPosition(CLOSED);
//        LeftServo.setPosition(CLOSED);
//
//
//        encoderDrive(TURN_SPEED,  4, -4, 0, 1.0); // turn to pole
//        Lift.setPower(0.5);
//        encoderDrive(TURN_SPEED,   0, 33, 33 , 1.0);  // go to pole
//        // drop cone
//        RightServo.setPosition(OPEN);
//        LeftServo.setPosition(OPEN);
//
//
//        encoderDrive(TURN_SPEED,  -4, 4, 0 , 1.0);  // turn to cones
//        Lift.setPower(-0.3);
//        encoderDrive(TURN_SPEED,   0, 33, 33 , 1.0);  // go to cones
//        // get cone
//        RightServo.setPosition(CLOSED);
//        LeftServo.setPosition(CLOSED);
//
//
//        encoderDrive(TURN_SPEED,  4, -4, 0, 1.0); // turn to pole
//        Lift.setPower(0.5);
//        encoderDrive(TURN_SPEED,   0, 33, 33 , 1.0);  // go to pole
//        // drop cone
//        RightServo.setPosition(OPEN);
//        LeftServo.setPosition(OPEN);
//
//
//        encoderDrive(TURN_SPEED, -4, 4, 4 , 1.0);  // turn to cones
//        Lift.setPower(-0.3);
//        encoderDrive(TURN_SPEED,   0, 33, 33 , 1.0);  // go to cones
//        // grab cone
//        RightServo.setPosition(CLOSED);
//        LeftServo.setPosition(CLOSED);
//
//
//        encoderDrive(TURN_SPEED,  4, -4, -4, 1.0); // turn to pole
//        Lift.setPower(0.5);
//        encoderDrive(TURN_SPEED,   0, 18.5, 18.5 , 1.0);  // go to pole
//        // drop cone
//        RightServo.setPosition(OPEN);
//        LeftServo.setPosition(OPEN);


        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
}