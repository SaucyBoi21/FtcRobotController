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

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
public class EncoderAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LeftBack   = null;
    private DcMotor         LeftFront  = null;
    private DcMotor         RightFront = null;
    private DcMotor         RightBack = null;
    private DcMotor         Lift = null;
    private Servo           RightServo = null;
    private Servo           LeftServo = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.2;
    static final double LIFT_UP = 5;
    static final double LIFT_DOWN = -5;
    static final double OPEN = 0.1;
    static final double CLOSED = 0.85;

    @Override
    public void runOpMode() {
               WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
                  Sleeve sleeveDetection = new Sleeve();
//
            OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(cam);
//
           camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
          public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                camera.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {}
        });
        sleep(1000);
        Sleeve.ParkingPosition pos = Sleeve.ParkingPosition.LEFT;

        // Initialize the drive system variables.
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        RightServo = hardwareMap.get(Servo.class, "ClawRight");
        LeftServo = hardwareMap.get(Servo.class, "ClawLeft");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        RightFront.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset

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

        for (int i = 0; i < 15; i++) {
            sleep(250);
            Sleeve.ParkingPosition p = sleeveDetection.getPosition();
            System.out.println(p);
            pos = p;
        }
        // I'll code the actual positions to go to in one of my classes. Also, the vision program I found returns the variable "position" as either 1 2 or 3z
         if(pos == Sleeve.ParkingPosition.LEFT){
        encoderDrive(TURN_SPEED,  4, 12, 12 , 1.0);  // turn to cones

         } else if(pos == Sleeve.ParkingPosition.RIGHT){
        encoderDrive(TURN_SPEED,  -4, 120, 12 , 1.0);
         } else if(pos == Sleeve.ParkingPosition.CENTER){
        encoderDrive(TURN_SPEED,  0, 12, 12 , 1.0);
//         go to pos 3
         }

        for (int i = 0; i < 150; i++) {
            sleep(250);
            Sleeve.ParkingPosition p = sleeveDetection.getPosition();
            System.out.println(p);
        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    public void encoderDrive(double speed,  double LeftFrontInches, double RightBackInches, double RightFrontInches,double timeoutS) {
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newRightBackTarget = (int) ((double) RightBack.getCurrentPosition() / COUNTS_PER_INCH + (int)(RightBackInches * COUNTS_PER_INCH));
            newLeftFrontTarget = (int) ((double) LeftFront.getCurrentPosition() / COUNTS_PER_INCH + (int)(LeftFrontInches * COUNTS_PER_INCH));
            newRightFrontTarget = (int) ((double) RightFront.getCurrentPosition() / COUNTS_PER_INCH + (int)(RightFrontInches * COUNTS_PER_INCH));
            RightBack.setTargetPosition(newRightBackTarget * 3);
            LeftFront.setTargetPosition(newLeftFrontTarget * 3);
            RightFront.setTargetPosition(newRightFrontTarget * 3);

            // Turn On RUN_TO_POSITION
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
                telemetry.update();
            }

            // Stop all motion;
            LeftFront.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.


        }
    }
}
