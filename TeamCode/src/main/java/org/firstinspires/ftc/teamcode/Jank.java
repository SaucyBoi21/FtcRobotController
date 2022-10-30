package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Jank")
public class Jank extends LinearOpMode {

    private DcMotor RightFront;
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightBack;
    private DcMotor Lift;

    private Servo clawR = null;
    private Servo clawL = null;


    private int liftAmount = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
//
//        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        Sleeve sleeveDetection = new Sleeve();
//
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(cam);
//
//        camera.setPipeline(sleeveDetection);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });
        // Put initialization blocks here.
        waitForStart();
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        clawL = hardwareMap.get(Servo.class, "ClawLeft");
        clawR = hardwareMap.get(Servo.class, "ClawRight");

        clawL.setDirection(Servo.Direction.FORWARD);
        clawR.setDirection(Servo.Direction.REVERSE);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                if (gamepad2.dpad_up) {
                    liftAmount += 1;
                    Lift.setPower(0.8);
                } else if (gamepad2.dpad_down) {
                    liftAmount -= 1;
                    Lift.setPower(-0.4);
                } else {
                    if (liftAmount > 10) {
                        Lift.setPower(.1);
                    } else {
                        Lift.setPower(0);
                    }
                }
                if (gamepad2.b) {
                    clawL.setPosition(1);
                    clawR.setPosition(1);
                } else if (gamepad2.a) {
                    // lower value is squeeze harder
                    clawL.setPosition(.85);
                    clawR.setPosition(.85);
                }

                LeftBack.setPower(gamepad1.left_stick_y);
                LeftFront.setPower(gamepad1.left_stick_y);
                RightBack.setPower(gamepad1.right_stick_y);
                RightFront.setPower(gamepad1.right_stick_y);
                LeftBack.setPower(gamepad1.right_trigger);
                LeftFront.setPower(-gamepad1.right_trigger);
                RightBack.setPower(-gamepad1.right_trigger);
                RightFront.setPower(gamepad1.right_trigger);
                LeftBack.setPower(-gamepad1.left_trigger);
                LeftFront.setPower(gamepad1.left_trigger);
                RightBack.setPower(gamepad1.left_trigger);
                RightFront.setPower(-gamepad1.left_trigger);
//                Lift.setPower(-gamepad2.left_trigger);
                telemetry.update();
            }
        }
    }
}