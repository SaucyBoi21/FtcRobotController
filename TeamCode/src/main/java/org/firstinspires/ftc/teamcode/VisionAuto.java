package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Vision Auto", group="Robot")
public class VisionAuto extends LinearOpMode {

    private DcMotor RightFront;
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightBack;
    private DcMotor Lift;

    private Servo clawR = null;
    private Servo clawL = null;

    private ElapsedTime runtime = new ElapsedTime();


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

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        Sleeve sleeveDetection = new Sleeve();

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(cam);

        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        // Put initialization blocks here.
        waitForStart();
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

//        clawL = hardwareMap.get(Servo.class, "ClawLeft");
//        clawR = hardwareMap.get(Servo.class, "ClawRight");
//
//        clawL.setDirection(Servo.Direction.FORWARD);
//        clawR.setDirection(Servo.Direction.REVERSE);

        double forward = -0.2;
        double strafePower = 0.3;
        // initial move forward
        int seconds = 2;
        // strafe period
        int secondsStrafe = 2;

        int leftAmt = 0;
        int centerAmt = 0;
        int rightAmt = 0;

        runtime.reset();

        boolean init = true;

            // Put run blocks here.
        while (opModeIsActive()) {
            if (init) {
                for (int i = 0; i < 20; i++) {
                    switch (sleeveDetection.getPosition()) {
                        case LEFT:
                            leftAmt++;
                            break;
                        case CENTER:
                            centerAmt++;
                            break;
                        case RIGHT:
                            rightAmt++;
                            break;
                    }

                }
                init = false;
            }

//                System.out.println(sleeveDetection.getPosition());
           // left parking
            if (leftAmt > centerAmt && leftAmt > rightAmt) {
                // forward
                if (runtime.seconds() < seconds) {
                    LeftBack.setPower(forward);
                    LeftFront.setPower(forward);
                    RightBack.setPower(forward);
                    RightFront.setPower(forward);
                }
                // strafe
                if (runtime.seconds() < seconds + secondsStrafe && runtime.seconds() > seconds) {
                    LeftBack.setPower(-strafePower);
                    LeftFront.setPower(strafePower);
                    RightBack.setPower(strafePower);
                    RightFront.setPower(-strafePower);
                }
            }
            // center/straight forward
            if (centerAmt > leftAmt && centerAmt > rightAmt) {
                if (runtime.seconds() < seconds) {
                    LeftBack.setPower(forward);
                    LeftFront.setPower(forward);
                    RightBack.setPower(forward);
                    RightFront.setPower(forward);
                }
            }
            // right parking
            if (rightAmt > leftAmt && rightAmt > centerAmt) {
                // forward
                if (runtime.seconds() < seconds) {
                    LeftBack.setPower(forward);
                    LeftFront.setPower(forward);
                    RightBack.setPower(forward);
                    RightFront.setPower(forward);
                }
                // strafe
                if (runtime.seconds() < seconds + secondsStrafe && runtime.seconds() > seconds) {
                    LeftBack.setPower(strafePower);
                    LeftFront.setPower(-strafePower);
                    RightBack.setPower(-strafePower);
                    RightFront.setPower(strafePower);
                }
            }
            if (runtime.seconds() > seconds + secondsStrafe) {
                break;
            }
            telemetry.update();
        }
    }
}
