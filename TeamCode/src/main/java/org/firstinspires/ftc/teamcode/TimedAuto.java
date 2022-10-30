package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Timed Auto", group="Robot")
public class TimedAuto extends LinearOpMode {

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


        runtime.reset();
        double forward = -0.2;
        int seconds = 2;

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive() && (runtime.seconds() < seconds)) {
                LeftBack.setPower(forward);
                LeftFront.setPower(forward);
                RightBack.setPower(forward);
                RightFront.setPower(forward);
                telemetry.update();
            }
        }
    }
}