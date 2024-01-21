        package org.firstinspires.ftc.teamcode;
        import com.acmerobotics.roadrunner.*;
        import com.acmerobotics.roadrunner.ftc.Actions;
        import com.qualcomm.hardware.dfrobot.HuskyLens;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.internal.system.Deadline;

        import java.lang.Math;
        import java.util.concurrent.TimeUnit;

@Autonomous(name="AutoTournament")
public class AutoTournament extends LinearOpMode {
    private final int READ_PERIOD = 2;
    private HuskyLens huskyLens;
    String mode = "TAG";
    String pos;
    int location = 0;
    private Deadline rateLimit;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor8");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor7");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorIntake = hardwareMap.dcMotor.get("motor5");
        DcMotorEx motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorEx motorSlideRight = hardwareMap.get(DcMotorEx.class, "motor6");
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor motorLauncher = hardwareMap.dcMotor.get("motor4");
        Servo servoClamp = hardwareMap.servo.get("servo1");
        Servo servoHOT = hardwareMap.servo.get("servo5"); //Hook ot
        Servo servoFOT = hardwareMap.servo.get("servo4"); // flap ot
        Servo servoTOT = hardwareMap.servo.get("servo2"); // top ot
        Servo servoBOT = hardwareMap.servo.get("servo3"); // bottom ot
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        //Middle Movement
        Action startM =
                drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(25,28))
                        .build();
        startPose = new Pose2d(25, 28, 180);
        Action toBoardM =
                drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(10,2))
//                        .lineToY(7)
//                        .lineToX(20)
//                        .turnTo(90)
//                        .lineToY(-2)
                        .build();


        //Right Movement
        Action startR =
                drive.actionBuilder(startPose)
                        .lineToX(10)
                        .lineToY(10)
                        .build();
        Action toBoardR =
                drive.actionBuilder(startPose)
                        .lineToY(7)
                        .lineToX(20)
                        .turnTo(90)
                        .lineToY(2)
                        .build();


        //Left Movement
        Action startL =
                drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(26,22))
                        .build();
        startPose = new Pose2d(15, 5, 180);
        Action toBoardL =
                drive.actionBuilder(startPose)
                        .lineToY(5)
                        .lineToX(20)
                        .turnTo(90)
                        .lineToY(-2)
                        .build();


        //Common Movement
        Action move =
                drive.actionBuilder(startPose)
                        .lineToX(10)
                        .waitSeconds(0.5)
                        .lineToY(2)
                        .build();
        Action park =
                drive.actionBuilder(startPose)
                        .lineToY(2)
                        .lineToX(15)
                        .build();


        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        waitForStart();
        servoClamp.setPosition(0.6);
        motorSlideLeft.setTargetPosition(0);
        motorSlideRight.setTargetPosition(0);
        servoTOT.setPosition(0.83);
        servoBOT.setPosition(0.23);
        servoFOT.setPosition(0.08);
        servoHOT.setPosition(0.67);
        long start = System.currentTimeMillis();
        long end = start + 2000;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            telemetry.update();
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (blocks[i].x <= 100) {
                    telemetry.addData("Pos:", "Left");
                    telemetry.update();
                    location = 3;
                } else if (blocks[i].x > 100 && blocks[i].x <= 200) {
                    telemetry.addData("Pos:", "Middle");
                    telemetry.update();
                    location = 2;
                } else if (blocks[i].x > 200) {
                    telemetry.addData("Pos:", "Right");
                    telemetry.update();
                    location = 1;
                }
            }
            if (blocks.length == 0) {
                location = 2;
            }
            if (location != 0) {
                break;
            }
        }


        if (location == 2) {
            Actions.runBlocking(startM);
            servoClamp.setPosition(0.15);
            sleep(20000);
//            Actions.runBlocking(toBoardM);

            //Viper Slides Up
//            motorSlideRight.setTargetPosition(1000);
//            motorSlideLeft.setTargetPosition(1000);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(250);
//            servoTOT.setPosition(0.54);
//            servoBOT.setPosition(0);
//            sleep(1200);
//            motorSlideRight.setTargetPosition(100);
//            motorSlideLeft.setTargetPosition(100);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);

            //Yellow Pixel Drop Sequence
//            sleep(3400);
//            drive.followTrajectorySequence(move);
//            sleep(300);
//            servoFOT.setPosition(0.70);
//            sleep(200);
//            drive.followTrajectorySequence(park);

            //Viper Slides Down
//            motorSlideRight.setTargetPosition(1000);
//            motorSlideLeft.setTargetPosition(1000);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            servoTOT.setPosition(0.83);
//            servoBOT.setPosition(0.23);
//            servoFOT.setPosition(0.57);
//            servoHOT.setPosition(0.52);
//            sleep(2000);
//            motorSlideRight.setTargetPosition(0);
//            motorSlideLeft.setTargetPosition(0);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(1000);
//        }
//        else if (location == 1) {
//            drive.followTrajectorySequence(startR);
//            motorIntake.setPower(-0.45);
//            drive.followTrajectorySequence(backR);
//            motorIntake.setPower(0);
//            drive.followTrajectorySequence(moveR);
//
//            //END PART DO NOT CHANGE
//            motorSlideRight.setTargetPosition(1000);
//            motorSlideLeft.setTargetPosition(1000);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(250);
//            servoTOT.setPosition(0.54);
//            servoBOT.setPosition(0);
//            sleep(1200);
//            motorSlideRight.setTargetPosition(100);
//            motorSlideLeft.setTargetPosition(100);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(3400);
//            drive.followTrajectorySequence(leftR);
//            sleep(300);
//            servoFOT.setPosition(0.72);
//            sleep(200);
//            drive.followTrajectorySequence(backwardR);
//            motorSlideRight.setTargetPosition(1000);
//            motorSlideLeft.setTargetPosition(1000);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            servoTOT.setPosition(0.83);
//            servoBOT.setPosition(0.23);
//            servoFOT.setPosition(0.57);
//            servoHOT.setPosition(0.52);
//            sleep(2000);
//            motorSlideRight.setTargetPosition(0);
//            motorSlideLeft.setTargetPosition(0);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(1000);
//        } else if (location == 3) {
//            drive.followTrajectorySequence(startL);
////                motorIntake.setPower(-0.45);
//            drive.followTrajectorySequence(backL);
////                motorIntake.setPower(0);
//            drive.followTrajectorySequence(moveL);
//
//            //END PART DO NOT CHANGE
//            motorSlideRight.setTargetPosition(1000);
//            motorSlideLeft.setTargetPosition(1000);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(250);
//            servoTOT.setPosition(0.54);
//            servoBOT.setPosition(0);
//            sleep(1200);
//            motorSlideRight.setTargetPosition(100);
//            motorSlideLeft.setTargetPosition(100);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(3400);
//            drive.followTrajectorySequence(leftL);
//            sleep(300);
//            servoFOT.setPosition(0.72);
//            sleep(200);
//            drive.followTrajectorySequence(backwardL);
//            motorSlideRight.setTargetPosition(1000);
//            motorSlideLeft.setTargetPosition(1000);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            servoTOT.setPosition(0.83);
//            servoBOT.setPosition(0.23);
//            servoFOT.setPosition(0.57);
//            servoHOT.setPosition(0.52);
//            sleep(2000);
//            motorSlideRight.setTargetPosition(0);
//            motorSlideLeft.setTargetPosition(0);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(1000);
//            motorSlideLeft.setVelocity(1000);
//            sleep(1000);
        }
    }
}

