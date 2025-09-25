package org.firstinspires.ftc.teamcode.TESTING;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class actionTesting extends LinearOpMode {


    public class Claw {
        Servo leftClaw; //port 0
        Servo rightClaw; // port 1

        public Claw(HardwareMap hardwareMap) {
            leftClaw = hardwareMap.get(Servo.class, "leftClaw"); //port 0
            rightClaw = hardwareMap.get(Servo.class, "rightClaw"); //port 1
        }

            public class CloseClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftClaw.setPosition(0.7);
                    rightClaw.setPosition(0.1);

                    return false;
                }
            }
            public Action closeClaw() {
                return new CloseClaw();
            }

            public class OpenClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftClaw.setPosition(0.5);
                    rightClaw.setPosition(0.4);
                    sleep(500);

                    return false;
                }
            }
            public Action openClaw() {
                return new OpenClaw();
            }
    }

    public class Intake{
        DcMotor intake; //port ?

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
        }
            public class IntakeBall implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intake.setPower(0.5);
                    sleep(2500);
                    intake.setPower(0);

                    return false;
                }
            }
            public Action intakeBall() {
                return new IntakeBall();
            }

            public class OuttakeBall implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    intake.setPower(-0.5);
                    sleep(2500);
                    intake.setPower(0);

                    return false;
                }

            }
            public Action outtakeBall(){
                return new OuttakeBall();
            }
    }



    @Override
    public void runOpMode(){
        //call all motors, servos, and code from other classes
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));//START POSE!!!
        Pose2d traj2 = new Pose2d(30, 0, Math.toRadians(0));
        MecanumDrive drive = new PinpointDrive(hardwareMap, startPose);
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);

                                    //INIT PHASE//



        waitForStart();

                                    //RUN PHASE//

        Actions.runBlocking(
            drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(20, -20, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(intake.intakeBall())
                    .build()
        );
    }

    //action to return the slide to ground
    public class returnSlideToGround implements Action{
        DcMotor linearSlide;
        TouchSensor touch;

        public returnSlideToGround(DcMotor linearSlide, TouchSensor touch){
            this.linearSlide = linearSlide;
            this.touch = touch;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            if(touch.isPressed()){
                linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
                linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset whatever encoder count we're at to the "new 0"
            }
            else{
                linearSlide.setTargetPosition(0);
            }

            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(1);

            telemetry.addData("slide encoder",linearSlide.getCurrentPosition());
            telemetry.addData("touch state",touch.isPressed());
            telemetry.update();

            //we need to tell the action to keep running as long as the slide is still moving toward target
            if(linearSlide.isBusy() || touch.isPressed()==false) {
                return true;
            }
            else{
                return false;
            }

        }
    }


    //action to raise slide to "position" (# of encoder counts)
    public class raiseSlide implements Action{
        DcMotor linearSlide;
        int slidePosition;

        public raiseSlide(DcMotor linearSlide, int slidePosition){
            this.linearSlide = linearSlide;
            this.slidePosition = slidePosition;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            linearSlide.setTargetPosition(slidePosition);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(1);

            telemetry.addData("slide encoder", linearSlide.getCurrentPosition());
            telemetry.update();

            //we need to tell the action to keep running as long as the slide is still moving toward target
            if (linearSlide.isBusy()){
                return true;
            }
            else {
                return false;
            }
        }
    }


    public class wallGrabVariable implements Action{
        DcMotor linearSlide;
        Servo specimenClaw;
        int slidePos;

        public wallGrabVariable(DcMotor linearSlide, Servo specimenClaw, int slidePos){
            this.linearSlide = linearSlide;
            this.specimenClaw = specimenClaw;
            this.slidePos = slidePos;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            specimenClaw.setPosition(0.85);
            sleep(400);

            linearSlide.setTargetPosition(slidePos);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(1);

            //we need to tell the action to keep running as long as the slide is still moving toward target
            if (linearSlide.isBusy()){
                return true;
            }
            else {
                return false;
            }
        }
    }

    //action to grab a specimen off of the wall (grab + lift)
    public class wallGrab implements Action{
        DcMotor linearSlide;
        Servo specimenClaw;

        public wallGrab(DcMotor linearSlide, Servo specimenClaw){
            this.linearSlide = linearSlide;
            this.specimenClaw = specimenClaw;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            specimenClaw.setPosition(0.85);
            sleep(400);

            linearSlide.setTargetPosition(240);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(1);

            //we need to tell the action to keep running as long as the slide is still moving toward target
            if (linearSlide.isBusy()){
                return true;
            }
            else {
                return false;
            }
        }
    }

}


