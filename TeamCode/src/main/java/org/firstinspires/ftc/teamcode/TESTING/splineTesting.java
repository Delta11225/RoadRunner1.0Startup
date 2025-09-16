package org.firstinspires.ftc.teamcode.TESTING;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Autonomous
public class splineTesting extends LinearOpMode {


    double ClawOpen = 0.4;
    double ClawClosed = 0.85;
    double ClawArmUp = .86;


    @Override
    public void runOpMode(){
        //call all motors, servos, and code from other classes
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0)); //START POSE!!!
        MecanumDrive drive = new PinpointDrive(hardwareMap, startPose);

        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        Servo specimenClaw = hardwareMap.get(Servo.class, "specimen_claw" );
        Servo clawArm = hardwareMap.get(Servo.class, "claw_arm" );
        TouchSensor touch = hardwareMap.get(TouchSensor.class, "touch_sensor");

                                    //INIT PHASE//

        specimenClaw.setPosition(ClawClosed);
        clawArm.setPosition(ClawArmUp);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setPower(0);



        waitForStart();

                                    //RUN PHASE//

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(6, 16 ), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(12,30), Math.toRadians(90))
                        .build());

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

    //action to open claw
    public class clawArmUp implements Action {
        Servo clawArm;

        public clawArmUp(Servo clawArm){
            this.clawArm = clawArm;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) { //write your action in here!
            clawArm.setPosition(0.86);
            return false;
        }
    }

    //action to open claw
    public class OpenClaw implements Action {
        Servo specimenClaw;

        public OpenClaw(Servo specimenClaw){
            this.specimenClaw = specimenClaw;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //write your action in here!
            specimenClaw.setPosition(0.4);
            return false;
        }
    }

    //action to open claw
    public class CloseClaw implements Action {
        Servo specimenClaw;

        public CloseClaw(Servo specimenClaw){
            this.specimenClaw = specimenClaw;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) { //write your action in here!
            specimenClaw.setPosition(0.85);
            return false;
        }
    }



}