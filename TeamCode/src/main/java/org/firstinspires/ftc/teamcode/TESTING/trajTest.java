package org.firstinspires.ftc.teamcode.TESTING;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import androidx.annotation.NonNull;

@Autonomous
public class trajTest extends LinearOpMode {


    double ClawOpen = 0.4;
    double ClawClosed = 0.85;

    double ClawArmUp = .86;
    double ClawArmHold = .6;
    double ClawArmDown = .5;

    @Override
    public void runOpMode(){
        //init and call all motors and servos
        Pose2d startPose = new Pose2d(50.375, 66.5, Math.toRadians(0)); //START POSE!!!
        MecanumDrive drive = new PinpointDrive(hardwareMap, startPose);

        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        Servo specimenClaw = hardwareMap.get(Servo.class, "specimen_claw" );
        Servo clawArm = hardwareMap.get(Servo.class, "claw_arm" );

                         //INIT PHASE//
        specimenClaw.setPosition(ClawClosed);
        clawArm.setPosition(ClawArmUp);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setPower(0);


        waitForStart();


        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .stopAndAdd(new clawArmUp(clawArm))
                        .setTangent(Math.toRadians(270))
                        .afterDisp(40, new raiseSlide(linearSlide, 1000))
                        .splineToConstantHeading(new Vector2d(50,-40), Math.toRadians(270))

                        .build());

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
        public boolean run(@NonNull TelemetryPacket telemetryPacket) { //write your action in here!
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