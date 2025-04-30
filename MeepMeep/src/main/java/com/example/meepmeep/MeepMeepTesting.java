package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double normalVel = 30;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-11.4, 66.5, Math.toRadians(0)))//START POSE!!!!!


                .splineToConstantHeading(new Vector2d(1.5, 29), Math.toRadians(270),
                        new TranslationalVelConstraint(30))

              //  .stopAndAdd(new OpenClaw(specimenClaw))

                //push 2 preset blue samples into the observation zone
                //.afterDisp(7, new raiseSlide(linearSlide, 0))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-13,46, Math.toRadians(90)), Math.toRadians(180),
                        new TranslationalVelConstraint(25))

                .splineToSplineHeading(new Pose2d(-33,28, Math.toRadians(180)), Math.toRadians(270),
                        new TranslationalVelConstraint(25))

                .splineToConstantHeading(new Vector2d(-33,16), Math.toRadians(270),
                        new TranslationalVelConstraint(25))

                .splineToConstantHeading(new Vector2d(-40,8), Math.toRadians(180),
                        new TranslationalVelConstraint(normalVel))

                .splineToConstantHeading(new Vector2d(-45,16), Math.toRadians(90),
                        new TranslationalVelConstraint(normalVel))

                .splineToConstantHeading(new Vector2d(-45,53), Math.toRadians(90),
                        new TranslationalVelConstraint(normalVel))

                //going out to push second sample
                .splineToConstantHeading(new Vector2d(-48,16), Math.toRadians(270),
                        new TranslationalVelConstraint(normalVel))

                .splineToConstantHeading(new Vector2d(-51,12), Math.toRadians(180),
                        new TranslationalVelConstraint(normalVel))

                .splineToConstantHeading(new Vector2d(-55,16), Math.toRadians(90),
                        new TranslationalVelConstraint(normalVel))

                .splineToConstantHeading(new Vector2d(-55,56), Math.toRadians(90),
                        new TranslationalVelConstraint(normalVel))

                //slow down to get into position for second sample grab
                .splineToConstantHeading(new Vector2d(-38,50), Math.toRadians(15),
                        new TranslationalVelConstraint(25.0))

                //GRAB SECOND SAMPLE!!
               // .afterDisp(17, new wallGrab(linearSlide, specimenClaw))
                .splineToConstantHeading(new Vector2d(-30,68.3), Math.toRadians(90),
                        new TranslationalVelConstraint(15))

                //go to hang second specimen
                //.afterDisp(1.5, new raiseSlide(linearSlide, 1800))

                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-23,68.3), Math.toRadians(0),
                        new TranslationalVelConstraint(25))

                .splineToSplineHeading(new Pose2d(-1,30, Math.toRadians(0)), Math.toRadians(270),
                        new TranslationalVelConstraint(22))

                //HANG SECOND SPECIMEN
                //.stopAndAdd(new raiseSlide(linearSlide, 875))
               // .stopAndAdd(new OpenClaw(specimenClaw))

                //traj2 START

                //go to grab third specimen
                .setTangent(Math.toRadians(270))
                //.afterDisp(3, new raiseSlide(linearSlide, 0))
                .splineToConstantHeading(new Vector2d(-7,43), Math.toRadians(160),
                        new TranslationalVelConstraint(25))

                .splineToSplineHeading(new Pose2d(-29,62, Math.toRadians(180)), Math.toRadians(90),
                        new TranslationalVelConstraint(22))

                //slow down to get into position for third sample grab
               // .afterDisp(9, new wallGrab(linearSlide, specimenClaw))
                .splineToConstantHeading(new Vector2d(-29,67), Math.toRadians(90),
                        new TranslationalVelConstraint(15))


                //go to hang third specimen
                //.afterDisp(7, new raiseSlide(linearSlide, 1800))

                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(29, 64), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}