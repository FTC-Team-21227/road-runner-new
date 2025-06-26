package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d initialPose = new Pose2d(0, 92-90, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(90), Math.toRadians(90), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(8, 112.5-90), Math.toRadians(-45))// loaded sample go to basket
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(8.4, 107-90), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, 112.5-90), Math.toRadians(-45)) //go away from wall bec arms lifting
                .waitSeconds(0.7)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(8.6, 119-90), Math.toRadians(1.5))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, 112.5-90), Math.toRadians(-45)) //go away from wall bec arms lifting
                .waitSeconds(0.7)
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(10.5,118-90),Math.toRadians(10))
                .waitSeconds(0.3)
                .turnTo(Math.toRadians(28))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(7, 113.5-90), Math.toRadians(-45)) //go away from wall bec arms lifting
                .waitSeconds(0.7)
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(64.5,105-90),Math.toRadians(-90), new TranslationalVelConstraint(70))
//                .waitSeconds(1.5) //get 4th sample from the sub
                .strafeTo(new Vector2d(64.5, 90/*+2*/-90), new TranslationalVelConstraint(30))
                .waitSeconds(0.3) //change0.5
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(64.5,100-90),Math.toRadians(180), new TranslationalVelConstraint(70))
                .splineToSplineHeading(new Pose2d(6, 108-90, Math.toRadians(-45)), Math.toRadians(165),new TranslationalVelConstraint(70)) //go away from wall bec arms lifting
                .waitSeconds(0.7)
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(64.5,105-90),Math.toRadians(-90), new TranslationalVelConstraint(70))
//                .waitSeconds(1.5) //get 4th sample from the sub
                .strafeTo(new Vector2d(64.5, 90/*+2*/-90), new TranslationalVelConstraint(30)) //get 4th sample
//                .strafeTo(new Vector2d(64.5+Y,90-X),new TranslationalVelConstraint(30));
                .waitSeconds(0.3) //change0.5
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(64.5,100-90),Math.toRadians(180), new TranslationalVelConstraint(70))
                .splineToSplineHeading(new Pose2d(6, 108-90, Math.toRadians(-45)), Math.toRadians(165),new TranslationalVelConstraint(70)) //go away from wall bec arms lifting
                .waitSeconds(0.7)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(52.5, 105-90, Math.toRadians(-90)),Math.toRadians(0),new TranslationalVelConstraint(70))//avoid bumping into submersible
                .splineToConstantHeading(new Vector2d(52.5, 87-90),Math.toRadians(0),new TranslationalVelConstraint(15))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

