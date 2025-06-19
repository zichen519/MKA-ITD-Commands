package org.firstinspires.ftc.teamcode.example.java.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.example.java.commands.EndEffectorPositions;
import org.firstinspires.ftc.teamcode.example.java.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.example.java.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

@Config
@Autonomous
public class SampleAuto extends PedroOpMode {
    public SampleAuto(){
        super(
                Claw.INSTANCE,
                Elbow.INSTANCE,
                Lift.INSTANCE,
                Linkage.INSTANCE,
                Rotate.INSTANCE,
                Wrist.INSTANCE
        );
    }

    public static double pickupX;
    public static double pickupY;
    private final Pose startPose = new Pose(6.500, 107.5, Math.toRadians(0.0));
    private final Pose scorePose = new Pose(17.5,121, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(25.5,115.25,Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(25.5, 124.5, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(28, 119.5, Math.toRadians(45));
    private final Pose pickupSubPose = new Pose(66.5, 88, Math.toRadians(-90));

    private PathChain scorePreload, grab1, grab2, grab3, grabSub, score1, score2, score3, score4;

    public void buildPaths(){

        scorePreload = follower.pathBuilder().addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        grab1 = follower.pathBuilder().addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1Pose.getHeading()).build();
        score1 = follower.pathBuilder().addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(),scorePose.getHeading()).build();
        grab2 = follower.pathBuilder().addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup2Pose.getHeading()).build();
        score2 = follower.pathBuilder().addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),scorePose.getHeading()).build();
        grab3 = follower.pathBuilder().addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup3Pose.getHeading()).setPathEndTimeoutConstraint(100)
                .build();
        score3 = follower.pathBuilder().addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(),scorePose.getHeading()).build();
    }

    public Command firstRoutine(){

        return new SequentialGroup(

                //score preload
                new ParallelGroup(
                        new FollowPath(scorePreload, true, 0.8),
                        Lift.INSTANCE.toHighBasket(),
                        EndEffectorPositions.avoidBasket()
                ),
                new ParallelGroup(
                        EndEffectorPositions.basketScore(),
                        new SequentialGroup(
                                new Delay(0.4),
                                Claw.INSTANCE.open()

                        )

                ),

                new Delay(.4),
                EndEffectorPositions.avoidBasket(),
                new Delay(.4),
                Lift.INSTANCE.retract(),

                //pickup 1


                new ParallelGroup(
                        EndEffectorPositions.hoverAboveFloor(),
                        Linkage.INSTANCE.linkageDown(),
                        Rotate.INSTANCE.setPosition(0.23)

                ),

                new FollowPath(grab1,true, 0.8),
                new Delay(0.4),
                EndEffectorPositions.grabFromFloor(),

                //score spike 1
                new ParallelGroup(
                        new FollowPath(score1, true, 0.8),
                        new SequentialGroup(
                                Linkage.INSTANCE.linkageUp(),
                                Lift.INSTANCE.toHighBasket()
                        ),
                        EndEffectorPositions.avoidBasket()
                ),
                new ParallelGroup(
                        EndEffectorPositions.basketScore(),
                        new SequentialGroup(
                                new Delay(0.4),
                                Claw.INSTANCE.open()

                        )

                ),
                new Delay(.4),
                EndEffectorPositions.avoidBasket(),
                new Delay(.4),
                Lift.INSTANCE.retract(),

                //Pickup 2
                new ParallelGroup(
                    EndEffectorPositions.hoverAboveFloor(),
                    Linkage.INSTANCE.linkageDown(),
                    Rotate.INSTANCE.setPosition(0.23)

                ),
                new FollowPath(grab2,true, 0.8),
                new Delay(0.4),
                EndEffectorPositions.grabFromFloor(),

                //Score spike 2
                new ParallelGroup(
                        new FollowPath(score2, true, 0.8),
                        new SequentialGroup(
                                Linkage.INSTANCE.linkageUp(),
                                Lift.INSTANCE.toHighBasket()
                        ),
                        EndEffectorPositions.avoidBasket()
                ),
                new ParallelGroup(
                        EndEffectorPositions.basketScore(),
                        new SequentialGroup(
                                new Delay(0.4),
                                Claw.INSTANCE.open()

                        )

                ),
                new Delay(.4),
                EndEffectorPositions.avoidBasket(),
                new Delay(.4),
                Lift.INSTANCE.retract(),

                //pickup 3
                new ParallelGroup(
                        EndEffectorPositions.hoverAboveFloor(),
                        Linkage.INSTANCE.linkageDown(),
                        Rotate.INSTANCE.setPosition(0.39)

                ),
                new FollowPath(grab3,true, 0.8),
                new Delay(0.4),
                EndEffectorPositions.grabFromFloor(),

                //score 3
                new ParallelGroup(
                        new FollowPath(score2, true, 0.8),
                        new SequentialGroup(
                                Linkage.INSTANCE.linkageUp(),
                                Lift.INSTANCE.toHighBasket()
                        ),
                        EndEffectorPositions.avoidBasket()
                ),
                new ParallelGroup(
                        EndEffectorPositions.basketScore(),
                        new SequentialGroup(
                                new Delay(0.4),
                                Claw.INSTANCE.open()

                        )

                ),
                new Delay(.4),
                EndEffectorPositions.avoidBasket(),
                new Delay(.4),
                Lift.INSTANCE.retract()
        );

    }



    @Override
    public void onInit(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OpModeData.telemetry = telemetry;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        Claw.INSTANCE.close().invoke();
    }

    @Override
    public void onStartButtonPressed(){
        firstRoutine().invoke();

    }
}
