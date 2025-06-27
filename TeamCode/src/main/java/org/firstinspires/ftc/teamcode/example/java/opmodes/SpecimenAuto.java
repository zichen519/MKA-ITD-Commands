package org.firstinspires.ftc.teamcode.example.java.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
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
@Autonomous
@Config
public class SpecimenAuto extends PedroOpMode {
    public SpecimenAuto(){
        super(
                Claw.INSTANCE,
                Elbow.INSTANCE,
                Lift.INSTANCE,
                Linkage.INSTANCE,
                Rotate.INSTANCE,
                Wrist.INSTANCE
        );
    }

    private final Pose startPose = new Pose(6.5,66, Math.toRadians(0.0));
    private final Pose scorePose1 = new Pose(41, 75, Math.toRadians(0));
    private final Pose pushPose1 = new Pose(57, 23, Math.toRadians(0));
    private final Pose humanPose1 = new Pose(17, 23, Math.toRadians(0));
    private final Pose pushPose2 = new Pose(52, 16, Math.toRadians(0));
    private final Pose humanPose2 = new Pose(17, 16, Math.toRadians(0));
    private final Pose humanPose3 = new Pose(14, 9, Math.toRadians(0));
    private final Pose grabPose1 = new Pose(11.875,9, Math.toRadians(0));
    private final Pose grabPose = new Pose(11.875,32, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(42,74, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(42,72, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(42,70, Math.toRadians(0));
    private final Pose scorePose5 = new Pose(42,68, Math.toRadians(0));
    private final Pose parkPose = new Pose(11,24, Math.toRadians(0));

    private PathChain score1, push1, human1, push2, human2, human3, grab1, score2, grab2, score3, grab3, score4, grab4, score5, park;

    public void buildPaths(){
        score1 = follower.pathBuilder().addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        push1 = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose1), new Point(8.000, 39.000, Point.CARTESIAN),
                        new Point(60.000, 38.000, Point.CARTESIAN), new Point(pushPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(.97).build();
        human1 = follower.pathBuilder().addPath(new BezierLine(new Point(pushPose1), new Point(humanPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(.97).build();
        push2 = follower.pathBuilder().addPath(new BezierCurve(new Point(humanPose1), new Point(53, 27.000, Point.CARTESIAN), new Point(pushPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(.97).build();
        human2 = follower.pathBuilder().addPath(new BezierLine(new Point(pushPose2), new Point(humanPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(.97).build();
        human3 = follower.pathBuilder().addPath(new BezierCurve(new Point(humanPose2), new Point(85, 6, Point.CARTESIAN), new Point(grabPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(.97).build();

        score2 = follower.pathBuilder().addPath(new BezierCurve(new Point(grabPose1), new Point(33,31, Point.CARTESIAN), new Point(16.5, 70, Point.CARTESIAN),new Point(scorePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        grab2 = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose2), new Point(17, 74, Point.CARTESIAN),
                        new Point(27.5, 36, Point.CARTESIAN), new Point(grabPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        score3 = follower.pathBuilder().addPath(new BezierCurve(new Point(grabPose), new Point(27.5, 36, Point.CARTESIAN), new Point(17, 76, Point.CARTESIAN),new Point(scorePose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        grab3 = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose3), new Point(17, 74, Point.CARTESIAN),
                        new Point(27.5, 36, Point.CARTESIAN), new Point(grabPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        score4 = follower.pathBuilder().addPath(new BezierCurve(new Point(grabPose), new Point(27.5, 36, Point.CARTESIAN), new Point(17, 74, Point.CARTESIAN),new Point(scorePose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        grab4 = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose4), new Point(17, 72, Point.CARTESIAN),
                        new Point(27.5, 36, Point.CARTESIAN), new Point(grabPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        score5 = follower.pathBuilder().addPath(new BezierCurve(new Point(grabPose), new Point(27.5, 36, Point.CARTESIAN), new Point(17, 72, Point.CARTESIAN),new Point(scorePose5)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
        park = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose5), new Point(13, 58.5, Point.CARTESIAN), new Point(parkPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
    }

    public Command specAuto(){
        return new SequentialGroup(
            new SequentialGroup(
                    new ParallelGroup(
                            Wrist.INSTANCE.setPosition(.2),
                            Elbow.INSTANCE.setPosition(.39),
                            Rotate.INSTANCE.setPosition(.23),
                            Linkage.INSTANCE.setTarget(-735)
                    ),
                    new FollowPath(score1,true,1.0)

            ),
            Claw.INSTANCE.open(),
            new ParallelGroup(
                    EndEffectorPositions.grabFromWall(),
                    new FollowPath(push1,true,1.0),
                    new SequentialGroup(
                            new Delay(0.5),
                            Linkage.INSTANCE.setTarget(50)
                    )
            ),
            new FollowPath(human1,true,1.0),
            new FollowPath(push2,true,1.0),
            new FollowPath(human2,true,1.0),
            new FollowPath(human3,true,1.0),

            new Delay(0.25),
            Claw.INSTANCE.close(),
            new Delay(0.3),
            new ParallelGroup(
                    new ParallelGroup(
                            Wrist.INSTANCE.setPosition(.2),
                            Elbow.INSTANCE.setPosition(.39),
                            Rotate.INSTANCE.setPosition(.23),
                            Linkage.INSTANCE.setTarget(-735)
                    ),
                    new SequentialGroup(
                            new Delay(0.25),
                            new FollowPath(score2,true,1.0)
                    )
            ),
            Claw.INSTANCE.open(),
            new ParallelGroup(
                    new FollowPath(grab2,true,.8),
                    new SequentialGroup(
                            new Delay(0.4),
                            new ParallelGroup(
                                    EndEffectorPositions.grabFromWall(),
                                    Linkage.INSTANCE.setTarget(50)
                            )
                    )
            ),
            new Delay(0.25),
            Claw.INSTANCE.close(),
            new Delay(0.25),

            new ParallelGroup(
                    new ParallelGroup(
                            Wrist.INSTANCE.setPosition(.2),
                            Elbow.INSTANCE.setPosition(.39),
                            Rotate.INSTANCE.setPosition(.23),
                            Linkage.INSTANCE.setTarget(-735)
                    ),
                   new SequentialGroup(
                           new Delay(0.25),
                           new FollowPath(score3,true,1.0)
                   )
            ),
            Claw.INSTANCE.open(),
            new ParallelGroup(
                    new FollowPath(grab3,true,.8),
                    new SequentialGroup(
                            new Delay(0.4),
                            new ParallelGroup(
                                    EndEffectorPositions.grabFromWall(),
                                    Linkage.INSTANCE.setTarget(50)
                            )
                    )
            ),
            new Delay(0.25),
            Claw.INSTANCE.close(),
            new Delay(0.25),
            new ParallelGroup(
                    new ParallelGroup(
                            Wrist.INSTANCE.setPosition(.2),
                            Elbow.INSTANCE.setPosition(.39),
                            Rotate.INSTANCE.setPosition(.23),
                            Linkage.INSTANCE.setTarget(-735)
                    ),
                    new SequentialGroup(
                            new Delay(0.25),
                            new FollowPath(score4,true,1.0)
                    )
            ),
            Claw.INSTANCE.open(),
            new ParallelGroup(
                    new FollowPath(grab4,true,.8),
                    new SequentialGroup(
                            new Delay(0.4),
                            new ParallelGroup(
                                    EndEffectorPositions.grabFromWall(),
                                    Linkage.INSTANCE.setTarget(50)
                            )
                    )
            ),
            new Delay(0.25),
            Claw.INSTANCE.close(),
            new Delay(0.25),
            new ParallelGroup(
                    new ParallelGroup(
                            Wrist.INSTANCE.setPosition(.2),
                            Elbow.INSTANCE.setPosition(.39),
                            Rotate.INSTANCE.setPosition(.23),
                            Linkage.INSTANCE.setTarget(-735)
                    ),
                    new SequentialGroup(
                            new Delay(0.25),
                            new FollowPath(score5,true,1.0)
                    )
            ),
            Claw.INSTANCE.open(),
            new ParallelGroup(
                    new FollowPath(park,true,1.0),
                    new SequentialGroup(
                            new Delay(0.4),
                            new ParallelGroup(
                                    Elbow.INSTANCE.setPosition(.99),
                                    Linkage.INSTANCE.setTarget(50)
                            )

                    )
            )


        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OpModeData.telemetry = telemetry;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        Elbow.INSTANCE.setPosition(0).invoke();
        Claw.INSTANCE.close().invoke();
        Wrist.INSTANCE.setPosition(0.1).invoke();
        Rotate.INSTANCE.setPosition(0.23).invoke();

        Lift.INSTANCE.leftMotor.resetEncoder();
        Lift.INSTANCE.rightMotor.resetEncoder();
        Linkage.INSTANCE.linkageMotor.resetEncoder();

        OpModeData.telemetry.addData("Status", "Initialized");
        OpModeData.telemetry.update();
    }

    @Override
    public void onStartButtonPressed(){
        specAuto().invoke();

    }
    @Override
    public void onUpdate(){
        telemetry.update();
    }
}
