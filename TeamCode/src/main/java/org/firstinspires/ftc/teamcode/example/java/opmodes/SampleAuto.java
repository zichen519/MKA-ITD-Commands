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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
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
    private Limelight3A limelight;

    public static double pickupX;
    public static double pickupY;
    private final Pose startPose = new Pose(6.500, 107.5, Math.toRadians(0.0));
    private final Pose scorePose = new Pose(17.5,121, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(25.5,114.5,Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(25.5, 124.25, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(28, 119.5, Math.toRadians(45));
    private final Pose pickupSubPose = new Pose(66.5, 88, Math.toRadians(-90));

    private PathChain scorePreload, grab1, grab2, grab3, grabSub, score1, score2, score3, score4;

    public void buildPaths(){

        scorePreload = follower.pathBuilder().addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        grab1 = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose), new Point(19,109,Point.CARTESIAN), new Point(pickup1Pose)))
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
        grabSub = follower.pathBuilder().addPath(new BezierCurve(new Point(scorePose), new Point(64.5,120,Point.CARTESIAN), new Point(pickupSubPose))).setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading())
                .build();
    }
    public Command alignWithSample() {
        return new LambdaCommand()
                .setStart(() -> {
                    telemetry.addData("Alignment", "Starting Limelight alignment");
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        double[] outputs = result.getPythonOutput();
                        if (outputs != null && outputs.length > 8) {
                            double xOffsetInches = outputs[7];  // Forward/backward
                            double yOffsetInches = outputs[8];
                            double robotHeading = Math.toRadians(-90);

                            Pose currentPose = follower.getPose();
                            double fieldX = currentPose.getX() - yOffsetInches;  // Y offset affects X movement (left/right)
                            double fieldY = currentPose.getY() - xOffsetInches;  // X offset affects Y movement (forward/back)

                            telemetry.addData("Limelight X Offset", xOffsetInches);
                            telemetry.addData("Limelight Y Offset", yOffsetInches);
                            telemetry.addData("Target Field X", fieldX);
                            telemetry.addData("Target Field Y", fieldY);

                            Pose targetPose = new Pose(fieldX, fieldY, robotHeading);

                            // Create alignment path
                            PathChain alignmentPath = follower.pathBuilder()
                                    .addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                                    .build();

                            // Follow the alignment path
                            new FollowPath(alignmentPath, true).invoke();

                            // Set rotate position based on Limelight data
                            if (outputs.length > 5) {
                                double rotatePosition = (outputs[5] / 255) + 0.2;
                                Rotate.INSTANCE.setPosition(rotatePosition).invoke();
                            }
                        }
                    } else {
                        telemetry.addData("Alignment", "No Limelight result available");
                    }
                })
                .setUpdate(() -> {
                    // Update telemetry during alignment
                    telemetry.addData("Alignment Status", "In progress");
                    telemetry.addData("Robot Pose", follower.getPose().toString());
                })
                .setIsDone(() -> {
                    // Command finishes when the path following is complete
                    return !follower.isBusy();
                })
                .setStop(interrupted -> {
                    if (interrupted) {
                        telemetry.addData("Alignment", "Interrupted");
                    } else {
                        telemetry.addData("Alignment", "Completed");
                    }
                });
    }
    public Command firstRoutine(){

        return new SequentialGroup(

                //score preload
                new ParallelGroup(
                        new FollowPath(scorePreload, true, 0.6),
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
                        new FollowPath(score3, true, 0.8),
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
                //pickup sub
                new FollowPath(grabSub, true, 1.0)
                /*
                new ParallelGroup(
                        EndEffectorPositions.hoverAboveFloor(),
                        Linkage.INSTANCE.linkageDown(),
                        Rotate.INSTANCE.setPosition(0.39)

                ),
                new FollowPath(grabSub, true, 1.0),

                new Delay(0.5), // Brief pause to let the robot settle
                alignWithSample(), // Perform Limelight-based alignment
                new Delay(0.2), // Brief pause after alignment
                EndEffectorPositions.grabFromFloor()

                 */
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
        limelight = OpModeData.INSTANCE.getHardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        limelight.pipelineSwitch(3);

        Lift.INSTANCE.leftMotor.resetEncoder();
        Lift.INSTANCE.rightMotor.resetEncoder();
        Linkage.INSTANCE.linkageMotor.resetEncoder();
    }

    @Override
    public void onStartButtonPressed(){
        firstRoutine().invoke();
        limelight.start();
    }
}
