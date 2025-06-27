package org.firstinspires.ftc.teamcode.example.java.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.example.java.commands.EndEffectorPositions;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

@TeleOp(name = "MAIN-1Person")
@Config
public class Tele1Person extends NextFTCOpMode {

    public Tele1Person() {
        super(
                Elbow.INSTANCE,
                Wrist.INSTANCE,
                Rotate.INSTANCE,
                Claw.INSTANCE,
                Lift.INSTANCE,
                Linkage.INSTANCE,
                Limelight.INSTANCE
        );
    }
    public String frontLeftName = "leftFront";
    public String frontRightName = "rightFront";
    public String backLeftName = "leftRear";
    public String backRightName = "rightRear";
    public MotorEx frontLeftMotor;
    public MotorEx frontRightMotor;
    public MotorEx backLeftMotor;
    public MotorEx backRightMotor;
    public MotorEx[] motors;
    public Command driverControlled;

    private Limelight3A limelight;

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OpModeData.telemetry = telemetry;
        frontLeftMotor = new MotorEx(frontLeftName);
        backLeftMotor = new MotorEx(backLeftName);
        backRightMotor = new MotorEx(backRightName);
        frontRightMotor = new MotorEx(frontRightName);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = new MotorEx[] {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        gamepadManager.getGamepad2().getLeftStick().setProfileCurve(x -> (float) (0.5*x));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        limelight.pipelineSwitch(0);

        telemetry.addLine("Initialized");
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        // Start driver-controlled driving
        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad2().getRightStick(),
                gamepadManager.getGamepad2().getLeftStick());
        driverControlled.invoke();
        limelight.start();



        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(
                () -> new SequentialGroup(
                        new ParallelGroup(
                                Linkage.INSTANCE.linkageUp(),
                                EndEffectorPositions.basketScore()
                        ),
                        Lift.INSTANCE.toHighBasket()

                )
        );

        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(
                () -> new SequentialGroup(
                        Lift.INSTANCE.retract(),
                        new ParallelGroup(
                                Linkage.INSTANCE.linkageDown(),
                                EndEffectorPositions.hoverAboveFloor()
                        )

                )

        );

        gamepadManager.getGamepad2().getRightBumper().setPressedCommand(
                () -> new SequentialGroup(
                        Lift.INSTANCE.retract(),
                        new ParallelGroup(
                                Linkage.INSTANCE.linkageUp(),
                                EndEffectorPositions.grabFromWall()
                        )

                )

        );

        gamepadManager.getGamepad2().getDpadRight().setPressedCommand(
                () -> EndEffectorPositions.grabFromFloor()
        );

        // TRIGGERS: Chamber Operations
        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(
                () -> new SequentialGroup(
                        Lift.INSTANCE.retract(),
                        new ParallelGroup(
                                Linkage.INSTANCE.linkageUp(),
                                EndEffectorPositions.preChamberScore()
                        )

                )
        );

        gamepadManager.getGamepad2().getLeftBumper().setPressedCommand(
                () -> new SequentialGroup(
                        Lift.INSTANCE.retract(),
                        EndEffectorPositions.ramScore()
                )
        );


        gamepadManager.getGamepad2().getTriangle().setPressedCommand(
                () -> Claw.INSTANCE.open()
        );

        gamepadManager.getGamepad2().getCross().setPressedCommand(
                () -> Claw.INSTANCE.close()
        );
        gamepadManager.getGamepad2().getRightTrigger().setHeldCommand(
                value -> Lift.INSTANCE.manualControl(-value)
        );

        gamepadManager.getGamepad2().getLeftTrigger().setHeldCommand(
                value -> Lift.INSTANCE.manualControl(value)
        );




        // LEFT STICK: Rotation Control
        // Press left stick for center rotation

    }
    public void onUpdate(){
        LLResult result = limelight.getLatestResult();
        if (result != null && gamepad2.circle) {

            double[] outputs = result.getPythonOutput();

            Rotate.INSTANCE.setPosition((outputs[5]/255)+0.2).invoke();

        }
        telemetry.update();
    }
}