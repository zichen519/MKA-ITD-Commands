package org.firstinspires.ftc.teamcode.example.java.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@TeleOp(name = "MAIN")
@Config
public class MainTele extends NextFTCOpMode {

    public MainTele() {
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

        telemetry.addLine("Initialized");
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        // Start driver-controlled driving
        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1().getLeftStick(),
                gamepadManager.getGamepad1().getRightStick());
        driverControlled.invoke();




        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(
                () -> new SequentialGroup(
                        Linkage.INSTANCE.linkageUp(),
                        Lift.INSTANCE.toHighBasket(),
                        EndEffectorPositions.basketScore()
                )
        );

        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(
                () -> new SequentialGroup(
                        Lift.INSTANCE.retract(),
                        Linkage.INSTANCE.linkageDown(),
                        EndEffectorPositions.hoverAboveFloor()
                )

        );

        gamepadManager.getGamepad2().getRightBumper().setPressedCommand(
                () -> new SequentialGroup(
                        Lift.INSTANCE.retract(),
                        Linkage.INSTANCE.linkageUp(),
                        EndEffectorPositions.grabFromWall()
                )

        );

        gamepadManager.getGamepad2().getDpadRight().setPressedCommand(
                () -> EndEffectorPositions.grabFromFloor()
        );

        // TRIGGERS: Chamber Operations
        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(
                () -> new SequentialGroup(
                        Lift.INSTANCE.retract(),
                        Linkage.INSTANCE.linkageUp(),
                        EndEffectorPositions.preChamberScore()
                )
        );

        gamepadManager.getGamepad2().getLeftBumper().setPressedCommand(
                () -> EndEffectorPositions.scoreOnChamber()
        );


        gamepadManager.getGamepad2().getLeftStick().getButton().setPressedCommand(
                () -> Claw.INSTANCE.open()
        );

        gamepadManager.getGamepad2().getRightStick().getButton().setPressedCommand(
                () -> Claw.INSTANCE.close()
        );
        gamepadManager.getGamepad2().getRightTrigger().setHeldCommand(
                value -> Lift.INSTANCE.manualControl(value)
        );

        gamepadManager.getGamepad2().getLeftTrigger().setHeldCommand(
                value -> Lift.INSTANCE.manualControl(-value)
        );



        // LEFT STICK: Rotation Control
        // Press left stick for center rotation

    }
    public void onUpdate(){

    }
}