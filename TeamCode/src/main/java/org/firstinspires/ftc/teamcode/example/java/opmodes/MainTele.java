package org.firstinspires.ftc.teamcode.example.java.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.example.java.commands.EndEffectorPositions;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Lift;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

@TeleOp(name = "MAIN")
@Config
public class MainTele extends NextFTCOpMode {

    public MainTele() {
        super(Elbow.INSTANCE, Wrist.INSTANCE, Rotate.INSTANCE, Claw.INSTANCE, Lift.INSTANCE);
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
        frontLeftMotor = new MotorEx(frontLeftName);
        backLeftMotor = new MotorEx(backLeftName);
        backRightMotor = new MotorEx(backRightName);
        frontRightMotor = new MotorEx(frontRightName);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // GAMEPAD 2 BINDINGS FOR END EFFECTOR TESTING

        // D-PAD: Preset Positions
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(
                () -> EndEffectorPositions.basketScore()
        );

        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(
                () -> EndEffectorPositions.hoverAboveFloor()
        );

        gamepadManager.getGamepad2().getRightBumper().setPressedCommand(
                () -> EndEffectorPositions.grabFromWall()
        );

        gamepadManager.getGamepad2().getDpadRight().setPressedCommand(
                () -> EndEffectorPositions.grabFromFloor()
        );

        // TRIGGERS: Chamber Operations
        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(
                () -> EndEffectorPositions.preChamberScore()
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



        // LEFT STICK: Rotation Control
        // Press left stick for center rotation
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("D-Pad: Preset positions");
        telemetry.addLine("Bumpers: Wall/Chamber");
        telemetry.addLine("Stick Btns: Claw open/close");
        telemetry.addLine("Face Btns: Individual testing");

        telemetry.update();
    }
}