package org.firstinspires.ftc.teamcode.example.java;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class AutonomousProgram extends NextFTCOpMode {
    public AutonomousProgram() {
        super(EndEffector.INSTANCE, Lift.INSTANCE);
    }

    public Command firstRoutine() {
        return new SequentialGroup(
                //.INSTANCE.toHigh(),
                new ParallelGroup(
                        //Lift.INSTANCE.toMiddle(),
                        EndEffector.INSTANCE.close()
                ),
                new Delay(0.5),
                new ParallelGroup(
                        EndEffector.INSTANCE.open(),
                        Lift.INSTANCE.toLowBasket()
                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }
}
