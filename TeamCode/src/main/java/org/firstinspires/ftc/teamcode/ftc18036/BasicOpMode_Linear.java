/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ftc18036;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.botcore.framework.BotManager;
import org.firstinspires.ftc.teamcode.botcore.framework.Subsystem;
import org.firstinspires.ftc.teamcode.botcore.subsystems.ControllerSubsystem;
import org.firstinspires.ftc.teamcode.botcore.subsystems.DriveEncoderSensingSubsystem;
import org.firstinspires.ftc.teamcode.botcore.subsystems.ImuSensingSubsystem;
import org.firstinspires.ftc.teamcode.ftc18036.subsystem.MecanumChassisSubsystem;
import org.firstinspires.ftc.teamcode.ftc18036.subsystem.DebuggingSubsystem;

import java.util.HashSet;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    HashSet<Subsystem> queuedSystems = new HashSet<>();
    BotManager bot;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = new BotManager(this);

        createControllerSubsystem();
        createEncoderSubsystem();
        createChassisSubsystem();
        createImuSubsystem();
        createDebuggingSubsystem();
        registerSubsystems();


        waitForStart();

        bot.run();
    }


    private void registerSubsystems() {
        int counter = 1;

        for (Subsystem s : queuedSystems) {
            telemetry.addData("Registering Subsystem - " + counter + "/" + queuedSystems.size(), s);
            telemetry.update();
            counter++;

            bot.addSubsystem(s);

        }
    }
    private void createDebuggingSubsystem()
    {
        DebuggingSubsystem debuggingSubsystem = new DebuggingSubsystem("Debugging", bot, 30);
        queuedSystems.add(debuggingSubsystem);
    }
    private void createChassisSubsystem()
    {
        MecanumChassisSubsystem mecanumChassisSubsystem = new MecanumChassisSubsystem("Mecanum chassis base", bot, 30);
        queuedSystems.add(mecanumChassisSubsystem);
    }

    private void createEncoderSubsystem()
    {
        DriveEncoderSensingSubsystem driveEncoderSensingSubsystem = new DriveEncoderSensingSubsystem("Drive encoder sensing", bot, 1);
        queuedSystems.add(driveEncoderSensingSubsystem);
    }


    private void createImuSubsystem()
    {
        ImuSensingSubsystem imuSensingSubsystem = new ImuSensingSubsystem("IMU sensing", bot, 1);
        queuedSystems.add(imuSensingSubsystem);
    }

    private void createControllerSubsystem()
    {
        ControllerSubsystem controller;

        controller = new ControllerSubsystem("Controller Sensing", bot, 22);
        controller.setInvertYAxis(true);

        controller.setLeftCurveX(ControllerSubsystem.JoystickScale.exponentialModerate);
        controller.setLeftCurveY(ControllerSubsystem.JoystickScale.exponentialStrong);
        controller.setRightCurveX(ControllerSubsystem.JoystickScale.linearHalf);
        controller.setRightCurveY(ControllerSubsystem.JoystickScale.sigmoid);

        queuedSystems.add(controller);
    }

}
