package org.technodot.ftc.twentyfivebeta.roboctrl;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.technodot.ftc.twentyfivebeta.Configuration;

/*
TechnoDot — 7:18 PM
oh btw did the cable arrive?
also did you take the robot home like i told leo to tell you

Silent Cyber — 7:20 PM
The bot isn't done

TechnoDot — 7:20 PM
womp

Silent Cyber — 7:20 PM
And the cable is on the bot

TechnoDot — 7:20 PM
did you bring the robot home
ok

Silent Cyber — 7:20 PM
And we're gonna model a bracket for it

TechnoDot — 7:20 PM
did you bring the robot home

Silent Cyber  — 7:20 PM
No

TechnoDot — 7:20 PM
WNAT
QWHRUHWU9RHT3UH
WGAET
WHAT

Silent Cyber — 7:21 PM
I didn't have any time after school

TechnoDot — 7:21 PM
HAIYAA

Silent Cyber — 7:21 PM
And it's not ready for you to code really
I mean it kinda is

TechnoDot — 7:21 PM
WHAT
HOW IS IT READY WHEN IM NOT READY
AND HOW IS IT NOT READY WHEN I AM READY
WHAT THE FUCK IS THIS PARADOXICAL AHH BS

Silent Cyber — 7:21 PM
But not logistically possible for me to take home

TechnoDot — 7:21 PM
WHY CANT IT NOT BE READY WHEN IM PLAYING CLASH

Silent Cyber — 7:21 PM
You j just sick
Suck

TechnoDot — 7:22 PM
NUNUNO
THIS IS A YOU PROBLEM
I WANT TO WORK ON THE ROBOT NOW
DID LEO NOT COMMUNICATE TO YOU TO BRING THE ROBOT HOME
 */
public class SilentRunner101 extends InputController {
    public SilentRunner101(Gamepad gamepad1, Gamepad gamepad2) {
        super(gamepad1, gamepad2);
    }

    public float driveForward() {
        return ready() && Math.abs(gamepad1.left_stick_y) > Configuration.DRIVE_CONTROLLER_DEADZONE ? -gamepad1.left_stick_y : 0.0f;
    }

    public float driveStrafe() {
        return ready() && Math.abs(gamepad1.left_stick_x) > Configuration.DRIVE_CONTROLLER_DEADZONE ? gamepad1.left_stick_x : 0.0f;
    }

    public float driveRotate() {
        return ready() && Math.abs(gamepad1.right_stick_x) > Configuration.DRIVE_CONTROLLER_DEADZONE ? gamepad1.right_stick_x : 0.0f;
    }

    public boolean driveAim() {
        return ready() && (gamepad1.a || gamepad1.x || gamepad1.y || gamepad1.b);
    }

    public boolean extakeClose() {
        return ready() && gamepad1.a;
    }

    public boolean extakeFar() {
        return ready() && gamepad1.left_bumper;
    }

//    public boolean extakeDualClose() {
//        return ready() && gamepad1.y;
//    }

    public boolean extakeDualClose() {
        return ready() && gamepad1.x; // rebound!
    }

    public boolean extakeReverse() {
        return ready() && gamepad1.dpad_down;
    }

    public boolean intakeIn() {
        return ready() && gamepad1.right_trigger > 0.1;
    }

    public boolean intakeOut() {
        return ready() && gamepad1.left_trigger > 0.1;
    }

//    public boolean intakeNudge() {
//        return ready() && gamepad1.left_bumper;
//    }

    public boolean intakeNudge() {
        return false; // this ctrl is disabled
    }


    public boolean intakeServoLeft() {
        return ready() && gamepad1.dpad_left;
    }

    public boolean intakeServoRight() {
        return ready() && gamepad1.dpad_right;
    }

    public boolean sequenceShoot() {
        return ready() && gamepad1.right_bumper;
    }

    public boolean resetYaw() {
        return ready() && gamepad1.start;
    }

    public boolean queuePurple() {
        return ready() && (gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y);
    }

    public boolean queueGreen() {
        return ready() && (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right);
    }

    public void vibrateExtakeReady() {
        if (ready()) {
            gamepad1.rumble(0.0, 1.0, Configuration.GAMEPAD_RUMBLE_DURATION_MS);
        }
    }

    public void vibrateEndgame() {
        if (ready()) {
            gamepad1.rumble(1.0, 0.0, Configuration.GAMEPAD_RUMBLE_STRONG_MS);
        }
    }
    
    public void vibrateEndgameTick() {
        if (ready()) {
            gamepad1.rumble(0.3, 0.0, Configuration.GAMEPAD_RUMBLE_WEAK_MS);
        }
    }
    
    public void vibrateEndgameFinale() {
        if (ready()) {
            gamepad1.rumble(1.0, 1.0, Configuration.GAMEPAD_RUMBLE_FINALE_MS);
        }
    }
}
