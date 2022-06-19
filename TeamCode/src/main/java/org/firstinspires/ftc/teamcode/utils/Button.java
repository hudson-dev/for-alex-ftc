package org.firstinspires.ftc.teamcode.utils;

public class Button {
    boolean lastButton = false;

    public Button () {}

    public boolean isClicked(boolean button) {
        boolean a = button && !lastButton;
        lastButton = button;
        return a;
    }
}
