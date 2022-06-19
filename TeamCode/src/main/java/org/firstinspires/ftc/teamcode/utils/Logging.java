package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

public class Logging {
    int delaySecs = 5;
    long lastTime = System.currentTimeMillis();

    public Logging(int delaySecs) {
        this.delaySecs = delaySecs;
    }
    public void log(String tag, String statement) {
        if(System.currentTimeMillis() - lastTime >= delaySecs/1000) {
            Log.e(tag,statement);
            lastTime = System.currentTimeMillis();
        }
    }
}
