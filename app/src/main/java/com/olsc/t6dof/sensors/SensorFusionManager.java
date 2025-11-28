package com.olsc.t6dof.sensors;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class SensorFusionManager implements SensorEventListener {
    private final NativeFusion nativeFusion = new NativeFusion();
    private boolean started;

    public void start(Context context) {
        if (started) return;
        if (nativeFusion.isLoaded()) {
            // User requested high sensitivity (~2.0).
            // beta controls mag correction speed (Higher beta = Faster correction).
            nativeFusion.init(2.0f);
        }
        SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        started = true;
        Sensor acc = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyr = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor mag = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor lin = sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        Sensor prs = sm.getDefaultSensor(Sensor.TYPE_PRESSURE);
        registerWithFallback(sm, acc, SensorManager.SENSOR_DELAY_FASTEST);
        registerWithFallback(sm, gyr, SensorManager.SENSOR_DELAY_FASTEST);
        registerWithFallback(sm, mag, SensorManager.SENSOR_DELAY_FASTEST);
        registerWithFallback(sm, lin, SensorManager.SENSOR_DELAY_FASTEST);
        if (prs != null) sm.registerListener(this, prs, SensorManager.SENSOR_DELAY_NORMAL);
    }

    private void registerWithFallback(SensorManager sm, Sensor sensor, int rate) {
        if (sensor == null) return;
        try {
            sm.registerListener(this, sensor, rate);
        } catch (SecurityException e) {
            sm.registerListener(this, sensor, SensorManager.SENSOR_DELAY_GAME);
        }
    }

    public void stop(Context context) {
        if (!started) return;
        SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        sm.unregisterListener(this);
        started = false;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        long ts = event.timestamp;
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                nativeFusion.addAccelerometer(ts, event.values[0], event.values[1], event.values[2]);
                break;
            case Sensor.TYPE_GYROSCOPE:
                nativeFusion.addGyroscope(ts, event.values[0], event.values[1], event.values[2]);
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                nativeFusion.addMagnetometer(ts, event.values[0], event.values[1], event.values[2]);
                break;
            case Sensor.TYPE_LINEAR_ACCELERATION:
                nativeFusion.addLinearAcceleration(ts, event.values[0], event.values[1], event.values[2]);
                break;
            case Sensor.TYPE_PRESSURE:
                nativeFusion.addPressure(ts, event.values[0]);
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    public float[] getPose() {
        return nativeFusion.safePose();
    }

    public boolean requireGyro(Context context) {
        SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        Sensor gyr = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        return gyr != null;
    }
}
