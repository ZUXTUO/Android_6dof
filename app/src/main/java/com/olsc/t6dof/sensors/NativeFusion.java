package com.olsc.t6dof.sensors;

public class NativeFusion {
    private static boolean loaded;
    static {
        try {
            System.loadLibrary("fusion");
            loaded = true;
        } catch (Throwable t) {
            loaded = false;
        }
    }

    public native void init(float beta);
    public native void addAccelerometer(long ts, float ax, float ay, float az);
    public native void addGyroscope(long ts, float gx, float gy, float gz);
    public native void addMagnetometer(long ts, float mx, float my, float mz);
    public native void addLinearAcceleration(long ts, float ax, float ay, float az);
    public native void addPressure(long ts, float p);
    public native float[] getPose();

    public boolean isLoaded() { return loaded; }
    public float[] safePose() {
        if (!loaded) return new float[]{0,0,0,1,0,0,0};
        float[] p = getPose();
        return (p == null || p.length < 7) ? new float[]{0,0,0,1,0,0,0} : p;
    }
}
