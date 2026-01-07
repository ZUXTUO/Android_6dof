package com.olsc.t6dof.sensors;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;

/**
 * 方向传感器管理器
 */
public class OrientationSensor implements SensorEventListener {
    private SensorManager sensorManager;
    private Sensor rotationSensor;
    private Sensor accelerometer;
    private Sensor magnetometer;
    
    private boolean useRotationVector = false;
    private final float[] rotationMatrix = new float[16];
    private final float[] orientation = new float[3]; // 用于调试
    
    // 用于加速度计和磁力计的备用方案
    private final float[] gravity = new float[3];
    private final float[] geomagnetic = new float[3];
    private boolean hasGravity = false;
    private boolean hasGeomagnetic = false;
    
    public OrientationSensor() {
        Matrix.setIdentityM(rotationMatrix, 0);
    }
    
    /**
     * 启动传感器
     * 优先使用旋转向量传感器(更精确)，如果不可用则使用加速度计+磁力计
     */
    public void start(Context context) {
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        
        // 尝试使用旋转向量传感器（融合传感器，更精确）
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        
        if (rotationSensor != null) {
            useRotationVector = true;
            sensorManager.registerListener(this, rotationSensor, SensorManager.SENSOR_DELAY_GAME);
        } else {
            // 备用方案：使用加速度计和磁力计
            useRotationVector = false;
            accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            
            if (accelerometer != null) {
                sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
            }
            if (magnetometer != null) {
                sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
            }
        }
    }
    
    /**
     * 停止传感器
     */
    public void stop() {
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
    }
    
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            // 使用旋转向量传感器
            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values);
            
        } else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, gravity, 0, 3);
            hasGravity = true;
            updateRotationMatrix();
            
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, geomagnetic, 0, 3);
            hasGeomagnetic = true;
            updateRotationMatrix();
        }
    }
    
    /**
     * 使用加速度计和磁力计更新旋转矩阵
     */
    private void updateRotationMatrix() {
        if (hasGravity && hasGeomagnetic) {
            float[] R = new float[16];
            float[] I = new float[16];
            
            if (SensorManager.getRotationMatrix(R, I, gravity, geomagnetic)) {
                System.arraycopy(R, 0, rotationMatrix, 0, 16);
            }
        }
    }
    
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // 不需要处理
    }
    
    /**
     * 获取当前的旋转矩阵
     * @return 4x4旋转矩阵
     */
    public float[] getRotationMatrix() {
        return rotationMatrix;
    }
    
    /**
     * 检查是否有必要的传感器
     */
    public boolean hasRequiredSensors(Context context) {
        SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        Sensor rotationSensor = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        Sensor accelerometer = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor magnetometer = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        
        // 至少需要旋转向量传感器，或者加速度计+磁力计的组合
        return rotationSensor != null || (accelerometer != null && magnetometer != null);
    }
}
