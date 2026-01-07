package com.olsc.t6dof;

import android.app.Activity;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.widget.Toast;
import com.olsc.t6dof.render.CubeRenderer;
import com.olsc.t6dof.sensors.OrientationSensor;

/**
 * 主Activity
 */
public class MainActivity extends Activity {
    private GLSurfaceView glSurfaceView;
    private OrientationSensor orientationSensor;
    private CubeRenderer renderer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        
        // 创建传感器
        orientationSensor = new OrientationSensor();
        
        // 检查设备是否有必要的传感器
        if (!orientationSensor.hasRequiredSensors(this)) {
            Toast.makeText(this, "此设备缺少必要的传感器（陀螺仪或加速度计+磁力计）", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        
        // 创建OpenGL Surface View
        glSurfaceView = new GLSurfaceView(this);
        glSurfaceView.setEGLContextClientVersion(2);
        
        // 创建渲染器
        renderer = new CubeRenderer(orientationSensor);
        glSurfaceView.setRenderer(renderer);
        
        // 设置为连续渲染模式
        glSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        
        setContentView(glSurfaceView);
    }

    @Override
    protected void onResume() {
        super.onResume();
        glSurfaceView.onResume();
        orientationSensor.start(this);
    }

    @Override
    protected void onPause() {
        orientationSensor.stop();
        glSurfaceView.onPause();
        super.onPause();
    }
}

