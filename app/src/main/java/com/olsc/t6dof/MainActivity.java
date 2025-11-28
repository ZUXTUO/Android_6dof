package com.olsc.t6dof;

import android.app.Activity;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.widget.Toast;
import com.olsc.t6dof.render.PoseRenderer;
import com.olsc.t6dof.sensors.SensorFusionManager;

public class MainActivity extends Activity {
    private GLSurfaceView glSurfaceView;
    private SensorFusionManager fusionManager;
    private PoseRenderer renderer;
    private ScaleGestureDetector scaleDetector;
    private float lastX;
    private float lastY;
    private boolean dragging;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        glSurfaceView = new GLSurfaceView(this);
        glSurfaceView.setEGLContextClientVersion(2);
        fusionManager = new SensorFusionManager();
        renderer = new PoseRenderer(fusionManager);
        glSurfaceView.setRenderer(renderer);
        scaleDetector = new ScaleGestureDetector(this, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            private float accum = 1f;
            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                accum *= detector.getScaleFactor();
                accum = Math.max(0.3f, Math.min(accum, 3.0f));
                renderer.setZoom(accum);
                return true;
            }
        });
        glSurfaceView.setOnTouchListener((v, event) -> {
            scaleDetector.onTouchEvent(event);
            int p = event.getPointerCount();
            int action = event.getActionMasked();
            if (p == 1) {
                if (action == MotionEvent.ACTION_DOWN) {
                    lastX = event.getX();
                    lastY = event.getY();
                    dragging = true;
                } else if (action == MotionEvent.ACTION_MOVE && dragging) {
                    float dx = event.getX() - lastX;
                    float dy = event.getY() - lastY;
                    lastX = event.getX();
                    lastY = event.getY();
                    renderer.addOrbit(dx, dy);
                } else if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL) {
                    dragging = false;
                }
            } else if (action == MotionEvent.ACTION_POINTER_DOWN || action == MotionEvent.ACTION_POINTER_UP) {
                dragging = false;
            }
            return true;
        });
        setContentView(glSurfaceView);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!fusionManager.requireGyro(this)) {
            Toast.makeText(this, "此设备无陀螺仪，无法显示位姿", Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        glSurfaceView.onResume();
        try {
            fusionManager.start(this);
        } catch (SecurityException se) {
            Toast.makeText(this, "传感器权限不足：已降级采样率", Toast.LENGTH_LONG).show();
            try {
                fusionManager.stop(this);
                fusionManager.start(this);
            } catch (Exception e) {
                Toast.makeText(this, "启动失败：" + e.getMessage(), Toast.LENGTH_LONG).show();
                finish();
            }
        } catch (Exception e) {
            Toast.makeText(this, "启动失败：" + e.getMessage(), Toast.LENGTH_LONG).show();
            finish();
        }
    }

    @Override
    protected void onPause() {
        fusionManager.stop(this);
        glSurfaceView.onPause();
        super.onPause();
    }
}
