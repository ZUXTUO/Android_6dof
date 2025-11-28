package com.olsc.t6dof.render;

import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import com.olsc.t6dof.sensors.SensorFusionManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;

public class PoseRenderer implements GLSurfaceView.Renderer {
    private final SensorFusionManager fusionManager;
    private int program;
    private final float[] proj = new float[16];
    private final float[] view = new float[16];
    private final float[] mvp = new float[16];
    private final float[] model = new float[16];
    private FloatBuffer axesBuf;
    private FloatBuffer gridBuf;
    private FloatBuffer trailBuf;
    private FloatBuffer groundAxesBuf;
    private int axesCount;
    private int gridCount;
    private int trailVertices;
    private final ArrayList<Float> trail = new ArrayList<>();
    private float lastTX, lastTY, lastTZ;
    private float orbitYaw;
    private float orbitPitch;
    private float zoom = 1f;

    public PoseRenderer(SensorFusionManager manager) {
        this.fusionManager = manager;
    }

    @Override
    public void onSurfaceCreated(javax.microedition.khronos.opengles.GL10 gl, javax.microedition.khronos.egl.EGLConfig config) {
        GLES20.glClearColor(0.08f, 0.08f, 0.1f, 1f);
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);
        GLES20.glEnable(GLES20.GL_BLEND);
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);
        program = buildProgram(
                "attribute vec3 aPos; uniform mat4 uMVP; void main(){ gl_Position = uMVP * vec4(aPos,1.0); }",
                "precision mediump float; uniform vec4 uColor; void main(){ gl_FragColor = uColor; }"
        );
        axesBuf = createAxes();
        int n = 10;
        gridBuf = createGrid(n, 1f);
        groundAxesBuf = createGroundAxes(n, 1f);
        axesCount = 6;
        gridCount = (2 * n + 1) * 4 * 2;
        trailVertices = 0;
        Matrix.setIdentityM(view, 0);
    }

    @Override
    public void onSurfaceChanged(javax.microedition.khronos.opengles.GL10 gl, int width, int height) {
        GLES20.glViewport(0, 0, width, height);
        float aspect = (float) width / Math.max(height, 1);
        Matrix.perspectiveM(proj, 0, 60f, aspect, 0.1f, 200f);
    }

    @Override
    public void onDrawFrame(javax.microedition.khronos.opengles.GL10 gl) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
        float[] pose = fusionManager.getPose();
        float x = pose[0];
        float z = pose[1];
        float y = pose[2];
        float qw = pose[3];
        float qx = pose[4];
        float qy = pose[5];
        float qz = pose[6];

        float r = 12f / Math.max(0.3f, Math.min(zoom, 3f));
        float yaw = (float) Math.toRadians(orbitYaw);
        float pitch = (float) Math.toRadians(Math.max(-80f, Math.min(80f, orbitPitch)));
        float cx = (float) (Math.cos(pitch) * Math.cos(yaw));
        float cy = (float) Math.sin(pitch);
        float cz = (float) (Math.cos(pitch) * Math.sin(yaw));
        float eyeX = x + r * cx;
        float eyeY = y + r * cy;
        float eyeZ = z + r * cz;
        Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, x, y, z, 0f, 1f, 0f);

        if (trailVertices == 0 || dist3(lastTX, lastTY, lastTZ, x, y, z) > 0.02f) {
            addTrailPoint(x, y, z);
            lastTX = x; lastTY = y; lastTZ = z;
        }

        GLES20.glUseProgram(program);
        int aPos = GLES20.glGetAttribLocation(program, "aPos");
        int uMVP = GLES20.glGetUniformLocation(program, "uMVP");
        int uColor = GLES20.glGetUniformLocation(program, "uColor");

        Matrix.multiplyMM(mvp, 0, proj, 0, view, 0);
        GLES20.glUniformMatrix4fv(uMVP, 1, false, mvp, 0);
        GLES20.glEnableVertexAttribArray(aPos);

        GLES20.glUniform4f(uColor, 0.25f, 0.25f, 0.28f, 1f);
        gridBuf.position(0);
        drawLines(aPos, gridBuf, gridCount, GLES20.GL_LINES);

        groundAxesBuf.position(0);
        GLES20.glUniform4f(uColor, 1f, 0f, 0f, 1f);
        drawLines(aPos, groundAxesBuf, 2, GLES20.GL_LINES);
        groundAxesBuf.position(6);
        GLES20.glUniform4f(uColor, 0.6f, 0f, 0f, 1f);
        drawLines(aPos, groundAxesBuf, 2, GLES20.GL_LINES);
        groundAxesBuf.position(12);
        GLES20.glUniform4f(uColor, 0f, 0f, 1f, 1f);
        drawLines(aPos, groundAxesBuf, 2, GLES20.GL_LINES);
        groundAxesBuf.position(18);
        GLES20.glUniform4f(uColor, 0f, 0f, 0.6f, 1f);
        drawLines(aPos, groundAxesBuf, 2, GLES20.GL_LINES);
        groundAxesBuf.position(24);
        GLES20.glUniform4f(uColor, 0f, 1f, 0f, 1f);
        drawLines(aPos, groundAxesBuf, 2, GLES20.GL_LINES);

        

        if (trailBuf != null && trailVertices > 1) {
            GLES20.glUniform4f(uColor, 1f, 1f, 0f, 1f);
            trailBuf.position(0);
            drawLines(aPos, trailBuf, trailVertices, GLES20.GL_LINE_STRIP);
        }

        buildModel(model, qw, qx, qy, qz, x, y, z);
        float[] mvp2 = new float[16];
        Matrix.multiplyMM(mvp2, 0, mvp, 0, model, 0);
        GLES20.glUniformMatrix4fv(uMVP, 1, false, mvp2, 0);

        axesBuf.position(0);
        GLES20.glUniform4f(uColor, 1f, 0f, 0f, 1f);
        drawLines(aPos, axesBuf, 2, GLES20.GL_LINES);


        GLES20.glDisableVertexAttribArray(aPos);
    }

    private void drawLines(int aPos, FloatBuffer buf, int vertCount, int mode) {
        GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, buf);
        GLES20.glDrawArrays(mode, 0, vertCount);
    }
    

    

    private static int buildProgram(String vs, String fs) {
        int v = compile(GLES20.GL_VERTEX_SHADER, vs);
        int f = compile(GLES20.GL_FRAGMENT_SHADER, fs);
        int p = GLES20.glCreateProgram();
        GLES20.glAttachShader(p, v);
        GLES20.glAttachShader(p, f);
        GLES20.glLinkProgram(p);
        return p;
    }

    private static int compile(int type, String src) {
        int s = GLES20.glCreateShader(type);
        GLES20.glShaderSource(s, src);
        GLES20.glCompileShader(s);
        return s;
    }

    private static FloatBuffer createAxes() {
        float[] axes = new float[]{
                0,0,0, 5,0,0,
                0,0,0, 0,5,0,
                0,0,0, 0,0,5
        };
        FloatBuffer b = ByteBuffer.allocateDirect(axes.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        b.put(axes).position(0);
        return b;
    }

    private static FloatBuffer createGrid(int n, float step) {
        int lines = (2 * n + 1) * 4;
        float[] pts = new float[lines * 2 * 3];
        int idx = 0;
        for (int i = -n; i <= n; i++) {
            pts[idx++] = -n * step; pts[idx++] = 0; pts[idx++] = i * step;
            pts[idx++] = n * step;  pts[idx++] = 0; pts[idx++] = i * step;
            pts[idx++] = i * step;  pts[idx++] = 0; pts[idx++] = -n * step;
            pts[idx++] = i * step;  pts[idx++] = 0; pts[idx++] = n * step;
        }
        FloatBuffer b = ByteBuffer.allocateDirect(pts.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        b.put(pts).position(0);
        return b;
    }

    private static FloatBuffer createGroundAxes(int n, float step) {
        float eps = 0.02f;
        float[] v = new float[]{
                0, eps, 0,  n * step, eps, 0,
                0, eps, 0, -n * step, eps, 0,
                0, eps, 0,  0, eps,  n * step,
                0, eps, 0,  0, eps, -n * step,
                0, 0, 0,  0, 5f, 0
        };
        FloatBuffer b = ByteBuffer.allocateDirect(v.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        b.put(v).position(0);
        return b;
    }

    private static void buildModel(float[] out, float w, float x, float y, float z, float tx, float ty, float tz) {
        Matrix.setIdentityM(out, 0);
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        out[0] = 1 - 2 * (yy + zz);
        out[1] = 2 * (xy + wz);
        out[2] = 2 * (xz - wy);
        out[4] = 2 * (xy - wz);
        out[5] = 1 - 2 * (xx + zz);
        out[6] = 2 * (yz + wx);
        out[8] = 2 * (xz + wy);
        out[9] = 2 * (yz - wx);
        out[10] = 1 - 2 * (xx + yy);
        out[12] = tx; out[13] = ty; out[14] = tz;
    }

    private void addTrailPoint(float x, float y, float z) {
        trail.add(x); trail.add(y); trail.add(z);
        if (trail.size() > 3000) {
            trail.remove(0); trail.remove(0); trail.remove(0);
        }
        float[] arr = new float[trail.size()];
        for (int i = 0; i < trail.size(); i++) arr[i] = trail.get(i);
        trailBuf = ByteBuffer.allocateDirect(arr.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        trailBuf.put(arr).position(0);
        trailVertices = arr.length / 3;
    }

    private static float dist3(float ax, float ay, float az, float bx, float by, float bz) {
        float dx = ax - bx, dy = ay - by, dz = az - bz;
        return (float) Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    

    

    public void setZoom(float z) { this.zoom = z; }
    public void addOrbit(float dx, float dy) {
        float s = 0.06f;
        orbitYaw += dx * s;
        orbitPitch += dy * s;
    }
}
