package com.olsc.t6dof.render;

import android.hardware.SensorManager;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import com.olsc.t6dof.sensors.OrientationSensor;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Cude渲染器
 */
public class CubeRenderer implements GLSurfaceView.Renderer {
    private final OrientationSensor orientationSensor;
    
    // Shader程序
    private int program;
    
    // 矩阵
    private final float[] projectionMatrix = new float[16];
    private final float[] viewMatrix = new float[16];
    private final float[] modelMatrix = new float[16];
    private final float[] mvpMatrix = new float[16];
    private final float[] tempMatrix = new float[16];
    
    // 立方体数据
    private FloatBuffer vertexBuffer;
    private FloatBuffer colorBuffer;
    private ShortBuffer indexBuffer;
    
    // 立方体的顶点数 (6面 x 2三角形 x 3顶点 = 36)
    private static final int CUBE_INDEX_COUNT = 36;
    
    public CubeRenderer(OrientationSensor sensor) {
        this.orientationSensor = sensor;
    }
    
    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        // 设置清屏颜色为深蓝灰色
        GLES20.glClearColor(0.1f, 0.15f, 0.2f, 1.0f);
        
        // 启用深度测试
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);
        GLES20.glDepthFunc(GLES20.GL_LEQUAL);
        
        // 创建shader程序
        String vertexShaderCode =
            "uniform mat4 uMVPMatrix;" +
            "attribute vec4 vPosition;" +
            "attribute vec4 vColor;" +
            "varying vec4 fColor;" +
            "void main() {" +
            "  gl_Position = uMVPMatrix * vPosition;" +
            "  fColor = vColor;" +
            "}";
        
        String fragmentShaderCode =
            "precision mediump float;" +
            "varying vec4 fColor;" +
            "void main() {" +
            "  gl_FragColor = fColor;" +
            "}";
        
        int vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, vertexShaderCode);
        int fragmentShader = loadShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderCode);
        
        program = GLES20.glCreateProgram();
        GLES20.glAttachShader(program, vertexShader);
        GLES20.glAttachShader(program, fragmentShader);
        GLES20.glLinkProgram(program);
        
        // 创建立方体几何数据
        createCube();
    }
    
    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        GLES20.glViewport(0, 0, width, height);
        
        // 设置投影矩阵
        float ratio = (float) width / height;
        Matrix.frustumM(projectionMatrix, 0, -ratio, ratio, -1, 1, 1.0f, 100.0f);
    }
    
    @Override
    public void onDrawFrame(GL10 gl) {
        // 清除颜色和深度缓冲
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
        
        // 获取设备的旋转矩阵 (Device -> World)
        // Android标准: X=East, Y=North, Z=Sky
        float[] deviceRotation = orientationSensor.getRotationMatrix();
        
        // 设置视图矩阵
        // View Matrix = World -> Device (Camera)
        // 由于是旋转矩阵，逆矩阵等于转置矩阵
        // 这将世界坐标系的点转换到设备坐标系中
        Matrix.transposeM(viewMatrix, 0, deviceRotation, 0);
        
        // 设置立方体的模型矩阵
        // 固定在世界坐标系的正上方（天空方向，Z轴）5米处
        Matrix.setIdentityM(modelMatrix, 0);
        Matrix.translateM(modelMatrix, 0, 0.0f, 0.0f, 5.0f);
        
        // 让立方体自转一下，方便观察3D结构
        Matrix.rotateM(modelMatrix, 0, 45.0f, 0.0f, 1.0f, 0.0f);
        Matrix.rotateM(modelMatrix, 0, 30.0f, 1.0f, 0.0f, 0.0f);
        
        // 计算MVP矩阵
        Matrix.multiplyMM(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0);
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0);
        
        // 绘制立方体
        drawCube();
    }
    
    /**
     * 创建立方体的几何数据
     */
    private void createCube() {
        // 立方体边长
        float size = 0.5f;
        
        // 立方体的8个顶点坐标
        float vertices[] = {
            // 前面
            -size, -size,  size,  // 0: 左下前
             size, -size,  size,  // 1: 右下前
             size,  size,  size,  // 2: 右上前
            -size,  size,  size,  // 3: 左上前
            
            // 后面
            -size, -size, -size,  // 4: 左下后
             size, -size, -size,  // 5: 右下后
             size,  size, -size,  // 6: 右上后
            -size,  size, -size   // 7: 左上后
        };
        
        // 每个面使用不同的颜色
        float colors[] = {
            // 前面 - 红色
            1.0f, 0.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 0.0f, 1.0f,
            
            // 后面 - 绿色
            0.0f, 1.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f, 1.0f
        };
        
        // 立方体的索引（6个面，每个面2个三角形）
        short indices[] = {
            // 前面
            0, 1, 2,  0, 2, 3,
            // 右面
            1, 5, 6,  1, 6, 2,
            // 后面
            5, 4, 7,  5, 7, 6,
            // 左面
            4, 0, 3,  4, 3, 7,
            // 顶面
            3, 2, 6,  3, 6, 7,
            // 底面
            4, 5, 1,  4, 1, 0
        };
        
        // 为每个索引顶点分配颜色
        float expandedColors[] = new float[indices.length * 4];
        int colorIdx = 0;
        
        // 前面 - 红色
        for (int i = 0; i < 6; i++) {
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 1.0f;
        }
        // 右面 - 绿色
        for (int i = 0; i < 6; i++) {
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 1.0f;
        }
        // 后面 - 蓝色
        for (int i = 0; i < 6; i++) {
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 1.0f;
        }
        // 左面 - 黄色
        for (int i = 0; i < 6; i++) {
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 1.0f;
        }
        // 顶面 - 青色
        for (int i = 0; i < 6; i++) {
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 1.0f;
        }
        // 底面 - 品红色
        for (int i = 0; i < 6; i++) {
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 0.0f;
            expandedColors[colorIdx++] = 1.0f;
            expandedColors[colorIdx++] = 1.0f;
        }
        
        // 创建顶点缓冲
        ByteBuffer vbb = ByteBuffer.allocateDirect(vertices.length * 4);
        vbb.order(ByteOrder.nativeOrder());
        vertexBuffer = vbb.asFloatBuffer();
        vertexBuffer.put(vertices);
        vertexBuffer.position(0);
        
        // 创建颜色缓冲
        ByteBuffer cbb = ByteBuffer.allocateDirect(expandedColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        colorBuffer = cbb.asFloatBuffer();
        colorBuffer.put(expandedColors);
        colorBuffer.position(0);
        
        // 创建索引缓冲
        ByteBuffer ibb = ByteBuffer.allocateDirect(indices.length * 2);
        ibb.order(ByteOrder.nativeOrder());
        indexBuffer = ibb.asShortBuffer();
        indexBuffer.put(indices);
        indexBuffer.position(0);
    }
    
    /**
     * 绘制立方体
     */
    private void drawCube() {
        GLES20.glUseProgram(program);
        
        // 获取shader中的变量句柄
        int positionHandle = GLES20.glGetAttribLocation(program, "vPosition");
        int colorHandle = GLES20.glGetAttribLocation(program, "vColor");
        int mvpMatrixHandle = GLES20.glGetUniformLocation(program, "uMVPMatrix");
        
        // 启用顶点数组
        GLES20.glEnableVertexAttribArray(positionHandle);
        GLES20.glEnableVertexAttribArray(colorHandle);
        
        // 准备顶点坐标数据
        GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false, 12, vertexBuffer);
        
        // 准备颜色数据 (使用索引展开的颜色)
        GLES20.glVertexAttribPointer(colorHandle, 4, GLES20.GL_FLOAT, false, 16, colorBuffer);
        
        // 应用MVP矩阵
        GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0);
        
        // 绘制立方体
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, CUBE_INDEX_COUNT, GLES20.GL_UNSIGNED_SHORT, indexBuffer);
        
        // 禁用顶点数组
        GLES20.glDisableVertexAttribArray(positionHandle);
        GLES20.glDisableVertexAttribArray(colorHandle);
    }
    
    /**
     * 加载shader
     */
    private int loadShader(int type, String shaderCode) {
        int shader = GLES20.glCreateShader(type);
        GLES20.glShaderSource(shader, shaderCode);
        GLES20.glCompileShader(shader);
        return shader;
    }
}
