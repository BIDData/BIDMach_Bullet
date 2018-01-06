package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class DebugVisualizerCameraInfo implements Serializable {
    private static final long serialVersionUID = 435278367839642345L;

    public int m_width;
    public int m_height;

    public float [] m_viewMatrix;
    public float [] m_projectionMatrix;
    
    public float [] m_camUp;
    public float [] m_camForward;
    
    public float [] m_horizontal;
    public float [] m_vertical;
	
    public float m_yaw;
    public float m_pitch;
    public float m_distance;
    public float [] m_target;

    public DebugVisualizerCameraInfo () {

	m_viewMatrix = new float[16];
	m_projectionMatrix = new float[16];
	
	m_camUp = new float[3];
	m_camForward = new float[3];
	
	m_horizontal = new float[3];
	m_vertical = new float[3];
	
	m_yaw = 0;
	m_pitch = 0;
	m_distance = 1;
	m_target = new float [3];
    }

    private String printVector(float [] vector) {
	return "[" + vector[0] + "," + vector[1] + "," + vector[2] + "]";
    }

    private String printVectorHead(float [] vector) {
	return "[" + vector[0] + "," + vector[1] + "," + vector[2] + "...]";
    }

    @Override
    public String toString () {
    	return "DebugVisualizerCameraInfo[" +
	    "width=" + m_width + ", " +
	    "height=" + m_height + ", " +
	    "viewMatrix=" + printVectorHead(m_viewMatrix) + ", " +
	    "projectionMatrix=" + printVectorHead(m_projectionMatrix) + ", " +
	    "camUp=" + printVector(m_camUp) + ", " +
	    "camForward=" + printVector(m_camForward) + ", " +
	    "horizontal=" + printVector(m_horizontal) + ", " +
	    "vertical=" + printVector(m_vertical) + ", " +
	    "yaw=" + m_yaw + ", " +
	    "pitch=" + m_pitch + ", " +
	    "distance=" + m_distance + ", " +
	    "target=" + printVector(m_target) + "]";
    }

}
