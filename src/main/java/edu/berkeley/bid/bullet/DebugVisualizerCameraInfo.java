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

    @Override
    public String toString () {
    	return "DebugVisualizerCameraInfo[" +
	    "width=" + m_width + ", " +
	    "height=" + m_height + ", " +
	    "yaw=" + m_yaw + ", " +
	    "pitch=" + m_pitch + 
	    "distance=" + m_distance + "]";
    }

}
