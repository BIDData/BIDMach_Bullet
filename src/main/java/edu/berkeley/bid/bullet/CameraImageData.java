package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class CameraImageData implements Serializable {
    private static final long serialVersionUID = -472381267164873L;


    public int m_pixelWidth;
    public int m_pixelHeight;
    public byte [] m_rgbColorData;
    public float [] m_depthValues;
    public int [] m_segmentationMaskValues;

    public CameraImageData (int width, int height) {
	m_pixelWidth = width;
	m_pixelHeight = height;
	int size = width * height;
	
	m_rgbColorData = new byte[size*3];
	m_depthValues = new float[size];
	m_segmentationMaskValues = new int[size];
    }

    public CameraImageData () {
    }

    @Override
    public String toString () {
	return "CameraImageData[width=" + m_pixelWidth + ",height=" + m_pixelHeight + "]";
    }

}
