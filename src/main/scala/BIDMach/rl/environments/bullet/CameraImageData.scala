package BIDMach.rl.environments.bullet;
import BIDMat.{BMat,FMat,IMat}

@SerialVersionUID(452462543275423L)
class CameraImageData(jcid:edu.berkeley.bid.bullet.CameraImageData) extends Serializable {
    val javaCameraImageData = jcid;

    def width = javaCameraImageData.m_pixelWidth;
    def height = javaCameraImageData.m_pixelHeight;

    val rgb = new BMat(width, height, javaCameraImageData.m_rgbColorData);
    val depth = new FMat(width, height, javaCameraImageData.m_depthValues);
    val segmentation = new IMat(width, height, javaCameraImageData.m_segmentationMaskValues);

    def unpack() = {
	(rgb, depth, segmentation)
    }

    override def toString () = {
	"CameraImageData[width=" + width + ",height=" + height + "]";
    }

}
