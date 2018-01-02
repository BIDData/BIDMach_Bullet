package BIDMach.rl.environments.bullet;
import BIDMat.{FMat,Quaternion};

@SerialVersionUID(32134214321342L)
class Transform3(jtf3:edu.berkeley.bid.bullet.Transform3) extends Serializable {
    val javaTransform3 = jtf3;

    val basis = FMat.zeros(3,3);
    val origin = FMat.zeros(1,3);

    syncFromJava();

    def syncFromJava() = {
	basis(0,0) = javaTransform3.m_basis.m_el(0).x;
	basis(0,1) = javaTransform3.m_basis.m_el(0).y;
	basis(0,2) = javaTransform3.m_basis.m_el(0).z;

	basis(1,0) = javaTransform3.m_basis.m_el(1).x;
	basis(1,1) = javaTransform3.m_basis.m_el(1).y;
	basis(1,2) = javaTransform3.m_basis.m_el(1).z;

	basis(2,0) = javaTransform3.m_basis.m_el(2).x;
	basis(2,1) = javaTransform3.m_basis.m_el(2).y;
	basis(2,2) = javaTransform3.m_basis.m_el(2).z;

	origin(0) = javaTransform3.m_origin.x;
	origin(1) = javaTransform3.m_origin.y;
	origin(2) = javaTransform3.m_origin.z;
    };

    def syncToJava() = {
	javaTransform3.m_basis.m_el(0).x = basis(0,0);
	javaTransform3.m_basis.m_el(0).y = basis(0,1);
	javaTransform3.m_basis.m_el(0).z = basis(0,2);

	javaTransform3.m_basis.m_el(1).x = basis(1,0);
	javaTransform3.m_basis.m_el(1).y = basis(1,1);
	javaTransform3.m_basis.m_el(1).z = basis(1,2);

	javaTransform3.m_basis.m_el(2).x = basis(2,0);
	javaTransform3.m_basis.m_el(2).y = basis(2,1);
	javaTransform3.m_basis.m_el(2).z = basis(2,2);

	javaTransform3.m_origin.x = origin(0);
	javaTransform3.m_origin.y = origin(1);
	javaTransform3.m_origin.z = origin(2);
    };

    def this() = this(new edu.berkeley.bid.bullet.Transform3);

    override def toString () = {
	 "Transform3[basis=" + basis.toString() + ", origin=" + origin.toString()+"]";
    }

}
