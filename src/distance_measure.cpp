#include "distance_measure.h"

DistanceMeasure::DistanceMeasure()
{
}

void DistanceMeasure::addCapsule(const std::string &name, double radius, double length)
{
	map_[name] = std::make_pair<ObjectType, fcl_2::ShapeBase*>(CAPSULE, static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(radius, length)));
}

void DistanceMeasure::addConvex(std::string name)
{
/*
 * The first two arguments (plane_normals_ and plane_dis_) are not used and can be set to NULL.
 * The polygons_ vector has the following structure:
 * polygons_[0]: n = number of points in polygon 0
 * polygons_[1]: point 0 in polygon 0
 * polygons_[n]: point n-1 in polygon 0
 * polygons_[n+1]: n = number of points in polygon 1

Convex(Vec3f* plane_normals_,
         FCL_REAL* plane_dis_,
         int num_planes_,
         Vec3f* points_,
         int num_points_,
         int* polygons_)
*/
	// allocate a lot of memory and initialize very simple convex hull mesh (4 points and 4 polygons)
	int num_points = 4;
	fcl_2::Vec3f* points = new fcl_2::Vec3f[5000];
	int num_planes = 4;
	int *polygons = new int[20000];
	points[0] = fcl_2::Vec3f(0.0, 0.0, 0.0);
	points[1] = fcl_2::Vec3f(0.1, 0.0, 0.0);
	points[2] = fcl_2::Vec3f(0.0, 0.1, 0.0);
	points[3] = fcl_2::Vec3f(0.0, 0.0, 0.1);
	polygons[0] = 3;	polygons[1] = 2;	polygons[2] = 1;	polygons[3] = 0;
	polygons[4] = 3;	polygons[5] = 3;	polygons[6] = 1;	polygons[7] = 0;
	polygons[8] = 3;	polygons[9] = 3;	polygons[10] = 2;	polygons[11] = 0;
	polygons[12] = 3;	polygons[13] = 3;	polygons[14] = 2;	polygons[15] = 1;
	map_[name] = std::make_pair<ObjectType, fcl_2::ShapeBase*>(CONVEX, static_cast<fcl_2::ShapeBase*>(new fcl_2::Convex(NULL, NULL, num_planes, points, num_points, polygons)));
}

void DistanceMeasure::updateConvex(std::string name, const std::vector<KDL::Vector> &v, const std::vector<Face> &f)
{
	ObjectsMap::iterator it = map_.find(name);
	if (it != map_.end() && it->second.first == CONVEX)
	{
		fcl_2::Convex* ob = static_cast<fcl_2::Convex*>(it->second.second);
		int poly_counter = 0;
		for (std::vector<Face>::const_iterator it = f.begin(); it != f.end(); it++)
		{
			ob->polygons[poly_counter++] = it->count;
			for (int i=0; i<it->count; i++)
			{
				ob->polygons[poly_counter++] = it->i[i];
			}
		}

		ob->num_planes = f.size();

		int points_counter = 0;
		for (std::vector<KDL::Vector>::const_iterator it = v.begin(); it != v.end(); it++)
		{
			ob->points[points_counter++] = fcl_2::Vec3f(it->x(), it->y(), it->z());
		}

		ob->num_points = v.size();

//		ROS_INFO("DistanceMeasure::updateConvex: %d  %d", ob->num_planes, ob->num_points);
		ob->fillEdges();
	}
	else
	{
		ROS_ERROR("DistanceMeasure::updateConvex: failure");
	}

}

double DistanceMeasure::getDistance(const std::string &name1, const KDL::Frame &tf1, const std::string &name2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out)
{
	ObjectsMap::iterator it1 = map_.find(name1);
	ObjectsMap::iterator it2 = map_.find(name2);
	if (it1 != map_.end() && it2 != map_.end() && it1 != it2)
	{
		if (it1->second.first == CAPSULE && it2->second.first == CAPSULE)
		{
			ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CAPSULE");
			const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(it1->second.second);
			const fcl_2::Capsule* ob2 = static_cast<const fcl_2::Capsule*>(it2->second.second);

			// capsules are shifted by length/2
			double x1,y1,z1,w1, x2, y2, z2, w2;
			KDL::Frame tf1_corrected = tf1 * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
			tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
			KDL::Frame tf2_corrected = tf2 * KDL::Frame(KDL::Vector(0,0,-ob2->lz/2.0));
			tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

			// output variables
			fcl_2::FCL_REAL distance;
			fcl_2::Vec3f p1;
			fcl_2::Vec3f p2;
			gjk_solver.shapeDistance(
				*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
				*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
				 &distance, &p1, &p2);
			// the output for two capsules is in global coordinates
			d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
			d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
			return distance;
		}
		else if ((it1->second.first == CAPSULE && it2->second.first == CONVEX) || (it1->second.first == CONVEX && it2->second.first == CAPSULE))
		{
			if (it1->second.first == CONVEX && it2->second.first == CAPSULE)
			{
				// swap iterators
				ObjectsMap::iterator tmp = it1;
				it1 = it2;
				it2 = tmp;
			}
			// now it1 points to CAPSULE and it2 points to CONVEX
			ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CONVEX");
			const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(it1->second.second);
			const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(it2->second.second);

			// capsules are shifted by length/2
			double x1,y1,z1,w1, x2, y2, z2, w2;
			KDL::Frame tf1_corrected = tf1 * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
			tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
			tf2.M.GetQuaternion(x2,y2,z2,w2);

			// output variables
			fcl_2::FCL_REAL distance;
			fcl_2::Vec3f p1;
			fcl_2::Vec3f p2;
			gjk_solver.shapeDistance(
				*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
				*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
				 &distance, &p1, &p2);
			// the output for two capsules is in wtf coordinates
			d1_out = tf1_corrected*KDL::Vector(p1[0], p1[1], p1[2]);
			d2_out = tf1_corrected*((tf1_corrected.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
			return distance;
		}
		else if (it1->second.first == CONVEX && it2->second.first == CONVEX)
		{
			ROS_INFO("DistanceMeasure::getDistance: CONVEX,CONVEX");
			const fcl_2::Convex* ob1 = static_cast<const fcl_2::Convex*>(it1->second.second);
			const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(it2->second.second);

			double x1,y1,z1,w1, x2, y2, z2, w2;
			tf1.M.GetQuaternion(x1,y1,z1,w1);
			tf2.M.GetQuaternion(x2,y2,z2,w2);

			// output variables
			fcl_2::FCL_REAL distance;
			fcl_2::Vec3f p1;
			fcl_2::Vec3f p2;
			gjk_solver.shapeDistance(
				*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1.p.x(),tf1.p.y(),tf1.p.z())),
				*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
				 &distance, &p1, &p2);
			// the output for two capsules is in wtf coordinates
			d1_out = tf1*KDL::Vector(p1[0], p1[1], p1[2]);
			d2_out = tf1*((tf1.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
			return distance;
		}
		else
		{
			ROS_ERROR("not supported distance measure");
		}
	}
}

int DistanceMeasure::publishMarker(ros::Publisher &pub, int m_id, const std::string &name, const KDL::Frame tf)
{
	ObjectsMap::iterator it = map_.find(name);
	if (it != map_.end())
	{
		if (it->second.first == CAPSULE)
		{
			const fcl_2::Capsule* ob = static_cast<const fcl_2::Capsule*>(it->second.second);
			m_id = publishCapsule(pub, m_id, tf, ob->lz, ob->radius);
		}
		else if (it->second.first == CONVEX)
		{
			const fcl_2::Convex* ob = static_cast<const fcl_2::Convex*>(it->second.second);
			m_id = publishMeshMarker(pub, m_id, tf, ob->points, ob->num_planes, ob->polygons, 0, 0, 1);
		}
		else
		{
			m_id = publishSinglePointMarker(pub, m_id, tf * KDL::Vector(), 1, 0, 0, 0.02);
		}
	}
	return m_id;
}

