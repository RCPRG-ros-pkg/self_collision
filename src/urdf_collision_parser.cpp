#include "ros/ros.h"
#include "urdf_collision_parser.h"
#include "urdf/model.h"
#include <kdl/frames.hpp>
#include <tinyxml.h>

namespace self_collision
{

fcl_2::GJKSolver_indep CollisionModel::gjk_solver;

Geometry::Geometry(int type) :
	type(type)
{
}

Capsule::Capsule() :
	Geometry(CAPSULE)
{
}

void Capsule::clear()
{
	radius = 0.0;
	length = 0.0;
}

int Capsule::publishMarker(ros::Publisher &pub, int m_id, const KDL::Frame tf)
{
	const fcl_2::Capsule* ob = static_cast<const fcl_2::Capsule*>(shape.get());
	m_id = publishCapsule(pub, m_id, tf, ob->lz, ob->radius);
}

Convex::Convex() :
	Geometry(CONVEX)
{
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
	shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Convex(NULL, NULL, num_planes, points, num_points, polygons)) );
}

Convex::~Convex()
{
	fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
	delete[] conv->polygons;
	conv->polygons = NULL;
	delete[] conv->points;
	conv->points = NULL;
}

void Convex::updateConvex(const std::vector<KDL::Vector> &v, const std::vector<Face> &f)
{
	fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
//	fcl_2::Convex* ob = static_cast<fcl_2::Convex*>(it->second.second);
	int poly_counter = 0;
	for (std::vector<Face>::const_iterator it = f.begin(); it != f.end(); it++)
	{
		conv->polygons[poly_counter++] = it->count;
		for (int i=0; i<it->count; i++)
		{
			conv->polygons[poly_counter++] = it->i[i];
		}
	}

	conv->num_planes = f.size();

	int points_counter = 0;
	for (std::vector<KDL::Vector>::const_iterator it = v.begin(); it != v.end(); it++)
	{
		conv->points[points_counter++] = fcl_2::Vec3f(it->x(), it->y(), it->z());
	}

	conv->num_points = v.size();
	conv->fillEdges();
}

void Convex::clear()
{
	points.clear();
}

int Convex::publishMarker(ros::Publisher &pub, int m_id, const KDL::Frame tf)
{
	const fcl_2::Convex* ob = static_cast<const fcl_2::Convex*>(shape.get());
	m_id = publishMeshMarker(pub, m_id, tf, ob->points, ob->num_planes, ob->polygons, 0, 0, 1);
	return m_id;
}

void Collision::clear()
{
	// TODO
}

void Link::clear()
{
	// TODO
}

boost::shared_ptr< const Link > CollisionModel::getLink(const std::string &name)
{
	LinkMap::const_iterator l_it = links_.find(name);
	if (l_it != links_.end())
	{
		return l_it->second;
	}
	return boost::shared_ptr< const Link >();
}


KDL::Vector initVectorFromString(const std::string &vector_str)
{
	std::vector<std::string> pieces;
	std::vector<double> xyz;
	boost::split( pieces, vector_str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); ++i){
		if (pieces[i] != "")
		{
			try
			{
				xyz.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			}
			catch (boost::bad_lexical_cast &e)
			{
				ROS_ERROR("Vector3 xyz element (%s) is not a valid float",pieces[i].c_str());
				return KDL::Vector();
			}
		}
	}

	if (xyz.size() != 3)
	{
		ROS_ERROR("Vector contains %i elements instead of 3 elements", (int)xyz.size()); 
		return KDL::Vector();
	}
	return KDL::Vector(xyz[0], xyz[1], xyz[2]);
};

bool CollisionModel::parsePose(KDL::Frame &pose, TiXmlElement* xml)
{
	KDL::Vector pos;
	KDL::Vector rpy;
	if (xml)
	{
		const char* xyz_str = xml->Attribute("xyz");
		if (xyz_str != NULL)
		{
			try
			{
				pos = initVectorFromString(xyz_str);
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
		const char* rpy_str = xml->Attribute("rpy");
		if (rpy_str != NULL)
		{
			try
			{
				rpy = initVectorFromString(rpy_str);
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
	}
	KDL::Frame pose_ret( KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()), pos );
	pose = pose_ret;
	return true;
}

bool CollisionModel::parseCapsule(Capsule &s, TiXmlElement *c)
{
	s.clear();
	s.type = Geometry::CAPSULE;
	if (!c->Attribute("radius"))
	{
		ROS_ERROR("Capsule shape must have a radius attribute");
		return false;
	}
	try
	{
		s.radius = boost::lexical_cast<double>(c->Attribute("radius"));
	}
	catch (boost::bad_lexical_cast &e)
	{
		std::stringstream stm;
		stm << "radius [" << c->Attribute("radius") << "] is not a valid float: " << e.what();
		ROS_ERROR("%s", stm.str().c_str());
		return false;
	}

	if (!c->Attribute("length"))
	{
		ROS_ERROR("Capsule shape must have a length attribute");
		return false;
	}
	try
	{
		s.length = boost::lexical_cast<double>(c->Attribute("length"));
	}
	catch (boost::bad_lexical_cast &e)
	{
		std::stringstream stm;
		stm << "length [" << c->Attribute("length") << "] is not a valid float: " << e.what();
		ROS_ERROR("%s", stm.str().c_str());
		return false;
	}
	return true;
}

bool CollisionModel::parsePoint(std::string &frame, KDL::Vector &p, TiXmlElement *c)
{
	if (c)
	{
		const char* xyz_str = c->Attribute("xyz");
		if (xyz_str != NULL)
		{
			try
			{
				p = initVectorFromString(xyz_str);
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
		const char* frame_str = c->Attribute("frame");
		if (frame_str != NULL)
		{
			try
			{
				frame = frame_str;
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
	}
	return true;
}

bool CollisionModel::parseConvex(Convex &s, TiXmlElement *c)
{
	s.clear();
	s.type = Geometry::CONVEX;

	for (TiXmlElement* col_xml = c->FirstChildElement("point"); col_xml; col_xml = col_xml->NextSiblingElement("point"))
	{
		std::string frame;
		KDL::Vector point;
		if (parsePoint(frame, point, col_xml))
		{
			s.points.push_back(std::make_pair<std::string, KDL::Vector>(frame, point));
		}
		else
		{
			ROS_ERROR("Could not parse point element for Convex");
			return false;
		}
	}

	return true;
}

boost::shared_ptr<Geometry> CollisionModel::parseGeometry(TiXmlElement *g)
{
	boost::shared_ptr<Geometry> geom;
	if (!g) return geom;
	TiXmlElement *shape_xml = g->FirstChildElement();
	if (!shape_xml)
	{
		ROS_ERROR("Geometry tag contains no child element.");
		return geom;
	}
	std::string type_name = shape_xml->ValueStr();
	if (type_name == "capsule")
	{
		Capsule *s = new Capsule();
		geom.reset(s);
		if (parseCapsule(*s, shape_xml))
		{
			s->shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(s->radius, s->length)) );
			return geom;
		}
	}
	else if (type_name == "convex")
	{
		Convex *s = new Convex();
		geom.reset(s);
		if (parseConvex(*s, shape_xml))
		{
			return geom;
		}
	}
	else
	{
		ROS_ERROR("Unknown geometry type '%s'", type_name.c_str());
		return geom;
	}
	return boost::shared_ptr<Geometry>();
}

bool CollisionModel::parseCollision(Collision &col, TiXmlElement* config)
{
	col.clear();
	// Origin
	TiXmlElement *o = config->FirstChildElement("origin");
	if (o) {
		if (!parsePose(col.origin, o))
			return false;
	}
	// Geometry
	TiXmlElement *geom = config->FirstChildElement("geometry");
	col.geometry = parseGeometry(geom);
	if (!col.geometry)
		return false;
	const char *name_char = config->Attribute("name");
//	if (name_char)
//		col.name = name_char;
	return true;
}

bool CollisionModel::parseLink(Link &link, TiXmlElement* config)
{
	link.clear();
	const char *name_char = config->Attribute("name");
	if (!name_char)
	{
		ROS_ERROR("No name given for the link.");
		return false;
	}
	link.name = std::string(name_char);
	// Multiple Collisions (optional)
	for (TiXmlElement* col_xml = config->FirstChildElement("self_collision_checking"); col_xml; col_xml = col_xml->NextSiblingElement("self_collision_checking"))
	{
		boost::shared_ptr<Collision> col;
		col.reset(new Collision());
		if (parseCollision(*col, col_xml))
		{
			link.collision_array.push_back(col);
		}
		else
		{
			col.reset();
			ROS_ERROR("Could not parse collision element for Link [%s]", link.name.c_str());
			return false;
		}
	}
}

CollisionModel::CollisionModel()
{
}

bool CollisionModel::parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c)
{
	const char* link1_str = c->Attribute("link1");
	const char* link2_str = c->Attribute("link2");
	if (link1_str != NULL && link2_str != NULL)
	{
		link1 = link1_str;
		link2 = link2_str;
		return true;
	}
	ROS_ERROR("disable_collisions has wrong attributes");

	return false;
}

void CollisionModel::parseSRDF(const std::string &xml_string)
{
	disabled_collisions.clear();

	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
	{
		ROS_ERROR("%s", xml_doc.ErrorDesc());
		xml_doc.ClearError();
		return;
	}

	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		ROS_ERROR("Could not find the 'robot' element in the xml file");
		return;
	}
	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name)
	{
		ROS_ERROR("No name given for the robot.");
		return;
	}

	if (name_ != std::string(name))
	{
		ROS_ERROR("Name from SRDF: %s differ from %s", name, name_.c_str());
		return;
	}

	// Get all disable_collisions elements
	for (TiXmlElement* disable_collision_xml = robot_xml->FirstChildElement("disable_collisions"); disable_collision_xml; disable_collision_xml = disable_collision_xml->NextSiblingElement("disable_collisions"))
	{
		std::string link1, link2;
		try {
			parseDisableCollision(link1, link2, disable_collision_xml);

			if (links_.find(link1) == links_.end())
			{
				ROS_ERROR("link '%s' does not exist.", link1.c_str());
				return;
			}
			if (links_.find(link2) == links_.end())
			{
				ROS_ERROR("link '%s' does not exist.", link2.c_str());
				return;
			}
			disabled_collisions.push_back(std::make_pair<std::string, std::string>(link1, link2));
		}
		catch (urdf::ParseError &e) {
			ROS_ERROR("disable_collisions xml is not initialized correctly");
			return;
		}
	}


}

boost::shared_ptr<CollisionModel> CollisionModel::parseURDF(const std::string &xml_string)
{
	boost::shared_ptr<CollisionModel> model(new CollisionModel);
	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
	{
		ROS_ERROR("%s", xml_doc.ErrorDesc());
		xml_doc.ClearError();
		model.reset();
		return model;
	}
	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		ROS_ERROR("Could not find the 'robot' element in the xml file");
		model.reset();
		return model;
	}
	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name)
	{
		ROS_ERROR("No name given for the robot.");
		model.reset();
		return model;
	}
	model->name_ = std::string(name);
	// Get all Link elements
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
	{
		boost::shared_ptr<Link> link;
		link.reset(new Link);
		try {
			parseLink(*link, link_xml);
			if (model->getLink(link->name))
			{
				ROS_ERROR("link '%s' is not unique.", link->name.c_str());
				model.reset();
				return model;
			}
			else
			{
				// set link visual material
				model->links_.insert(make_pair(link->name,link));
				ROS_INFO("urdfdom: successfully added a new link '%s'", link->name.c_str());
			}
		}
		catch (urdf::ParseError &e) {
			ROS_ERROR("link xml is not initialized correctly");
			model.reset();
			return model;
		}
	}
	if (model->links_.empty()){
		ROS_ERROR("No link elements found in urdf file");
		model.reset();
		return model;
	}
	return model;
}

void CollisionModel::generateCollisionPairs()
{
	enabled_collisions.clear();
	ROS_INFO("%ld", links_.size());

	for (LinkMap::iterator it1 = links_.begin(); it1 != links_.end(); it1++)
	{
		for (LinkMap::iterator it2 = links_.begin(); it2 != links_.end(); it2++)
		{
			if (it1->first == it2->first)
			{
				continue;
			}
			bool add = true;
			for (CollisionPairs::iterator dc_it = disabled_collisions.begin(); dc_it != disabled_collisions.end(); dc_it++)
			{
				if (	(dc_it->first == it1->first && dc_it->second == it2->first) ||
					(dc_it->second == it1->first && dc_it->first == it2->first) )
				{
					add = false;
					break;
				}
			}
			if (add)
			{
				for (CollisionPairs::iterator ec_it = enabled_collisions.begin(); ec_it != enabled_collisions.end(); ec_it++)
				{
					if (	(ec_it->first == it1->first && ec_it->second == it2->first) ||
						(ec_it->second == it1->first && ec_it->first == it2->first) )
					{
						add = false;
						break;
					}
				}

				if (add)
				{
					enabled_collisions.push_back(std::make_pair<std::string, std::string>(it1->first, it2->first));
				}
			}
		}
	}
}

double CollisionModel::getDistance(const Geometry &geom1, const KDL::Frame &tf1, const Geometry &geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out)
{
	if (geom1.type == Geometry::CAPSULE && geom2.type == Geometry::CAPSULE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CAPSULE");
		const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(geom1.shape.get());
		const fcl_2::Capsule* ob2 = static_cast<const fcl_2::Capsule*>(geom2.shape.get());

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
	else if (geom1.type == Geometry::CAPSULE && geom2.type == Geometry::CONVEX)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CONVEX");
		const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(geom1.shape.get());
		const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(geom2.shape.get());

		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		KDL::Frame tf1_corrected = tf1;// * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
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
	else if (geom1.type == Geometry::CONVEX && geom2.type == Geometry::CAPSULE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CONVEX, CAPSULE");
		const fcl_2::Convex* ob1 = static_cast<const fcl_2::Convex*>(geom1.shape.get());
		const fcl_2::Capsule* ob2 = static_cast<const fcl_2::Capsule*>(geom2.shape.get());

		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		tf1.M.GetQuaternion(x1,y1,z1,w1);
		KDL::Frame tf2_corrected = tf2;// * KDL::Frame(KDL::Vector(0,0,-ob2->lz/2.0));
		tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::FCL_REAL distance;
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		gjk_solver.shapeDistance(
			*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1.p.x(),tf1.p.y(),tf1.p.z())),
			*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
			 &distance, &p1, &p2);
		// the output for two capsules is in wtf coordinates
		d1_out = tf1*KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = tf1*((tf1.Inverse()*tf2_corrected).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
		return distance;
	}
	else if (geom1.type == Geometry::CONVEX && geom2.type == Geometry::CONVEX)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CONVEX,CONVEX");
		const fcl_2::Convex* ob1 = static_cast<const fcl_2::Convex*>(geom1.shape.get());
		const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(geom2.shape.get());

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

}	// namespace self_collision


