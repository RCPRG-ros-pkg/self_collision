#include "ros/ros.h"
#include "self_collision_test/urdf_collision_parser.h"
#include "urdf/model.h"
#include <kdl/frames.hpp>
#include <tinyxml.h>

namespace self_collision
{

void Capsule::clear()
{
	radius = 0.0;
	length = 0.0;
}

void Convex::clear()
{
	points.clear();
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
	TiXmlElement *shape = g->FirstChildElement();
	if (!shape)
	{
		ROS_ERROR("Geometry tag contains no child element.");
		return geom;
	}
	std::string type_name = shape->ValueStr();
	if (type_name == "capsule")
	{
		Capsule *s = new Capsule();
		geom.reset(s);
		if (parseCapsule(*s, shape))
			return geom;
	}
	else if (type_name == "convex")
	{
		Convex *s = new Convex();
		geom.reset(s);
		if (parseConvex(*s, shape))
			return geom;
	}
	// TODO: add convex_hull type
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

}	// namespace self_collision


