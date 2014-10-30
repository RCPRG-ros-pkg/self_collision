#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "visualization_msgs/MarkerArray.h"
#include "self_collision_test/urdf_collision_parser.h"

#include "urdf/model.h"

//#include <iostream>
//#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <tinyxml.h>

namespace self_collision
{

void Capsule::clear()
{
	radius = 0.0;
	length = 0.0;
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
	// Inertial (optional)
/*	TiXmlElement *i = config->FirstChildElement("inertial");
	if (i)
	{
		link.inertial.reset(new Inertial());
		if (!parseInertial(*link.inertial, i))
		{
			logError("Could not parse inertial element for Link [%s]", link.name.c_str());
			return false;
		}
	}
	// Multiple Visuals (optional)
	for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
	{
		boost::shared_ptr<Visual> vis;
		vis.reset(new Visual());
		if (parseVisual(*vis, vis_xml))
		{
			link.visual_array.push_back(vis);
		}
		else
		{
			vis.reset();
			logError("Could not parse visual element for Link [%s]", link.name.c_str());
			return false;
		}
	}
	// Visual (optional)
	// Assign the first visual to the .visual ptr, if it exists
	if (!link.visual_array.empty())
		link.visual = link.visual_array[0];
*/
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
	// Collision (optional)
	// Assign the first collision to the .collision ptr, if it exists
//	if (!link.collision_array.empty())
//		link.collision = link.collision_array[0];
}

boost::shared_ptr<CollisionModel> CollisionModel::parseURDF(const std::string &xml_string)
{
	boost::shared_ptr<CollisionModel> model(new CollisionModel);
//	model->clear();
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
	// Get all Material elements... NOT!
/*	for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
	{
		boost::shared_ptr<Material> material;
		material.reset(new Material);
		try {
			parseMaterial(*material, material_xml, false); // material needs to be fully defined here
			if (model->getMaterial(material->name))
			{
				ROS_ERROR("material '%s' is not unique.", material->name.c_str());
				material.reset();
				model.reset();
				return model;
			}
			else
			{
				model->materials_.insert(make_pair(material->name,material));
				logDebug("urdfdom: successfully added a new material '%s'", material->name.c_str());
			}
		}
		catch (ParseError &e) {
			ROS_ERROR("material xml is not initialized correctly");
			material.reset();
			model.reset();
			return model;
		}
	}
*/
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
/*				ROS_INFO("urdfdom: setting link '%s' material", link->name.c_str());
				if (link->visual)
				{
					if (!link->visual->material_name.empty())
					{
						if (model->getMaterial(link->visual->material_name))
						{
							ROS_INFO("urdfdom: setting link '%s' material to '%s'", link->name.c_str(),link->visual->material_name.c_str());
							link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
						}
						else
						{
							if (link->visual->material)
							{
								ROS_INFO("urdfdom: link '%s' material '%s' defined in Visual.", link->name.c_str(),link->visual->material_name.c_str());
								model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
							}
							else
							{
								ROS_ERROR("link '%s' material '%s' undefined.", link->name.c_str(),link->visual->material_name.c_str());
								model.reset();
								return model;
							}
						}
					}
				}
*/				model->links_.insert(make_pair(link->name,link));
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
	// Get all Joint elements... NOT!
/*	for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
	{
		boost::shared_ptr<Joint> joint;
		joint.reset(new Joint);
		if (parseJoint(*joint, joint_xml))
		{
			if (model->getJoint(joint->name))
			{
				ROS_ERROR("joint '%s' is not unique.", joint->name.c_str());
				model.reset();
				return model;
			}
			else
			{
				model->joints_.insert(make_pair(joint->name,joint));
				logDebug("urdfdom: successfully added a new joint '%s'", joint->name.c_str());
			}
		}
		else
		{
			ROS_ERROR("joint xml is not initialized correctly");
			model.reset();
			return model;
		}
	}
*/
	// every link has children links and joints, but no parents, so we create a
	// local convenience data structure for keeping child->parent relations
/*	std::map<std::string, std::string> parent_link_tree;
	parent_link_tree.clear();
	// building tree: name mapping
	try
	{
		model->initTree(parent_link_tree);
	}
	catch(ParseError &e)
	{
		ROS_ERROR("Failed to build tree: %s", e.what());
		model.reset();
		return model;
	}
	// find the root link
	try
	{
		model->initRoot(parent_link_tree);
	}
	catch(ParseError &e)
	{
		ROS_ERROR("Failed to find root link: %s", e.what());
		model.reset();
		return model;
	}
*/
	return model;
}

}	// namespace self_collision

