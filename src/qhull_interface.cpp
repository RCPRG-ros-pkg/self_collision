#include "ros/ros.h"

#include <iostream>

#ifdef __cplusplus
extern "C" {
#include <libqhull/qset.h>
#include <libqhull/libqhull.h>
}
#else
#include <libqhull/qset.h>
#include <libqhull/libqhull.h>
#endif

#include "qhull_interface.h"

void initQhull()
{
	static bool initialized = false;

	if (initialized)
	{
//		ROS_ERROR("initQhull: already initialized");
//		return;
	}

	initialized = true;

	static FILE *null_sink = fopen("/dev/null", "w");

	static char qhull_command_const[] = "Qt i";
	static char qhull_command[256];
	memcpy(qhull_command, qhull_command_const, strlen(qhull_command_const) + 1);

//	qh_init_A(NULL, null_sink, null_sink, 0, NULL);
	qh_init_A(NULL, stdout, stdout, 0, NULL);
	qh_initflags( qhull_command );
}

void calculateQhull(const std::vector<KDL::Vector> &v, std::vector<KDL::Vector> &v_out, std::vector<Face> &f_out)
{
	// initialize points
	static coordT points[100 * 3];
	int counter = 0;
	for (std::vector<KDL::Vector>::const_iterator it = v.begin(); it != v.end(); it++)
	{
		points[counter++] = it->x();
		points[counter++] = it->y();
		points[counter++] = it->z();
	}

	qh_init_B(points, v.size(), 3, false);
	qh_qhull();

	int faces_count = qh num_facets;
	int vertices_count = qh num_vertices;
//	ROS_INFO("qhull test: vertices: %d  faces: %d", vertices_count, faces_count);

	std::map<int, int> v_id_map;
	int v_id = 0;
	v_out.clear();
	v_out.resize(vertices_count);
	for (vertexT *v = qh vertex_list; v != qh vertex_tail; v = v->next)
	{
//		ROS_INFO("%d", v->id);
		v_id_map[v->id] = v_id;
		v_out[v_id] = KDL::Vector(v->point[0], v->point[1], v->point[2]);
		v_id++;
	}

	f_out.clear();
	int f_id = 0;
	for (facetT *f = qh facet_list; f != qh facet_tail; f = f->next)
	{
		Face face;

//		std::cout << "FACE" << std::endl;
//		std::cout << "vertices2: ";
		for (int v_idx=0; v_idx < f->vertices->maxsize; v_idx++)
		{
			if (f->vertices->e[v_idx].p == NULL)
				break;
			vertexT *v = static_cast<vertexT *>(f->vertices->e[v_idx].p);

//			std::cout << v->id << " ";
			if (v_idx >= 20)
			{
				ROS_ERROR("v_idx >= 20");
				break;
			}
			if (v_id_map.find(v->id) == v_id_map.end())
			{
				ROS_ERROR("v_id in polygon is not in vertex list");
			}
			face.i[v_idx] = v_id_map[v->id];
			face.count = v_idx+1;
		}
//		std::cout << std::endl;

		if (face.count < 3)
		{
			ROS_ERROR("degenerated facet: %d", face.count);
			continue;
		}

		// triangle facet is reliable
		if (face.count == 3)
		{
			f_out.push_back(face);
			f_id++;
			continue;
		}

		// the facet has more than 3 vertices - we must get edges to construct proper polygon
		face.count = 0;

		std::multimap<int, int> ridges_map;
		// get the ridges
//		std::cout << "ridges: ";
		if (f->ridges == NULL)
		{
//			std::cout << "NULL" << std::endl;
		}
		else
		{
			for (int r_idx=0; r_idx < f->ridges->maxsize; r_idx++)
			{
				if (f->ridges->e[r_idx].p == NULL)
					break;
				ridgeT *r = static_cast<ridgeT *>(f->ridges->e[r_idx].p);
//				std::cout << "vertices: ";
				if (r->vertices == NULL)
				{
//					std::cout << "NULL" << std::endl;
				}
				else
				{
					int ridge[2] = {-1, -1};
					for (int v_idx=0; v_idx < r->vertices->maxsize; v_idx++)
					{
						if (r->vertices->e[v_idx].p == NULL)
							break;
						vertexT *v = static_cast<vertexT *>(r->vertices->e[v_idx].p);
						if (v_idx > 1)
						{
							ROS_ERROR("ridge has over 2 vertices");
							break;
						}
						ridge[v_idx] = v->id;
//						std::cout << v->id << " ";
					}
					if (ridge[0] == -1 || ridge[1] == -1)
					{
						ROS_ERROR("wrong ridge: %d %d", ridge[0], ridge[1]);
					}
					else
					{
						ridges_map.insert(std::make_pair<int,int>(ridge[0], ridge[1]));
						ridges_map.insert(std::make_pair<int,int>(ridge[1], ridge[0]));
					}
				}
			}
//			std::cout << std::endl;
		}

		int init_v_id = ridges_map.begin()->first;
		int v_id_prev = -1;
		int v_id = init_v_id;
		int v_idx = 0;

//		std::cout << "vertices: ";
		while (true)
		{
//			std::cout << "v_id:" << v_id << " prev_id:" << v_id_prev << " ";
			face.i[v_idx] = v_id_map[v_id];
			face.count = v_idx+1;
			v_idx++;
			std::pair<std::multimap<int, int>::iterator, std::multimap<int, int>::iterator> range = ridges_map.equal_range(v_id);
			for (std::multimap<int, int>::iterator it = range.first; it != range.second; it++)
			{
				if (it->second != v_id_prev)
				{
					v_id_prev = v_id;
					v_id = it->second;
					break;
				}
			}
			if (v_id == init_v_id)
				break;
			if (v_idx >= 20)
			{
				ROS_ERROR("too many vertices in one polygon");
				break;
			}
		}
//		std::cout << std::endl;
		
		f_out.push_back(face);
		f_id++;
	}
//	ROS_INFO("f_out: %ld", f_out.size());
//	ROS_INFO("v_out: %ld", v_out.size());
	qh_freebuffers();
	qh_freeqhull( False);

	int curlong, totlong; /* used !qh_NOmem */
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong)
		ROS_ERROR("qhull internal warning (main): did not free %d bytes of long memory(%d pieces)\n", totlong, curlong);
}


