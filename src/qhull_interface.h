#include <kdl/frames.hpp>

typedef struct
{
	int i[20];
	int count;
} Face;

void initQhull();
void calculateQhull(const std::vector<KDL::Vector> &v, std::vector<KDL::Vector> &v_out, std::vector<Face> &f_out);


