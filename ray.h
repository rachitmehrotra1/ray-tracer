#define HEADER_RAY
#define INTERSECT_EPSILON 0.0000000001
#define SCREEN_ALIGN_EPSILON 0.0000001
#define ANGLE_CULL_EPSILON 0.0000001 
#define SINGLE_COL_MAX 255.0
#include <Eigen/Core>	
#include <Eigen/Geometry>
#include<vector>
#include<cmath>//For sqrt
using namespace Eigen;

struct surface {
	double specular;
	double reflection;
	double diffusion;
};

class colour {
	public:
	double red;
	double green;
	double blue;
	double specular;
	colour(): red(0), green(0), blue(0) , specular(0.0) {}
	colour(double nr, double ng, double nb):
		red(nr), green(ng), blue(nb), specular(0.0) {}
	colour(double nr, double ng, double nb, double ns):
		red(nr), green(ng), blue(nb) {
		specular = ns > 1.0 ? 1.0 : ns;
	}
	
	// Need operations to make color operations easier. Plus makes it easy to change values
	// instead of using a 3d vector as need specular values at some places
	colour operator+(const colour &c) const {
		return colour(red + c.red, green + c.green, blue + c.blue);
	}
	colour operator*(const colour &c) const {
		return colour((red * c.red)/SINGLE_COL_MAX,\
			(green * c.green)/SINGLE_COL_MAX,\
			(blue * c.blue)/SINGLE_COL_MAX);
	}
	
	colour operator*(const double d) const {
		return colour(red*d, green*d, blue*d);
	}
	
};


struct ray {
	Vector3d  dir;
	Vector3d  origin;
};

class shape {
	public:
	virtual double intersect(ray r)=0;
	virtual Vector3d  get_normal(const Vector3d  pos) const=0;
	colour col;
	surface surf;
	virtual ~shape() {}
};

class light {
	public:
	Vector3d  pos;
	colour col;
	light(Vector3d  npos, colour nc=colour(255, 255, 255)): pos(npos), col(nc) {}
};


//In world.cpp
std::vector<shape*>* getMeshWorld();
std::vector<light*>* getLights();
std::vector<shape*>* partAWorld();
std::vector<light*>* partALights();
std::vector<shape*>* partBWorld();
std::vector<light*>* partBLights();
std::vector<shape*>* partCWorld();
std::vector<shape*>* partEWorld(int n_spheres);
std::vector<light*>* partELights();

void freeWorld(std::vector<shape*> *w);
void freeLights(std::vector<light*> *l);
Vector3d  operator*(double s, const Vector3d  &v);


