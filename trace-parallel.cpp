#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cmath>
#include "tbb/tbb.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ray.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"
#define TIME_TRACE
#define SPECULAR_CUTOFF 0.7
#define REFLECTION_CUTOFF 0.01
#define DBL_INFINITY (1.0/0.0)
#define SCREEN_WIDTH ((double) (SCREEN_SIZE))
#define SCREEN_HEIGHT ((((double) (SCREEN_RESOLUTION_HEIGHT))/((double) (SCREEN_RESOLUTION_WIDTH))) * SCREEN_SIZE)
#define TRACE_DEPTH 5
#define AMBIENT_COLOUR (colour(32, 32, 32))
#define SPECULAR_POWER 20
#define LIGHT_FALLOFF 500
#define COL_MAX 255
#define SCREEN_RESOLUTION_WIDTH 800
#define SCREEN_RESOLUTION_HEIGHT 800
#define SCREEN_DISTANCE 1
#define SCREEN_SIZE 1.0
#define EYE_POSITION (Vector3d (0, 0, -1)) 
#define EYE_DIRECTION (Vector3d(0, 0 ,0.5)) 
#define BACKGROUND_COLOUR (colour(32, 32, 32))
#define SHADOWS_ENABLE

 using namespace Eigen;
 using namespace std;
struct s_screen {
	Vector3d  eye;
	Vector3d  botl;
	Vector3d  topl;
	Vector3d  botr;
	int height;
	int width;
};

void gen_screen(ray eye, s_screen *s) {
	Vector3d  planevec, zx_comp, zy_comp;
	Vector3d  viewdir = eye.dir.normalized();
	Vector3d  centre = eye.origin + SCREEN_DISTANCE*viewdir;
	//Form vector in eye.dir-y plane, then find z-x screen component
	//if eye.dir ~= y then z-x component is x
	if(viewdir(0) < SCREEN_ALIGN_EPSILON && viewdir(0) > -SCREEN_ALIGN_EPSILON &&\
			 viewdir(2) < SCREEN_ALIGN_EPSILON && viewdir(2) > -SCREEN_ALIGN_EPSILON) {
		zx_comp = SCREEN_WIDTH * Vector3d (1, 0, 0);
	} else {
		planevec = Vector3d (viewdir(0), viewdir(1) + 1, viewdir(2));
		zx_comp = SCREEN_WIDTH * planevec.cross(viewdir).normalized();
	}
	//Then the z-y component
	if(viewdir(1) < SCREEN_ALIGN_EPSILON && viewdir(1) > -SCREEN_ALIGN_EPSILON &&\
			 viewdir(2) < SCREEN_ALIGN_EPSILON && viewdir(2) > -SCREEN_ALIGN_EPSILON) {
		zy_comp = SCREEN_HEIGHT * Vector3d (0, 1, 0);
	} else {
		planevec = Vector3d (viewdir(0) + 1, viewdir(1), viewdir(2));
		zy_comp = -SCREEN_HEIGHT * planevec.cross(viewdir).normalized();
	}

	s->botl   = centre - (zx_comp + zy_comp);
	s->topl   = centre + (zy_comp - zx_comp);
	s->botr   = centre + (zx_comp - zy_comp);
	s->eye    = eye.origin;
	s->height = SCREEN_RESOLUTION_HEIGHT;
	s->width  = SCREEN_RESOLUTION_WIDTH;
}



//Find the nearest intersecting shpae and return a pointer to it or NULL on no intersection.
shape* trace_nearest(ray &r, std::vector<shape*> &world, double &distance) {
	double closest_distance = DBL_INFINITY;
	double t;
	shape* closest_shape = NULL;
	std::vector<shape*>::iterator v = world.begin();
	while(v != world.end()) {
		t = (**v).intersect(r);
		if(t != 0.0 && t < closest_distance) {
			closest_distance = t;
			closest_shape = *v;
		}
		v++;
	}
	distance = closest_distance;
	return closest_shape;
}



bool trace_light(Vector3d  pos, std::vector<shape*> &w, light *l, colour *c) {
	ray r;
	double t, distsq, strength;
	r.origin = pos;
	r.dir = l->pos - pos;
	shape* s;
	#ifdef SHADOWS_ENABLE
	//Check for collisions before the light source
	// cout<<"Shadow enabled";
	s = trace_nearest(r, w, t);
	#else
	s = NULL;
	#endif
	
	if(s != NULL && t < (1.0+INTERSECT_EPSILON)) {
		//Light is blocked (shadow)
		return false;
	} else {
		distsq = r.dir(0)*r.dir(0)+r.dir(1)*r.dir(1)+r.dir(2)*r.dir(2);
		strength = (distsq <= LIGHT_FALLOFF ? 1.0 : LIGHT_FALLOFF/(distsq));
		*c = l->col * strength;
		return true;
	}
}



bool trace(ray &r, std::vector<shape*> &world, std::vector<light*> &lights, int depth, double strength, colour *c) {
	double t, light_cosine;
	Vector3d  hit_position, normal, inv_ray_dir, dir_to_light;
	colour tmpcol, diff_light_col = AMBIENT_COLOUR, spec_light_col = colour(0, 0, 0);
	std::vector<light*>::iterator liter;
	shape *intersect_shape;
	intersect_shape = trace_nearest(r, world, t);
	ray reflection;
	double reflection_cosine;

	if(intersect_shape != NULL) {
		inv_ray_dir = -r.dir;
		liter = lights.begin();
		hit_position = r.origin + t*r.dir;
		normal = intersect_shape->get_normal(hit_position).normalized();
		if(inv_ray_dir.dot(normal) < 0.0) {
			normal = -normal;
		}
		reflection.origin = hit_position;
		reflection.dir    = r.dir - 2*(r.dir.dot(normal))*normal;

		//Go through each light and find its contribution
		while(liter != lights.end()) {
			dir_to_light = ((**liter).pos - hit_position).normalized();
			//Don't look for light begind the object
			light_cosine = dir_to_light.dot(normal);
			if(light_cosine > 0.0) {
				if(trace_light(hit_position, world, *liter, &tmpcol)) {
					diff_light_col = diff_light_col + tmpcol*light_cosine;
					reflection_cosine = reflection.dir.normalized().dot(dir_to_light);
					if(reflection_cosine > SPECULAR_CUTOFF) {
						spec_light_col = spec_light_col + tmpcol*std::pow(reflection_cosine, SPECULAR_POWER);
					}
				}
			}
			liter++;
		}
		
		colour diffuse_component  = diff_light_col * intersect_shape->col * intersect_shape->surf.diffusion;
		colour specular_component = spec_light_col * intersect_shape->surf.specular;
		*c = diffuse_component + specular_component;

		

		return true;
	} else {
		*c = BACKGROUND_COLOUR;
		return false;
	}
}



void ray_trace(s_screen &s, std::vector<shape*> &world, std::vector<light*> &lights) {
	Vector3d  h_step, v_step, pos;
	
	MatrixXd R = MatrixXd::Zero(800,800); // Store the color
    MatrixXd A = MatrixXd::Zero(800,800); 
    MatrixXd G = MatrixXd::Zero(800,800);
    MatrixXd B = MatrixXd::Zero(800,800);
	
	ray r;
	r.origin = s.eye;
	colour col;
	// int i, j;
	
	h_step = (s.botr - s.botl)/(s.width-1);
	v_step = (s.topl - s.botl)/(-s.height+1);
	

 tbb::parallel_for(tbb::blocked_range<std::size_t>(0, s.height),
  [&](const tbb::blocked_range<std::size_t> &range) {
	for(auto j=range.begin(); j!=range.end(); j++) {
		pos = s.topl + j*v_step;
		for(int i=0; i<s.width; i++) {
			
			r.dir = pos-s.eye;
			trace(r, world, lights, TRACE_DEPTH, 1.0, &col);

			// cout << "col.blue is " <<col.blue<<","<<col.red<<","<<col.green<<std::endl;
			B(i,j) = fmax(col.blue/255.0,0.0);
			G(i,j) = fmax(col.green/255.0,0.0);
			R(i,j) = fmax(col.red/255.0,0.0);
			A(i,j) = 1.0;
			pos += h_step;
		}
	}
});

	write_matrix_to_png(R,G,B,A,"test.png");
	
}


void ray_tracer() {
	
	s_screen screen;
	std::vector<shape*> *w;
	std::vector<light*> *l;
	
	cout << "### Ray tracer ###" << std::endl;

	ray eye = {EYE_DIRECTION, EYE_POSITION};
	gen_screen(eye, &screen);

	//EYE_DIRECTION set to 0,0,-1
	// w = getWorld();
	// l = getLights();
	// w = partAWorld();
	// l = partALights();

	//w = partBWorld();
	// l = partBLights();
	//FOr part C we use the same lights as B and change the EYE_DIRECTION to incorporate x,y,z and make it perspective
	//Thus for part C we change EYE_DIRECTION from 0,0,-1 to 2,3,4 
	// w = partCWorld();

	//For part D we get the data from the Mesh - we use the same lights as partBLights()
	//This is a big buggy as was not able to get the triange intersect logic work PERFECTLY
	// Have the paths to both the meshes commented in the Word.cpp getMeshWorld() function
	// w = getMeshWorld();

	//for shadows we need to #define SHADOWS_ENABLE from the header of THIS FILE
	//then shadows get enabled
	l=partELights();
	w=partEWorld(27);
	
	cout << w->size() << " shapes and " << l->size() << " lights." << std::endl;
	
	 ray_trace(screen, *w, *l);

	freeWorld(w);
	freeLights(l);
}



int main() {
	
	ray_tracer();
	
	return 0;
}
