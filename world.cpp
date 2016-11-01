#include<iostream>
#include<stdio.h>
#include<vector>
#include<cmath>
#include "ray.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
using namespace std;
using namespace Eigen;


Vector3d operator*(double s, const Vector3d &v) {
	return Vector3d(s*v(0), s*v(1), s*v(2));
}

//Triangle with verticies pos, pos + p and pos + q.
class triangle: public shape {
	Vector3d p, q, normal;	
	public:
	Vector3d pos;
	triangle(Vector3d np, Vector3d nq, Vector3d npos, colour &ncol, surface &s):
		p(np), q(nq), pos(npos) {
		col = ncol;
		surf = s;
		normal = p.cross(q);
	}
	
	Vector3d get_normal(const Vector3d ignore) const {
		return normal;
	}
	
	//Moller-Trumbore intersection algorithm
	double intersect(ray r) {
		Vector3d vec, T;
		double det, u, v, t;
		vec = r.dir.cross(q);
		det = p.dot(vec);
		//std::cout << "det = " << det << std::endl;
		//If det is zero then ray in plane of triangle
		if(det < INTERSECT_EPSILON && det > -INTERSECT_EPSILON) return 0.0;
		//Get ray from pos to ray origin
		T = r.origin - pos;
		u = T.dot(vec)/det;
		//Test u
		if(u < 0 || u > 1.0) return 0.0;
		//and v (+u)
		vec = T.cross(p);
		v = r.dir.dot(vec)/det;
		if(v < 0 || v + u > 1.0) return 0.0;
		//Intersect at origin + t*dir
		t = q.dot(vec)/det;
		//Check we're not behind the ray origin
		if(t < INTERSECT_EPSILON) return 0.0;
		//return ray intersect parameter: point = Origin + t*dir
		return t;
	}
	 
};

class sphere: public shape {
	public:
	Vector3d pos;
	double radius;
	sphere(double nr, Vector3d np, colour &ncol, surface &s):
		pos(np), radius(nr){
		col = ncol;
		surf = s;
	}
	
	Vector3d get_normal(const Vector3d p) const {
		return p - pos;
	}

	double intersect(ray r) {
		double a, b, c, d, dsq, s1, s2;
		c = r.origin.dot(r.origin) + pos.dot(pos) - 2*r.origin.dot(pos) - radius*radius;
		b = 2 * r.dir.dot(r.origin - pos);
		a = r.dir.dot(r.dir);
		if(a < INTERSECT_EPSILON) {
			fprintf(stderr, "Warn: sphere::intersect: a < ITERSECT_EPSILON\n");
			return 0.0;
		}
		dsq = b*b - 4*a*c;
		if(dsq <= 0) {
			return 0;
		}
		d = std::sqrt(dsq);
		s1 = (-b+d)/(2*a);
		s2 = (-b-d)/(2*a);
		if(s1 < s2) {
			if(s1 > INTERSECT_EPSILON) {
				return s1;
			}
		}
		if(s2 > INTERSECT_EPSILON) {
			return s2;
		} else {
			return s1 > INTERSECT_EPSILON ? s1 : 0.0;
		}
	}
	
};



std::vector<shape*>* getMeshWorld() {
	std::vector<shape*> *w = new std::vector<shape*>();
	
	colour red  = colour(255, 0, 0, 1.0);
	colour blue = colour(0, 0, 255, 1.0);
	colour green = colour(0, 255.0, 0, 1.0);

	surface shiny;
	shiny.reflection = 0.8;
	shiny.specular = 1.0;
	shiny.diffusion = 0.9;

	// w->push_back((shape*) (new sphere(2, Vector3d(0, 3, 15), red, shiny)));

		ifstream fin ;
					    // fin.open("/Users/Rachit/Documents/iqra_cg/tracer3/bumpy_cube.off");
					    fin.open("/Users/Rachit/Documents/iqra_cg/tracer3/bunny.off");
					    
					    int nrows,nrows2;
					    string output;
					    if(fin.is_open())
					    {
					        fin >> output;
					        fin >> output;
					        nrows=std::stoi(output);
					        fin >> output;
					        nrows2 = std::stoi(output);
					        fin >> output;

					    }

					    Eigen::ArrayXXf X = Eigen::ArrayXXf::Zero(nrows,3);
					    Eigen::ArrayXXd Y = Eigen::ArrayXXd::Zero(nrows2,4);

					    if (fin.is_open())
					    {
					        for (int row = 0; row < nrows; row++)
					            for (int col = 0; col < 3; col++)
					            {
					                float item = 0.0;
					                fin >> item;
					                X(row, col) = item;
					            }

					    }
					   // cerr << "X = " << endl << X << endl;
					    if (fin.is_open())
					    {
					        for (int row = 0; row < nrows2; row++)
					            for (int col = 0; col < 4; col++)
					            {
					                float item = 0.0;
					                fin >> item;
					                Y(row, col) = item;
					            }
					        fin.close();
					    }
					    // cerr<<"Y ="<< endl << Y <<endl;

					    for(unsigned k=0 ; k < Y.rows();k++) {
					    	// if(k%100==0)
					    	// cerr<<"at row-->"<<k<<endl;
					    	w->push_back((shape*) (new triangle(
					    		   Vector3d(X(Y(k, 1), 0), X(Y(k, 1), 1), X(Y(k, 1), 2)),
                Vector3d(X(Y(k, 2), 0), X(Y(k, 2), 1), X(Y(k, 2), 2)),
                Vector3d(X(Y(k, 3), 0), X(Y(k, 3), 1), X(Y(k, 3), 2)),
                red,shiny
					    		)));


}






	

	//One sphere
	// w->push_back((shape*) (new sphere(3, Vector3d(5, 5, 10), red, shiny)));
	// w->push_back((shape*) (new sphere(1.5, Vector3d(0, 0, -5), red, shiny)));
	return w;

}



std::vector<shape*>* partAWorld() {
	std::vector<shape*> *w = new std::vector<shape*>();
	
	colour red  = colour(255, 0, 0, 1.0);
	colour blue = colour(0, 0, 255, 1.0);
	colour green = colour(0, 255.0, 0, 1.0);


	//SETTING ONLY DIFFUSE as using imple Lambertian shading
	surface lambert;
	lambert.reflection = 0.0;
	lambert.specular = 0.0;
	lambert.diffusion = 0.9;

	w->push_back((shape*) (new sphere(5, Vector3d(0, 3, 15), red, lambert)));
	w->push_back((shape*) (new sphere(5, Vector3d(10, 10, 15), red, lambert)));

	// w->push_back((shape*) (new sphere(3, Vector3d(5, 5, 10), red, shiny)));
	// w->push_back((shape*) (new sphere(1.5, Vector3d(0, 0, -5), red, shiny)));
	return w;

}
std::vector<shape*>* partBWorld() {
	std::vector<shape*> *w = new std::vector<shape*>();
	
	colour red  = colour(255, 0, 0, 1.0);
	colour blue = colour(0, 0, 255, 1.0);
	colour green = colour(0, 255.0, 0, 1.0);


	//SETTING ONLY DIFFUSE as using imple Lambertian shading
	surface lambert;
	lambert.reflection = 0.0;
	lambert.specular = 0.0;
	lambert.diffusion = 0.9;
	//Diffuse+specular
	surface specular;
	specular.reflection = 0.0;
	specular.specular = 1.0;
	specular.diffusion = 0.9;

	w->push_back((shape*) (new sphere(10, Vector3d(0, 3, 15), red, lambert)));
	w->push_back((shape*) (new sphere(5, Vector3d(10, 10, 15), blue, specular)));

	// w->push_back((shape*) (new sphere(3, Vector3d(5, 5, 10), red, shiny)));
	// w->push_back((shape*) (new sphere(1.5, Vector3d(0, 0, -5), red, shiny)));
	return w;

}
std::vector<shape*>* partCWorld() {
	std::vector<shape*> *w = new std::vector<shape*>();
	
	colour red  = colour(255, 0, 0, 1.0);
	colour blue = colour(0, 0, 255, 1.0);
	colour green = colour(0, 255.0, 0, 1.0);


	//SETTING ONLY DIFFUSE as using imple Lambertian shading
	surface lambert;
	lambert.reflection = 0.0;
	lambert.specular = 0.0;
	lambert.diffusion = 0.9;
	//Diffuse+specular
	surface specular;
	specular.reflection = 0.0;
	specular.specular = 1.0;
	specular.diffusion = 0.9;

	w->push_back((shape*) (new sphere(10, Vector3d(0, 3, 15), red, lambert)));
	w->push_back((shape*) (new sphere(5, Vector3d(10, 10, 15), blue, specular)));

	// w->push_back((shape*) (new sphere(3, Vector3d(5, 5, 10), red, shiny)));
	// w->push_back((shape*) (new sphere(1.5, Vector3d(0, 0, -5), red, shiny)));
	return w;

}
std::vector<shape*>* partEWorld(int n_spheres) {
	
	surface shiny;
	shiny.reflection = 0.4;
	shiny.specular = 1.0;
	shiny.diffusion = 0.9;
	std::vector<shape*> *w = new std::vector<shape*>();
	int i, j, k;
	for(i=0; i<3; i++) {
	for(j=0; j<3; j++) {
	for(k=0; k<3; k++) {
		if(n_spheres > 0) {
			colour c = colour(i*127.5, 255-j*127.5, ((int) (127.5+(k*127.5)))%382);
			w->push_back((shape*) (new sphere(1, Vector3d(i*4-5.5, j*4-6, k*4+14), c, shiny)));
			n_spheres -= 1;
		}
	}
	}
	}
	return w;

}



void freeWorld(std::vector<shape*> *w) {
	while(w->size() > 0) {
		delete (w->back());
		w->pop_back();
	}
	delete w;
}

std::vector<light*>* getLights() {
	std::vector<light*> *l = new std::vector<light*>();
	l->push_back(new light(Vector3d(15, 15, 5), colour(255, 255, 255)));
	return l;
}
std::vector<light*>* partALights() {
	std::vector<light*> *l = new std::vector<light*>();
	l->push_back(new light(Vector3d(15, 15, 5), colour(255, 255, 255)));
	return l;
}
std::vector<light*>* partBLights() {
	std::vector<light*> *l = new std::vector<light*>();
	l->push_back(new light(Vector3d(15, 15, 5), colour(255, 255, 255)));
	l->push_back(new light(Vector3d(-15, -15, 5), colour(255, 255, 255)));
		
	return l;
}
std::vector<light*>* partELights() {
	std::vector<light*> *l = new std::vector<light*>();
	l->push_back(new light(Vector3d(-5, 3, 2), colour(255, 255, 255)));
	l->push_back(new light(Vector3d(15, 15, 5), colour(255, 255, 255)));
		
	return l;
}

void freeLights(std::vector<light*> *l) {
	while(l->size() > 0) {
		delete (l->back());
		l->pop_back();
	}
	delete l;
}

