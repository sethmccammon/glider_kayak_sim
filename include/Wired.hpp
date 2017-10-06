#ifndef WIRED_H
#define WIRED_H
#include "includes.hpp"

/*
Wired-Shaped object representation.
Wire-shaped objects are defined by a list of points representing a polygon.
*/

#define NONE Color(0,0,0,0)
#define BLACK Color(0,0,0,1)
#define WHITE Color(1,1,1,1)
#define GREEN Color(0,1,0,1)

class Color{
	public:
		double r;
		double g;
		double b;
		double a;
		Color(double r=1.0, double g=1.0, double b=1.0, double a=1.0){
			this->r = r;
			this->g = g;
			this->b = b;
			this->a = a;
		}
};

class Wired{
	public:
		vector<pair<double, double>> shape; // Object visual shape (List of points for polygon)
		double x;	// Position coordinate x
		double y;	// Position coordinate y
		double t;	// Object heading theta (in Degrees)
		double s;	// Object scale
		Color stroke;
		Color fill;
		bool selected; 
		Wired(vector<pair<double, double>> shape, double s=1.0, double t=0.0, Color stroke=WHITE, Color fill=NONE, double x=0.0, double y=0.0);
		virtual ~Wired();
		void render(bool global=true);
		void point_selection(double sel_x, double sel_y);
		void area_selection(double min_x, double max_x, double min_y, double max_y);
};

#endif
