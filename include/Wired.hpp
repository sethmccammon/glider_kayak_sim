#ifndef WIRED_H
#define WIRED_H
#include "includes.hpp"

/*
Wired-Shaped object representation.
Wire-shaped objects are defined by a list of points representing a polygon.
*/

#define NONE	Color(0.0, 0.0, 0.0, 0.0)
#define BLACK	Color(0.0, 0.0, 0.0, 1.0)
#define TBLACK	Color(0.0, 0.0, 0.0, 0.5)
#define WHITE	Color(1.0, 1.0, 1.0, 1.0)
#define GREEN	Color(0.0, 1.0, 0.0, 1.0)
#define RED		Color(1.0, 0.0, 0.0, 1.0)
#define YELLOW	Color(1.0, 1.0, 0.0, 1.0)
#define ORANGE	Color(1.0, 0.5, 0.0, 1.0)

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

	static Color pick(string name){
		if(0 == name.compare(string("NONE")))
			return NONE;
		if(0 == name.compare(string("BLACK")))
			return BLACK;
		if(0 == name.compare(string("TBLACK")))
			return TBLACK;
		if(0 == name.compare(string("WHITE")))
			return WHITE;
		if(0 == name.compare(string("GREEN")))
			return GREEN;
		if(0 == name.compare(string("RED")))
			return RED;
		if(0 == name.compare(string("YELLOW")))
			return YELLOW;
		if(0 == name.compare(string("ORANGE")))
			return ORANGE;
	}
};

class Wired{
	public:
		vector<pair<double,double>> shape; // Object visual shape (List of points for polygon)
		vector<tuple<double,double,double>> hist; // Position and orientation history
		int hist_size;
		double x;	// Position coordinate x
		double y;	// Position coordinate y
		double t;	// Object heading theta (in Degrees)
		double w;	// Object width
		double h;	// Object height
		Color stroke;
		Color fill;
		bool selected; 
		Wired(double w=1.0, double h=1.0, Color stroke=WHITE, Color fill=NONE, int hist_size=100, double t=0.0, double x=0.0, double y=0.0);
		Wired(string filemane, double s=1.0, Color stroke=WHITE, Color fill=NONE, int hist_size=100, double t=0.0, double x=0.0, double y=0.0);
		virtual ~Wired();
		void render(bool global=true);
		bool screen_click(double sel_x, double sel_y);
		void point_selection(double sel_x, double sel_y);
		void area_selection(double min_x, double max_x, double min_y, double max_y);
		void update(double x, double y, double t);
		int load_shape(string filemane);
};

#endif
