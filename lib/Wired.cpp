#include "Wired.hpp"

Wired::Wired(vector<pair<double, double>> shape, double s, double t, Color stroke, Color fill, double x, double y){
	this->shape = shape;
	this->s = s;
	this->x = x;
	this->y = y;
	this->t = t;
	this->stroke = stroke;
	this->fill = fill;
	this->selected = false;
};

Wired::~Wired(){};

void Wired::render(bool global){
	glPushMatrix();
		if(selected)
			glLineWidth(3);
		else
			glLineWidth(1);
		if(global){
			glTranslatef(-x,-y,0);
			glScalef(s,s,1);
		}
		else{
			glTranslatef(x,y,0);
			glScalef(s,-s,1);
		}
		glRotatef(t,0,0,1);
		glTranslatef(-0.5,-0.5,0);
		glColor4f(this->fill.r,this->fill.g,this->fill.b,this->fill.a);
		glBegin(GL_POLYGON);
			for(auto p : this->shape)
				glVertex2f(p.first,p.second);
		glEnd();
		glColor4f(this->stroke.r,this->stroke.g,this->stroke.b,this->stroke.a);
		glBegin(GL_LINE_LOOP);
			for(auto p : this->shape)
				glVertex2f(p.first,p.second);
		glEnd();
		glColor3f(1.0,1.0,1.0);
		glLineWidth(1);
	glPopMatrix();
};

void Wired::point_selection(double sel_x, double sel_y){
	this->selected = false;
	if(this->x-this->s/2 > sel_x) return;
	if(this->x+this->s/2 < sel_x) return;
	if(this->y-this->s/2 > sel_y) return;
	if(this->y+this->s/2 < sel_y) return;
	this->selected = true;	
}

void Wired::area_selection(double min_x, double max_x, double min_y, double max_y){
	if(this->x-this->s/2 < min_x) return;
	if(this->x+this->s/2 > max_x) return;
	if(this->y-this->s/2 < min_y) return;
	if(this->y+this->s/2 > max_y) return;
	this->selected = true;
}