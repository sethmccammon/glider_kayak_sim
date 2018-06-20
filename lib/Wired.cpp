#include "Wired.hpp"

Wired::Wired(double w, double h, Color stroke, Color fill, int hist_size, double t, double x, double y){
	this->hist_size = hist_size;
	this->w = w;
	this->h = h;
	this->x = x;
	this->y = y;
	this->t = t;
	this->stroke = stroke;
	this->fill = fill;
	this->selected = false;
	vector<pair<double,double>> shape;
	shape.push_back(pair<double,double>(0.0,0.0));
	shape.push_back(pair<double,double>(0.0,1.0));
	shape.push_back(pair<double,double>(1.0,1.0));
	shape.push_back(pair<double,double>(1.0,0.0));
	this->shape = shape;
};

Wired::Wired(string filemane, double s, Color stroke, Color fill, int hist_size, double t, double x, double y): Wired(s, s, stroke, fill, hist_size, t, x, y){
	this->load_shape(filemane);
};

Wired::~Wired(){};

void Wired::render(bool global){
	glPushMatrix();
		if(selected)
			glLineWidth(3);
		else
			glLineWidth(1);
		// Hist
		for(int i = 0; i < this->hist.size(); i++){
			glColor4f(this->stroke.r,this->stroke.g,this->stroke.b,(double)i/this->hist.size());
			glPushMatrix();
				glTranslatef(get<0>(this->hist.at(i)),get<1>(this->hist.at(i)),0);
				glScalef(0.03*w,0.03*h,1);
				glBegin(GL_POLYGON);
					glVertex2f( 1, 0);
					glVertex2f( 0, 1);
					glVertex2f(-1, 0);
					glVertex2f( 0,-1);
				glEnd();
			glPopMatrix();
		}
		if(global){
			glTranslatef(x,y,0);
			glScalef(w,h,1);
		}
		else{
			glTranslatef(x,y,0);
			glScalef(w,-h,1);
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

bool Wired::screen_click(double sel_x, double sel_y){
	if(this->x-this->w/2 > sel_x) return false;
	if(this->x+this->w/2 < sel_x) return false;
	if(this->y-this->h/2 > sel_y) return false;
	if(this->y+this->h/2 < sel_y) return false;
	return true;
}

void Wired::point_selection(double sel_x, double sel_y){
	this->selected = screen_click(sel_x,sel_y);	
}

void Wired::area_selection(double min_x, double max_x, double min_y, double max_y){
	if(this->x-this->w/2 < min_x) return;
	if(this->x+this->w/2 > max_x) return;
	if(this->y-this->h/2 < min_y) return;
	if(this->y+this->h/2 > max_y) return;
	this->selected = true;
}

void Wired::update(double x, double y, double t){
	this->x = x;
	this->y = y;
	this->t = t;
	this->hist.push_back(tuple<double,double,double>(x,y,t));
	if(this->hist.size() > this->hist_size)
		this->hist.erase(hist.begin());
}

int Wired::load_shape(string filemane){

	char *aux = 0;
	FILE *file;	
	char fileBuffer[BUFFER_SIZE]; 

	file 	= fopen(filemane.c_str(),"r");
	if(file == NULL)
		return 1;

	shape.clear();

	while(fgets(fileBuffer,BUFFER_SIZE,file) != NULL){
		aux = fileBuffer;
		double x = atof(aux);
		while(*aux != ',')
			aux++;
		aux++;
		double y = atof(aux);
		this->shape.push_back(pair<double,double>(x,y));
	}
	
	fclose(file);

	return 0;

}
