#include "Raster.hpp"

// Arrow Constructor
Arrow::Arrow(){
	float x	= 0.0;
	float y	= 0.0;
	float w	= 0.0;
	float h	= 0.0;
};

// QuadVertex Constructor
QuadVertex::QuadVertex(){
	float x	= 0.0;
	float y	= 0.0;
	float r	= 1.0;
	float g	= 1.0;
	float b	= 1.0;
	float a	= 1.0;
};

// QuadVertex Color Map
void QuadVertex::set_color(float input){
	if(input != input){
		this->a = 0.0;
		return;
	}
	this->a = 1.0;
	if(input < 0.25){
		this->r = 0;
		this->g = 4*input;
		this->b = 1;
	}else if(input < 0.50){
		this->r = 0;
		this->g = 1;
		this->b = 1.0-4*(input-0.25);
	}else if(input < 0.75){
		this->r = 4*(input-0.50);
		this->g = 1;
		this->b = 0;
	}else{
		this->r = 1;
		this->g = 1.0-4*(input-0.75);
		this->b = 0;
	}
};

// Field Constructor
Field::Field(string name, int height, int width, int depth, float sim_n_bound, float sim_s_bound, float sim_e_bound, float sim_w_bound){

	this->name			= name;

	// Data array dimension sizes
	this->height		= height;
	this->width			= width;
	this->depth			= depth;

	// Geographical bounds
	this->sim_n_bound	= sim_n_bound;
	this->sim_s_bound	= sim_s_bound;
	this->sim_e_bound	= sim_e_bound;
	this->sim_w_bound	= sim_w_bound;

	// Geographical resolution
	this->x_res			= (sim_e_bound-sim_w_bound)/width;
	this->y_res			= (sim_n_bound-sim_s_bound)/height;

};

// Raster Constructor
Raster::Raster(string name, int height, int width, int depth, vector<float> data, float sim_n_bound, float sim_s_bound, float sim_e_bound, float sim_w_bound):
		Field(name, height, width, depth, sim_n_bound, sim_s_bound, sim_e_bound, sim_w_bound){

	this->data			= data;

	// Determining value scale and limits
	this->min_value	=  1e9;
	this->max_value	= -1e9;
	for(auto const &d : this->data)
		if(d == d){
			if(d > this->max_value)
				this->max_value = d;
			if(d < this->min_value)
				this->min_value = d;
		}
	this->scale = this->max_value-this->min_value;

	// Building raster viz structure
	for(float d = 0; d < depth; d++){
		vector<Quad> layer;
		for(float i = 0; i < width-1; i++)
			for(float j = 0; j < height-1; j++){
				Quad quad;
				int ct = 0;
				vector<pair<int,int>> steps;
				steps.push_back(pair<int,int>(0,0));
				steps.push_back(pair<int,int>(0,1));
				steps.push_back(pair<int,int>(1,1));
				steps.push_back(pair<int,int>(1,0));
				for(auto &s : steps){
					int vi = s.first;
					int vj = s.second;
					int index = (j+vj)*width+(i+vi) + d*width*height;
					float input = this->data.at(index);
					if(input == input){
						quad.vertex[ct].set_color((input-this->min_value)/this->scale);
						quad.vertex[ct].x = (i+vi)*x_res+sim_w_bound;
						quad.vertex[ct].y = (j+vj)*y_res+sim_s_bound;
						ct++;
					}
				}
				if(ct == 4)
					layer.push_back(quad);
			}
		this->visual_data.push_back(layer);
	}

};

// VectorField Constructor
VectorField::VectorField(string name, int height, int width, int depth, vector<float> data_x, vector<float> data_y, float sim_n_bound, float sim_s_bound, float sim_e_bound, float sim_w_bound):
		Field(name, height, width, depth, sim_n_bound, sim_s_bound, sim_e_bound, sim_w_bound){

	this->data_x			= data_x;
	this->data_y			= data_y;

	// Determining value scale and limits
	this->min_value	=  1e9;
	this->max_value	= -1e9;
	for(auto const &d : this->data_x)
		if(d == d){
			if(d > this->max_value)
				this->max_value = d;
			if(d < this->min_value)
				this->min_value = d;
		}
	for(auto const &d : this->data_y)
		if(d == d){
			if(d > this->max_value)
				this->max_value = d;
			if(d < this->min_value)
				this->min_value = d;
		}
	this->scale = this->max_value-this->min_value;

	// Building raster viz structure
	for(float d = 0; d < depth; d++){
		vector<Arrow> layer;
		for(float i = 0; i < width; i++)
			for(float j = 0; j < height; j++){
				Arrow arrow;
				int index = i + j*width + d*width*height;
				float px = this->data_x.at(index);
				float py = this->data_y.at(index);
				if(px == px && py == py){
					arrow.i = i;
					arrow.j = j;
					arrow.x = i*x_res+sim_w_bound;
					arrow.y = j*y_res+sim_s_bound;
					arrow.w = px/this->scale;
					arrow.h = py/this->scale;
					layer.push_back(arrow);
				}
			}
		this->visual_data.push_back(layer);
	}

};

void VectorField::render(int depth_layer, float scn_scale, float vector_decimation,float x_min,float x_max,float y_min,float y_max){
	int deci = int(vector_decimation*scn_scale);
	if(deci < 1)
		deci = 1;
	glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO);
	glColor4f(1.0, 1.0, 1.0, 1.0);
	for(auto &a : this->visual_data.at(depth_layer))
		if(a.i%deci == 0 && a.j%deci == 0 && a.x > x_min && a.y > y_min && a.x < x_max && a.y < y_max){
			float theta = 180*atan2(this->y_res*a.h,this->y_res*a.w)/PI;
			float mag = ((float)deci)*sqrt(pow(this->y_res*a.h,2) + pow(this->x_res*a.w,2));
			glPushMatrix();
				glTranslatef(a.x, a.y, 0);
				glScalef(mag,mag,1);
				glRotatef(theta,0,0,1);
				glBegin(GL_LINES);
					glVertex2f(0.00, 0.00);
					glVertex2f(0.60, 0.00);
				glEnd();
				glBegin(GL_TRIANGLES);
					glVertex2f(1.00, 0.00);
					glVertex2f(0.60, 0.15);
					glVertex2f(0.60,-0.15);
				glEnd();
			glPopMatrix();
		}
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
};

void Raster::render(int depth_layer){
	for(auto &q : this->visual_data.at(depth_layer)){
		glBegin(GL_QUADS);
			for(auto &v : q.vertex){
				glColor4f(v.r,v.g,v.b,v.a);
				glVertex2f(v.x,v.y);
			}
		glEnd();
	}
};

