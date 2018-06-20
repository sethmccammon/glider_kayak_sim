#ifndef RASTER_H
#define RASTER_H
#include "includes.hpp"

class Arrow{

	public:

		int i; int j; float x; float y; float w; float h; float r; float g; float b; float a;

		// Constructor
		Arrow();

};

class QuadVertex{

	public:

		float x; float y; float r; float g; float b; float a;

		// Constructor
		QuadVertex();

		// Color Map
		void set_color(float input);

};

class Quad{

	public:

		QuadVertex vertex[4];

};

class Field{

	public:

		string name;

		// Geographical bounds
		float sim_n_bound;
		float sim_s_bound;
		float sim_e_bound;
		float sim_w_bound;

		int height;
		int width;
		int depth;

		// Geographical resolution
		float x_res;
		float y_res;
		double scale;

		// Value limits
		double min_value;
		double max_value;

		// Constructor
		Field(string name, int height, int width, int depth, float sim_n_bound, float sim_s_bound, float sim_e_bound, float sim_w_bound);

};

class Raster : public Field{

	public:

		vector<float> data;

		// Data Arrays
		vector<vector<Quad>> visual_data;

		// Constructor
		Raster(string name, int height, int width, int depth, vector<float> data, float sim_n_bound, float sim_s_bound, float sim_e_bound, float sim_w_bound);

		void render(int depth_layer);

};

class VectorField : public Field{

	public:

		vector<float> data_x;
		vector<float> data_y;

		// Data Arrays
		vector<vector<Arrow>> visual_data;

		// Constructor
		VectorField(string name, int height, int width, int depth, vector<float> data_x, vector<float> data_y, float sim_n_bound, float sim_s_bound, float sim_e_bound, float sim_w_bound);

		void render(int depth_layer, float scn_scale, float vector_decimation,float x_min,float x_max,float y_min,float y_max);

};

class Layers{

	public:

		bool display_temperature;
		bool display_currents;
		int depth_layer;
		float vector_decimation;

		Raster *temperature;
		Raster *salinity;
		VectorField *currents;

		Layers(){
			this->display_temperature	= true;
			this->display_currents		= true;
			this->depth_layer			= 0;
			this->vector_decimation		= 0.0;
			this->temperature			= NULL;
			this->salinity				= NULL;
			this->currents				= NULL;
		};

		void render(float scn_scale,float x_min,float x_max,float y_min,float y_max){
			if(display_temperature && temperature != NULL)
				this->temperature->render(this->depth_layer);
			if(!display_temperature && salinity != NULL)
				this->salinity->render(this->depth_layer);
			if(display_currents && currents != NULL)
				this->currents->render(this->depth_layer, scn_scale, this->vector_decimation,x_min,x_max,y_min,y_max);
		};

		void switch_depth(int delta){
			this->depth_layer += delta;
			if(this->depth_layer < 0)
				this->depth_layer = 0;
			if(this->depth_layer >= temperature->depth)
				this->depth_layer = temperature->depth-1;
		};

		void switch_raster(){
			this->display_temperature = !display_temperature;
		};

		void switch_currents(){
			this->display_currents = !display_currents;
		};

		string get_description(){
			char description[BUFFER_SIZE];
			string ret_val("");
			if(display_temperature && temperature != NULL){
				sprintf(description, " (%6.2f - %6.2f Celsius)", this->temperature->min_value, this->temperature->max_value);
				ret_val += string("Temperature")+string(description);
			}
			if(!display_temperature && salinity != NULL){
				sprintf(description, " (%6.2f - %6.2f)", this->salinity->min_value, this->salinity->max_value);
				ret_val += string("Salinity")+string(description);
			}
			if(display_currents && currents != NULL){
				sprintf(description, " (%6.2f - %6.2f m/s)", this->currents->min_value, this->currents->max_value);
				ret_val += string(" and Currents")+string(description);
			}
			return ret_val;
		};

};

#endif
