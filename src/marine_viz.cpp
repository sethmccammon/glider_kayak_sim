#include <ros/ros.h>
#include <tf/tf.h>
#include <glider_kayak_sim/UnderwaterGeoPose.h>
#include <glider_kayak_sim/UnderwaterGeoPoint.h>
#include <glider_kayak_sim/SimQuery.h>
#include <glider_kayak_sim/SimSnapshot.h>
#include "includes.hpp"
#include "Wired.hpp"
#include "Raster.hpp"

map<string,bool> viz_bool;
map<string,int> viz_int;
map<string,float> viz_float;
map<string,string> viz_string;
map<string,string> viz_keys;
map<string,string> viz_bkeys;

vector<double> depths;
map<string,Wired*> robots;
map<string,Wired*> buttons;
Layers layers;

Wired *cursor;
Wired *flag;

// Simulator Service 
int num_kayaks			= 0;
int num_gliders			= 0;
float sim_update_rate	= 0.0;

// Status
double origin_x			= 0.0;
double origin_y			= 0.0;

double old_mouse_x		= 0;
double old_mouse_y		= 0;
double mouse_gnd_x		= 0;
double mouse_gnd_y		= 0;

// Camera position/motion
double view_x			= 0;
double view_y			= 0;
double x_min			= 0;
double x_max			= 0;
double y_min			= 0;
double y_max			= 0;

// User input flags
bool mouse_l			= 0;
bool mouse_m			= 0;
bool mouse_r			= 0;
bool lock_view_robot	= 0;

// Keyboard mov flags
bool keyLeft			= 0;
bool keyRight			= 0;
bool keyDown			= 0;
bool keyUp				= 0;
bool keyZoomIn			= 0;
bool keyZoomOut			= 0;

// Mouse mov flags
bool mouseLeft			= 0;
bool mouseRight			= 0;
bool mouseDown			= 0;
bool mouseUp			= 0;

double selX1			= 0;
double selY1			= 0;
double selX2			= 0;
double selY2			= 0;

char statusBuffer[BUFFER_SIZE];
char mouse_coord_buffer[BUFFER_SIZE];

void print(string str);

void underwater_geo_pose_callback(const ros::MessageEvent<glider_kayak_sim::UnderwaterGeoPose const>& event);

void get_roms_data(string world_snapshot_topic);

void getScreenResolution(int &s, int &v);

void iniGl();

void areaSelect();

void singleSelect();

void mouseButton(int b,int s,int x,int y);

void cursorUpdate(int x,int y);

void mouseMove(int x, int y);

void mouseAction(int x, int y);

void set_goal(double x, double y);

void glutMouseFunc(int button, int state, int x, int y);

void keyPressed(unsigned char key, int x, int y);

void keyReleased(unsigned char key, int x, int y);

void updateValues(int n);

void RenderScene();

int main(int argc, char **argv){

	string world_sim_topic;
	string world_snapshot_topic;
	string underwater_geo_pose_topic;
	string kayak_namespace;
	string glider_namespace;
	string package_path;

	view_x				= origin_x;
	view_y				= origin_y;

	// Initializing node
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle n;

	// Getting Viz Parameters
	ros::param::get("viz_bool", viz_bool);
	ros::param::get("viz_int", viz_int);
	ros::param::get("viz_float", viz_float);
	ros::param::get("viz_string", viz_string);
	ros::param::get("viz_keys", viz_keys);
	ros::param::get("viz_bkeys", viz_bkeys);
	n.getParam("depths", depths);

	// Getting Global Parameters
	n.param<int>("num_kayaks",						num_kayaks, 0);
	n.param<int>("num_gliders",						num_gliders, 0);
	n.param<float>("sim_update_rate",				sim_update_rate, 0.0);
	n.param<string>("world_sim_topic",				world_sim_topic, "default");
	n.param<string>("world_snapshot_topic",			world_snapshot_topic, "default");
	n.param<string>("underwater_geo_pose_topic",	underwater_geo_pose_topic, "default");
	n.param<string>("kayak_namespace",				kayak_namespace, "default");
	n.param<string>("glider_namespace",				glider_namespace, "default");
	n.param<string>("package_path",					package_path, "default");

	thread th(get_roms_data,world_snapshot_topic);

	vector<ros::Subscriber> subscribers;

	layers.vector_decimation = viz_float["vector_decimation"];

	// Initializing buttons
	float i = 0.5;
	for(auto &k : viz_bkeys){
		buttons[k.first] = new Wired(viz_float["botton_width"], viz_float["botton_height"], Color::pick(viz_string["button_stroke"]), Color::pick(viz_string["button_fill"]), 0);
		buttons[k.first]->update(i*viz_float["botton_width"],viz_int["scn_height"]-0.5*viz_float["botton_height"], 0);
		i += 1.0;
	}

	// Initializing kayaks
	for(int i = 0; i < num_kayaks; i++){
		char topic_name[BUFFER_SIZE];
		sprintf(topic_name, (kayak_namespace+underwater_geo_pose_topic).c_str(), i);
		robots[string(topic_name)] = new Wired(package_path+viz_string["kayak_shape"], viz_float["kayak_size"], Color::pick(viz_string["kayak_stroke"]), Color::pick(viz_string["kayak_fill"]), viz_int["hist_size"]);
		subscribers.push_back(n.subscribe(topic_name, viz_int["topic_queue_size"], underwater_geo_pose_callback));
	}

	// Initializing gliders
	for(int i = 0; i < num_gliders; i++){
		char topic_name[BUFFER_SIZE];
		sprintf(topic_name, (glider_namespace+underwater_geo_pose_topic).c_str(), i);
		robots[string(topic_name)] = new Wired(package_path+viz_string["glider_shape"], viz_float["glider_size"], Color::pick(viz_string["glider_stroke"]), Color::pick(viz_string["glider_fill"]), viz_int["hist_size"]);
		subscribers.push_back(n.subscribe(topic_name, viz_int["topic_queue_size"], underwater_geo_pose_callback));
	}

	// Ready message
	print(string("Initialized."));

	// Cursor and goal objects
	cursor = new Wired(package_path+viz_string["pointer_shape"], viz_int["pointer_size"], Color::pick(viz_string["pointer_stroke"]), Color::pick(viz_string["pointer_fill"]), 0, viz_float["pointer_angle"]);
	flag = new Wired(package_path+viz_string["goal_shape"], viz_float["flag_size"], Color::pick(viz_string["flag_stroke"]), Color::pick(viz_string["flag_fill"]));

	// Initializing graphics
	glutInit(&argc, argv);
	iniGl();

	// Main loop
	glutMainLoop();

	return 0;
}

void underwater_geo_pose_callback(const ros::MessageEvent<glider_kayak_sim::UnderwaterGeoPose const>& event){
	string topic = event.getConnectionHeader().at("topic");
	const glider_kayak_sim::UnderwaterGeoPoseConstPtr& msg = event.getMessage();
	double roll		= 0.0;
	double pitch	= 0.0;
	double yaw		= 0.0;
	tf::Quaternion qt(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
	robots[topic]->update(msg->position.longitude, msg->position.latitude, 180*yaw/PI);
}

void get_roms_data(string world_snapshot_topic){

	vector<string>raster_names = {"Temperature", "Salinity", "Current_u", "Current_v"};

	vector<float> temperature;
	vector<float> salinity;
	vector<float> current_u;
	vector<float> current_v;

	float height;
	float width;
	float depth;

	float n_bound;
	float s_bound;
	float e_bound;
	float w_bound;
	ros::NodeHandle n;

	ros::ServiceClient sim_client;
	sim_client = n.serviceClient<glider_kayak_sim::SimSnapshot>(world_snapshot_topic);
	sim_client.waitForExistence(ros::Duration((int)viz_float["world_sim_timeout"]));

	n.param<float>("sim_w_bound",		w_bound, 0);
	n.param<float>("sim_e_bound",		e_bound, 0);
	n.param<float>("sim_s_bound",		s_bound, 0);
	n.param<float>("sim_n_bound",		n_bound, 0);

	// Moving view to region of interest
	origin_x			= (e_bound+w_bound)/2;
	origin_y			= (n_bound+s_bound)/2;
	view_x				= origin_x;
	view_y				= origin_y;

	// Gathering environment data
	glider_kayak_sim::SimSnapshot world_snapshot;
	for(auto &name : raster_names){
		print(string(": Fetching ")+name+string(" layer ..."));
		world_snapshot.request.ocean_feature = &name-&raster_names[0];
		if(sim_client && sim_client.call(world_snapshot)){
			height	= world_snapshot.response.snapshot.height;
			width	= world_snapshot.response.snapshot.width;
			depth	= world_snapshot.response.snapshot.depth;
			if(&name-&raster_names[0] == 0){
				temperature = world_snapshot.response.snapshot.data;
				layers.temperature = new Raster("temperature", height, width, depth, temperature, n_bound, s_bound, e_bound, w_bound);
			}
			if(&name-&raster_names[0] == 1){
				salinity = world_snapshot.response.snapshot.data;
				layers.salinity = new Raster("salinity", height, width, depth, salinity, n_bound, s_bound, e_bound, w_bound);
			}
			if(&name-&raster_names[0] == 2)
				current_u = world_snapshot.response.snapshot.data;
			if(&name-&raster_names[0] == 3){
				current_v = world_snapshot.response.snapshot.data;
				layers.currents = new VectorField("current", height, width, depth, current_u, current_v, n_bound, s_bound, e_bound, w_bound);
			}
		}
	}
}

void print(string str){
	ROS_INFO((string(NODE_NAME)+string(": ")+str).c_str());
}

// OpenGL initialization
void iniGl(){
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(viz_int["scn_width"],viz_int["scn_height"]);
	glutCreateWindow(viz_string["window_title"].c_str());
	glutMouseFunc(&mouseButton);
	glutMotionFunc(&mouseAction);
	glutPassiveMotionFunc(&mouseMove);
	glutKeyboardFunc(&keyPressed);
	glutKeyboardUpFunc(&keyReleased);
	glutDisplayFunc(&RenderScene);
	glutIdleFunc(&RenderScene);
	glutTimerFunc(1,updateValues,0);
	glMatrixMode(GL_PROJECTION);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glDisable(GL_ALPHA_TEST);
	gluOrtho2D(x_min,x_max,y_min,y_max);
	if(viz_bool["full_screen"])
		glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);
}

// Mouse click selection
void singleSelect(){

	// Robot Click Selection
	for(auto &r : robots)
		r.second->point_selection(selX1,selY1);

	// Button Click
	if(buttons["switch_raster"]->screen_click(cursor->x,cursor->y))
		layers.switch_raster();
	if(buttons["switch_currents"]->screen_click(cursor->x,cursor->y))
		layers.switch_currents();
	if(buttons["increase_depth"]->screen_click(cursor->x,cursor->y))
		layers.switch_depth(+1);
	if(buttons["decrease_depth"]->screen_click(cursor->x,cursor->y))
		layers.switch_depth(-1);
	if(buttons["reset_zoom"]->screen_click(cursor->x,cursor->y))
		viz_float["scn_scale"] = viz_float["scn_scale_def"];
	if(buttons["reset_view_pos"]->screen_click(cursor->x,cursor->y)){
		view_x = origin_x;
		view_y = origin_y;
	}
	if(buttons["lock_view_robot"]->screen_click(cursor->x,cursor->y))
		lock_view_robot = !lock_view_robot;
	if(buttons["zoon_in"]->screen_click(cursor->x,cursor->y))
		viz_float["scn_scale"] /= viz_float["zoom_speed"];
	if(buttons["zoon_out"]->screen_click(cursor->x,cursor->y))
		viz_float["scn_scale"] *= viz_float["zoom_speed"];
}

// Mouse area selection
void areaSelect(){
	double min_x = (selX2 > selX1)?selX1:selX2;
	double max_x = (selX2 > selX1)?selX2:selX1;
	double min_y = (selY2 > selY1)?selY1:selY2;
	double max_y = (selY2 > selY1)?selY2:selY1;
	for(auto &r : robots)
		r.second->area_selection(min_x, max_x, min_y, max_y);
}

// Screen to space conversion
double scn2space(int a){
	return viz_float["scn_scale"]*((double)a);
}

// Space to screen conversion
double space2scn(double a){
	return ((double)a)/viz_float["scn_scale"];
}

// Updating cursor position variables
void cursorUpdate(int x,int y){	
	cursor->x	= x;
	cursor->y	= y;
	mouseLeft	= (x == 0);
	mouseRight	= (x == viz_int["scn_width"]-1);
	mouseUp		= (y == 0);
	mouseDown	= (y == viz_int["scn_height"]-1);
	mouse_gnd_x =  scn2space(x-viz_int["scn_width"]/2)+view_x;
	mouse_gnd_y = -scn2space(y-viz_int["scn_height"]/2)+view_y;
}

// When mouse moves
void mouseMove(int x,int y){
	cursorUpdate(x,y);
	old_mouse_x	= x;
	old_mouse_y	= y;
}

// Mouse click trigger actions
void mouseButton(int b,int s,int x,int y){
	switch (b){
		case GLUT_LEFT_BUTTON:
			if(s == GLUT_DOWN){
				selX1 = mouse_gnd_x;
				selY1 = mouse_gnd_y;
				singleSelect();
				mouse_l = 1;
			}
			else{
				selX2 = mouse_gnd_x;
				selY2 = mouse_gnd_y;
				areaSelect();
				mouse_l = 0;
			}
			break;
		case GLUT_MIDDLE_BUTTON:
			if(s == GLUT_DOWN)
				mouse_m = 1;
			else
				mouse_m = 0;
			break;
		case GLUT_RIGHT_BUTTON:
			if(s == GLUT_DOWN){
				mouse_r = 1;
				set_goal(mouse_gnd_x, mouse_gnd_y);
			}
			else
				mouse_r = 0;
			break;
		default:
			break;
	}
}

// Sustained mouse click actions
void mouseAction(int x,int y){

	cursorUpdate(x,y);

	double delta_mouse_x = x-old_mouse_x;
	double delta_mouse_y = y-old_mouse_y;

	if(mouse_m){
		view_x +=  viz_float["mouse_m_speed"]*scn2space(delta_mouse_x);
		view_y += -viz_float["mouse_m_speed"]*scn2space(delta_mouse_y);
	}
	if(mouse_l){
	}
	if(mouse_r){
	}
	old_mouse_x = x;
	old_mouse_y = y;
}

// Set rally goal position
void set_goal(double x, double y){
	flag->x = x;
	flag->y = y;
}

// When keyboard pressed
void keyPressed(unsigned char key, int x, int y){
	if(key == KEY_EXIT)
		exit(0);
	for(unsigned char caps = 0; caps<=32; caps+=32){
		if(key+caps == viz_keys["left"].c_str()[0])
			keyLeft = 1;
		if(key+caps == viz_keys["right"].c_str()[0])
			keyRight = 1;
		if(key+caps == viz_keys["down"].c_str()[0])
			keyDown = 1;
		if(key+caps == viz_keys["up"].c_str()[0])
			keyUp = 1;
		if(key+caps == viz_bkeys["zoon_in"].c_str()[0])
			keyZoomIn = 1;
		if(key+caps == viz_bkeys["zoon_out"].c_str()[0])
			keyZoomOut = 1;
		if(key+caps == viz_bkeys["increase_depth"].c_str()[0])
			layers.switch_depth(+1);
		if(key+caps == viz_bkeys["decrease_depth"].c_str()[0])
			layers.switch_depth(-1);
		if(key+caps == viz_bkeys["switch_raster"].c_str()[0])
			layers.switch_raster();
		if(key+caps == viz_bkeys["switch_currents"].c_str()[0])
			layers.switch_currents();
		if(key+caps == viz_bkeys["reset_zoom"].c_str()[0])
			viz_float["scn_scale"] = viz_float["scn_scale_def"];
		if(key+caps == viz_bkeys["reset_view_pos"].c_str()[0]){
			view_x = origin_x;
			view_y = origin_y;
		}
		if(key+caps == viz_bkeys["lock_view_robot"].c_str()[0])
			lock_view_robot = !lock_view_robot;
	}
}

// When keyboard released
void keyReleased(unsigned char key, int x, int y){
	for(int caps = 0; caps<=32; caps+=32){
		if(key+caps == viz_keys["left"].c_str()[0])
			keyLeft = 0;
		if(key+caps == viz_keys["right"].c_str()[0])
			keyRight = 0;
		if(key+caps == viz_keys["down"].c_str()[0])
			keyDown = 0;
		if(key+caps == viz_keys["up"].c_str()[0])
			keyUp = 0;
		if(key+caps == viz_bkeys["zoon_in"].c_str()[0])
			keyZoomIn = 0;
		if(key+caps == viz_bkeys["zoon_out"].c_str()[0])
			keyZoomOut = 0;
	}
}

void updateValues(int n){

	ros::spinOnce();

	cursorUpdate(cursor->x,cursor->y);

	glutTimerFunc(1e3/sim_update_rate,updateValues,0);

	// Camera view will track specific robots
	if(lock_view_robot){
		for(auto const &r : robots)
			if(r.second->selected){
				view_x = r.second->x;
				view_y = r.second->y;
				break;
			}
	}

	// Moving the camera given the camera speed
	if(mouseLeft	|	keyLeft)	view_x -= viz_float["cam_speed"]*viz_float["scn_scale"];
	if(mouseRight	|	keyRight)	view_x += viz_float["cam_speed"]*viz_float["scn_scale"];
	if(mouseDown	|	keyDown)	view_y -= viz_float["cam_speed"]*viz_float["scn_scale"];
	if(mouseUp		|	keyUp)		view_y += viz_float["cam_speed"]*viz_float["scn_scale"];

	// Zoom
	if(keyZoomIn)
		viz_float["scn_scale"] /= viz_float["zoom_speed"];
	if(keyZoomOut)
		viz_float["scn_scale"] *= viz_float["zoom_speed"];

	// Updating camera projection parameters
	x_min = view_x - viz_float["scn_scale"]*viz_int["scn_width"]/2;
	x_max = view_x + viz_float["scn_scale"]*viz_int["scn_width"]/2;
	y_min = view_y - viz_float["scn_scale"]*viz_int["scn_height"]/2;
	y_max = view_y + viz_float["scn_scale"]*viz_int["scn_height"]/2;

	// String printed in the screen corner
	sprintf(statusBuffer, "Number of robots: %02d | %s | Depth Layer: %9.3f m", robots.size(), layers.get_description().c_str(), depths[layers.depth_layer]);

	sprintf(mouse_coord_buffer, "(%.3f,%.3f)", mouse_gnd_x, mouse_gnd_y);

	if(!ros::ok())
		exit(0);

}

// Scene rendering
void RenderScene(){

	// Clearing screen
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Space coordinates
	glPushMatrix();

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(x_min,x_max,y_min,y_max);
		glMatrixMode(GL_MODELVIEW);

		// Grid
		glLineWidth(2);
		glColor4f(0.0,0.5,0.0,0.75);
		glBegin(GL_LINES);
			for(float i = -180.00; i <= 180.0; i += 180.0){
				glVertex2f(i, -90.0);
				glVertex2f(i,  90.0);
			}
			for(float i = -90.00; i <= 90.0; i += 90.0){
				glVertex2f(-180.0,  i);
				glVertex2f( 180.0,  i);
			}
		glEnd();
		glLineWidth(1);
		glColor4f(0.0,0.5,0.0,0.75);
		glBegin(GL_LINES);
			for(float i = -180.00; i <= 180.0; i += 10.0){
				glVertex2f(i, -90.0);
				glVertex2f(i,  90.0);
			}
			for(float i = -90.00; i <= 90.0; i += 10.0){
				glVertex2f(-180.0,  i);
				glVertex2f( 180.0,  i);
			}
		glEnd();

		layers.render(viz_float["scn_scale"],x_min,x_max,y_min,y_max);

		// Drawing robots
		for(auto &r : robots)
			r.second->render();

		// Goal flag
		flag->render();

		// Mouse selection
		if(mouse_l){
			glLineWidth(2);
			glColor4f(0,0,1,0.75);
			glBegin(GL_LINE_LOOP);
				glVertex2f(selX1,selY1);
				glVertex2f(mouse_gnd_x,selY1);
				glVertex2f(mouse_gnd_x,mouse_gnd_y);
				glVertex2f(selX1,mouse_gnd_y);
			glEnd();
			glColor4f(0,0,1,0.25);
			glBegin(GL_QUADS);
				glVertex2f(selX1,selY1);
				glVertex2f(mouse_gnd_x,selY1);
				glVertex2f(mouse_gnd_x,mouse_gnd_y);
				glVertex2f(selX1,mouse_gnd_y);
			glEnd();
			glLineWidth(1);
		}

	glPopMatrix();

	// Screen coordinates
	glPushMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0,viz_int["scn_width"],viz_int["scn_height"],0);
		glMatrixMode(GL_MODELVIEW);
		glTranslatef(0,0,0);

		// Drawing buttons
		for(auto &b : buttons){
			b.second->render(0);
			char coordBuffer[BUFFER_SIZE];
			sprintf(coordBuffer, "%s (%c)", b.first.c_str(), viz_bkeys[b.first].c_str()[0]-32);
			coordBuffer[0] -= 32;
			for(int i = 0; i < BUFFER_SIZE; i++)
				if(coordBuffer[i] == '_'){
					coordBuffer[i] = ' ';
					coordBuffer[i+1] -= 32;
				}
			glRasterPos2i(b.second->x-0.5*b.second->w+viz_float["botton_padding"], b.second->y+0.5*viz_float["botton_height"]-12);
			glutBitmapString(GLUT_BITMAP_8_BY_13,(const unsigned char*)coordBuffer);
		}

		// Drawing mouse pointer
		cursor->render(0);

		// Cursor tag background shade
		glTranslatef(cursor->x+30,cursor->y+13,0);
		glColor4f(0,0,0,0.75);
		glPushMatrix();
			glScalef(140,24,1);
			glBegin(GL_QUADS);
				glVertex2f(0,0);
				glVertex2f(1,0);
				glVertex2f(1,1);
				glVertex2f(0,1);
			glEnd();
		glPopMatrix();

	glPopMatrix();

	glColor3f(1,1,1);
	glRasterPos2i(cursor->x+30,cursor->y+30);
	glutBitmapString(GLUT_BITMAP_8_BY_13,(const unsigned char*)mouse_coord_buffer);

	// Grid Ticks
	glColor4f(0.0,0.5,0.0,0.75);
	for(float i = -180.00; i <= 180.0; i += 10.0){
		glRasterPos2i(int(space2scn(i-view_x)+float(viz_int["scn_width"])/2)-16, int(space2scn(90.0+view_y)+float(viz_int["scn_height"])/2)+15);
		char coordBuffer[BUFFER_SIZE];
		sprintf(coordBuffer, "% 4d", int(i));
		glutBitmapString(GLUT_BITMAP_8_BY_13,(const unsigned char*)coordBuffer);
	}
	glColor4f(0.0,0.5,0.0,0.75);
	for(float i = -90.00; i <= 90.0; i += 10.0){
		glRasterPos2i(int(space2scn(180.0-view_x)+float(viz_int["scn_width"])/2)+12, int(space2scn(-i+view_y)+float(viz_int["scn_height"])/2)+4);
		char coordBuffer[BUFFER_SIZE];
		sprintf(coordBuffer, "% 3d", int(i));
		glutBitmapString(GLUT_BITMAP_8_BY_13,(const unsigned char*)coordBuffer);
	}

	// Status display
	glColor4f(0,0,0,0.75);
	glPushMatrix();
		glScalef(viz_int["scn_width"],24,1);
		glBegin(GL_QUADS);
			glVertex2f(0,0);
			glVertex2f(1,0);
			glVertex2f(1,1);
			glVertex2f(0,1);
		glEnd();
	glPopMatrix();
	glColor3f(1,1,1);
	glRasterPos2i(3,15);
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glutBitmapString(GLUT_BITMAP_8_BY_13,(const unsigned char*)statusBuffer);

	glutSwapBuffers();
}
