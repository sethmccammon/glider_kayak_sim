#ifndef INCLUDES_H
#define INCLUDES_H
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <X11/Xlib.h>
#include <cstdlib>
#include <vector>
#include <regex>
#include <string>
#include <tuple>
#include <map>

using namespace std;

// Sim
#define NUM_ROBOTS		10
#define PI				3.14159265359

// File Parameters
#define BUFFER_SIZE		256

// Video Parameters
#define WINDOW_TITLE	"Video"
#define FULL_SCREEN		0
#define SIM_STEP_TIME	1.000

// #define SIM_STEP_TIME	0.001

// Camera Parameters
#define CAM_SPEED		10.0
#define SCN_SCALE		0.01

// Keyboard Commands
#define EXIT			27
#define KEY_UP			'w'
#define KEY_DOWN		's'
#define KEY_LEFT		'a'
#define KEY_RIGHT		'd'
#define KEY_ZOON_IN		'e'
#define KEY_ZOON_OUT	'q'
#define RESET_ZOOM		'z'
#define RESET_VIEW_POS	'h'
#define LOCK_VIEW_ROBOT	'l'

#endif
