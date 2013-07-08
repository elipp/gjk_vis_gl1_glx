#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>

#include <cassert>
#include <signal.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>

#include "gjkvis.h"
#include "lin_alg.h"
#include "physics/OBB.h"

#include "window_crap.h"

const mat4 indicator_proj = mat4::proj_ortho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -1.0, 1.0);

extern void draw_simplex(const std::pair<int, simplex> &sp);

static int coll_active = 0;

static GLint PMODE = GL_FILL;


extern std::vector<std::pair<int, simplex> > simplex_history;
static std::vector<std::pair<int, simplex> >::iterator hist_iter;

bool mouseLocked = true;

mat4 Projection;
mat4 View;
Quaternion viewq;

vec4 view_position;

static OBB OBBa, OBBb;
static Quaternion OBBaQ, OBBbQ;

static GJKSession sess(OBBa, OBBb);

bool plot_normals = true;

#ifndef M_PI
#define M_PI 3.1415926535
#endif


static int current_retval = 0;

static float c_vel_side = 0, c_vel_fwd = 0;

static void showCursor(int arg) {
	if (arg) {
		XDefineCursor(dpy, win, visibleCursor);
	}
	else {
		XDefineCursor(dpy, win, invisibleCursor);
	}
}

void rotateview(float modx, float mody) {
	static float qx = 0;
	static float qy = 0;
	qx += modx;
	qy -= mody;
	Quaternion xq = Quaternion::fromAxisAngle(1.0, 0.0, 0.0, qy);
	Quaternion yq = Quaternion::fromAxisAngle(0.0, 1.0, 0.0, qx);
	viewq = yq*xq;
}

void update_c_pos() {
	view_position -= vec4(c_vel_side, 0.0, 0.0, 1.0).applyQuatRotation(viewq);
	view_position += vec4(0.0, 0.0, c_vel_fwd, 1.0).applyQuatRotation(viewq);
	viewq.normalize();
	View = viewq.toRotationMatrix();
	View = View*mat4::translate(view_position);
}

void control() {
	static const float fwd_modifier = 0.008;
	static const float side_modifier = 0.005;
	static const float mouse_modifier = 0.001;

	static Window root, child;
	static int rootX, rootY;

	if (mouseLocked) {
		unsigned int buttonmask;
		XQueryPointer(dpy, win, &root, &child, &rootX, &rootY, &cursorPos.x, &cursorPos.y, &buttonmask);
		float dx = (HALF_WINDOW_WIDTH - cursorPos.x);
		float dy = -(HALF_WINDOW_HEIGHT - cursorPos.y);

		XWarpPointer(dpy, None, win, 0, 0, 0, 0, HALF_WINDOW_WIDTH, HALF_WINDOW_HEIGHT);
		XSync(dpy, False);

		if (keys[KEY_W]) {
			c_vel_fwd += fwd_modifier;
			//keys['W'] = FALSE;
		} 
		if (keys[KEY_S]) {
			c_vel_fwd -= fwd_modifier;
			//keys['S'] = FALSE;
		}
		c_vel_fwd *= 0.97;

		if (keys[KEY_A]) {
			c_vel_side -= side_modifier;
			//keys['A'] = FALSE;
		} 
		if (keys[KEY_D]) {
			c_vel_side += side_modifier;
			//keys['D'] = FALSE;
		} 
		c_vel_side *= 0.95;


		if (keys[KEY_N]) {
			OBBaQ = OBBaQ*Quaternion::fromAxisAngle(0.0, 1.0, 0.0, 0.03);
		}
		if (keys[KEY_M]) {
			OBBbQ = OBBbQ*Quaternion::fromAxisAngle(1.0, 0.0, 0.0, 0.03);
		}

		if (keys[KEY_J]) {
			OBBa.C += vec4(0.05, 0.0, 0.0, 0.0);
		}
		if (keys[KEY_H]) {
			OBBa.C += vec4(-0.05, 0.0, 0.0, 0.0);
		}

		if (keys[KEY_G]) {
			keys[KEY_G] = false;
			if (coll_active) {
				auto tmp = hist_iter;
				++hist_iter;
				if (hist_iter == simplex_history.end()) {
					hist_iter = tmp;
				}
			}
		}
		
		if (keys[KEY_F]) {
			keys[KEY_F] = false;
			if (coll_active) {
				if (hist_iter != simplex_history.begin()) {
					--hist_iter;
				}
			}
		}


		if (keys[KEY_T]) {
			keys[KEY_T] = false;
			if (!coll_active) {
				sess = GJKSession(OBBa, OBBb);
				current_retval = sess.collision_test();
				hist_iter = simplex_history.begin();
				coll_active = 1;
			}
			else {
				coll_active = 0;
			}
		}
		
		rotateview(mouse_modifier*dx, mouse_modifier*dy);
	}

}

#define QUAD(arr, i1, i2, i3, i4) do {\
	glVertex3f(arr[i1](0), arr[i1](1), arr[i1](2));\
	glVertex3f(arr[i2](0), arr[i2](1), arr[i2](2));\
	glVertex3f(arr[i3](0), arr[i3](1), arr[i3](2));\
	glVertex3f(arr[i4](0), arr[i4](1), arr[i4](2));\
	} while (0)

void drawBoxes() {

	vec4 VA[8];
	OBBa.rotate(OBBaQ);
	OBBa.compute_box_vertices(VA);

	float_arr_vec4 VA_arr[8] = {
		VA[0], VA[1], VA[2], VA[3],
		VA[4], VA[5], VA[6], VA[7]
	};

	vec4 VB[8];
	OBBb.rotate(OBBbQ);
	OBBb.compute_box_vertices(VB);

	float_arr_vec4 VB_arr[8] = {
		VB[0], VB[1], VB[2], VB[3],
		VB[4], VB[5], VB[6], VB[7]
	};

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_QUADS);
	glColor3f(1.0, 0.0, 0.0);
	QUAD(VA_arr, 0, 1, 2, 3);
	QUAD(VA_arr, 0, 1, 5, 4);
	QUAD(VA_arr, 0, 3, 7, 4);
	QUAD(VA_arr, 4, 5, 6, 7);
	QUAD(VA_arr, 1, 2, 6, 5);
	QUAD(VA_arr, 2, 3, 7, 6);

	glColor3f(0.0, 0.0, 1.0);
	QUAD(VB_arr, 0, 1, 2, 3);
	QUAD(VB_arr, 0, 1, 5, 4);
	QUAD(VB_arr, 0, 3, 7, 4);
	QUAD(VB_arr, 4, 5, 6, 7);
	QUAD(VB_arr, 1, 2, 6, 5);
	QUAD(VB_arr, 2, 3, 7, 6);
	glEnd();
	
}

int initGL(void) {

	GLenum err = glewInit();
	if (GLEW_OK != err) {
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	}

	glClearColor(0.0, 0.0, 0.0, 1.0);
	glEnable(GL_DEPTH_TEST);

	glPointSize(10);

	printf("OpenGL version: %s\n", glGetString(GL_VERSION));
	printf("GLSL version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

	view_position = vec4(0.0, 0.0, -10.0, 1.0);
	View = view_position.toTranslationMatrix();

	Projection = mat4::proj_persp(M_PI/8.0, (float)WINDOW_WIDTH/(float)WINDOW_HEIGHT, 3.0, 400.0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf((const GLfloat*)Projection.rawData());

	OBBa = OBB(vec4(1.0, 1.0, 1.0, 0.0));
	OBBb = OBB(vec4(1.0, 1.0, 1.0, 0.0));

	OBBa.C = vec4(-1.0, 0.0, 0.0, 1.0);
	OBBb.C = vec4(1.0, 0.0, 0.0, 1.0);

	return 1;
}

static void draw_returnvalue_indicator() {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixf((const GLfloat*)indicator_proj.rawData());
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glBegin(GL_POINTS);
	if (current_retval == 1) {
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(15, WINDOW_HEIGHT-15, 0.0);
	}
	else if (current_retval == 0) {
		glColor3f(0.0, 1.0, 0.0);
		glVertex3f(15, WINDOW_HEIGHT-15, 0.0);
	}
	glEnd();
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

}

int main(int argc, char* argv[]) {

	if (!createWindow()) { fprintf(stderr, "couldn't create window.\n"); exit(1); }
	if (!initGL()) { fprintf(stderr, "Failed to initialize OpenGL.\n"); exit(1); }

	signal(SIGINT, signal_handler);

	int c;
	bool esc = false;

	while (!done) {
		while(XPending(dpy)) {
			XNextEvent(dpy, &xev);
			if (!handle_event(xev)) {
				done = true;
			}
		}
		control();
		update_c_pos();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf((const GLfloat*)View.rawData());
	
		if (coll_active) {
			draw_simplex(*hist_iter);
			draw_returnvalue_indicator();
		} else {
			drawBoxes();
		}

		static float running = 0;
		running += 0.05;

		glXSwapBuffers(dpy, win); 

	}

	cleanup();

	return 0;

}

