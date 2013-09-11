#include "glwindow_win32.h"
#include "lin_alg.h"
#include "physics/OBB.h"
#include "common.h"

mat4 view = mat4::identity();
Quaternion viewq;
mat4 projection;
vec4 view_position = vec4(0.0, 0.0, -8.0, 1.0);
vec4 cameraVel = vec4(0.0, 0.0, 0.0, 1.0);
vec4 camera_position;

OBB OBBa, OBBb;
Quaternion OBBaQ, OBBbQ;

GJKSession gjk_sess(OBBa, OBBb);

static LPPOINT cursorPos = new POINT;	/* holds cursor position */

static float terminating_margin = std::numeric_limits<float>::max();	// positive infinity? :P

#ifndef M_PI
#define M_PI 3.1415926535
#endif

float PROJ_FOV_RADIANS = (M_PI/4);
float PROJ_Z_FAR = 100.0;

bool use_wireframe = false;
int session_finished = 1;

float c_vel_fwd = 0, c_vel_side = 0;

bool EPA_vis_in_progress = false;

BEGIN_ALIGN16
struct EPA_history_t {
	vec4 new_p;
	my_aligned_vector<triangle_face, 16> active_faces;
	EPA_history_t(const std::vector<triangle_face*> &act_faces, const vec4 &p) {
		for (auto &p : act_faces) {
			active_faces.push_back(*p);	// deep copy :P
		}
		new_p = p;
	}
	EPA_history_t() {};
} END_ALIGN16;

my_aligned_vector<EPA_history_t, 16> EPA_history;

int current_history_index = 0;

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
	view_position -= vec4(0, 0.0, 0.0, 1.0);
	view_position += vec4(0.0, 0.0, c_vel_fwd, 1.0);
	viewq.normalize();
	view = viewq.toRotationMatrix();
}

void control()
{
	static const float fwd_modifier = 0.024;
	static const float side_modifier = 0.020;
	static const float mouse_modifier = 0.0009;

	static float dx, dy;
	dx = 0; dy = 0;
	
	if (mouse_locked) {
		GetCursorPos(cursorPos);	// these are screen coordinates, ie. absolute coordinates. it's fine though
		SetCursorPos(HALF_WINDOW_WIDTH, HALF_WINDOW_HEIGHT);
		dx = ((LONG)HALF_WINDOW_WIDTH - cursorPos->x);
		dy = -((LONG)HALF_WINDOW_HEIGHT - cursorPos->y);
	}
	
	if (WM_KEYDOWN_KEYS['W']) { c_vel_fwd += fwd_modifier; }
	if (WM_KEYDOWN_KEYS['S']) { c_vel_fwd -= fwd_modifier; }	
	c_vel_fwd *= 0.96;

	if (WM_KEYDOWN_KEYS['A']) { c_vel_side -= side_modifier; }
	if (WM_KEYDOWN_KEYS['D']) { c_vel_side += side_modifier; }
	c_vel_side *= 0.95;

	if (WM_KEYDOWN_KEYS['E']) {
		if (!EPA_vis_in_progress) {
			gjk_sess = GJKSession(OBBa, OBBb);
			if (gjk_sess.collision_test() == GJK_COLLISION) {			
				fprintf(stderr, "---------------------------------------------\n--------------------------------------------\n OBBs overlap -> recording step-wise EPA progress.\n");
				vec4 dummy;
				gjk_sess.EPA_penetration_stepwise_record(&dummy);
				current_history_index = 0;
				EPA_vis_in_progress = true;
			}
		}
		else {
			EPA_vis_in_progress = false;
			current_history_index = 0;
			EPA_history.clear();
		}
		WM_KEYDOWN_KEYS['E'] = FALSE;
	}

	if (WM_KEYDOWN_KEYS['N']) {
		if (EPA_vis_in_progress) {
			if (current_history_index < EPA_history.size()-1) { ++current_history_index; }
		}
		WM_KEYDOWN_KEYS['N'] = FALSE;
	}
	if (WM_KEYDOWN_KEYS['B']) {
		if (EPA_vis_in_progress) {
			if (current_history_index > 0) { --current_history_index; }
		}	
		WM_KEYDOWN_KEYS['B'] = FALSE;
	}

	if (WM_KEYDOWN_KEYS['P']) {
		use_wireframe = !use_wireframe;
		WM_KEYDOWN_KEYS['P'] = FALSE;
	}

	if (WM_KEYDOWN_KEYS['I']) {
		OBBaQ = OBBaQ*Quaternion::fromAxisAngle(0.0, 1.0, 0.0, 0.03);
	}
	if (WM_KEYDOWN_KEYS['O']) {
		OBBbQ = OBBbQ*Quaternion::fromAxisAngle(1.0, 0.0, 0.0, 0.03);
	}
	if (WM_KEYDOWN_KEYS['Y']) {
		OBBaQ = OBBaQ*Quaternion::fromAxisAngle(1.0, 0.0, 0.0, 0.03);
	}
	if (WM_KEYDOWN_KEYS['H']) {
		OBBbQ = OBBbQ*Quaternion::fromAxisAngle(0.0, 0.0, 1.0, 0.03);
	}

	if (WM_KEYDOWN_KEYS['L']) {
		OBBa.C += vec4(0.05, 0.0, 0.0, 0.0);
	}
	if (WM_KEYDOWN_KEYS['K']) {
		OBBa.C += vec4(-0.05, 0.0, 0.0, 0.0);
	}

	if (WM_KEYDOWN_KEYS['U']) {
		OBBa.C += vec4(0.0, 0.05, 0.0, 0.0);
	}
	if (WM_KEYDOWN_KEYS['J']) {
		OBBa.C += vec4(0.0, -0.05, 0.0, 0.0);
	}

	rotateview(mouse_modifier*dx, mouse_modifier*dy);

	camera_position = -view_position;
	camera_position.assign(V::z, camera_position(V::z)*(-1));

}

void handle_WM_KEYDOWN(WPARAM wParam) {		
	WM_KEYDOWN_KEYS[wParam]=TRUE;
	return;
}

void handle_WM_CHAR(WPARAM wParam) {
	WM_CHAR_KEYS[wParam] = TRUE;
	return;
}

static int initGL() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glLineWidth(5.0);
	resize_GL_scene(WINDOW_WIDTH, WINDOW_HEIGHT);	

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf((const GLfloat*) projection.rawData());
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);	

	glEnable(GL_COLOR_MATERIAL);
	
	float pos[] = {-2.0f, 2.0f, -3.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, pos);
	
	float dif[] = {1.0f, 1.0f, 1.0f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
	
	float amb[] = {0.05f, 0.05f, 0.05f, 1.0f};
	glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
	
	float spec[] = {1.0, 1.0, 1.0, 1.0 };
	glLightfv(GL_LIGHT0, GL_SPECULAR, spec);

	glPointSize(10);

	return 1;
}

void resize_GL_scene(GLsizei width, GLsizei height)	{
	height = MAXIMUM(height, 1);

	WINDOW_WIDTH = width;
	WINDOW_HEIGHT = height;

	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);	

	projection = mat4::proj_persp(PROJ_FOV_RADIANS, ((float)WINDOW_WIDTH/(float)WINDOW_HEIGHT), 4.0, PROJ_Z_FAR);

}

static convex_hull hull;

int GJKSession::EPA_penetration_stepwise_record(vec4 *outv) {

	hull.faces.clear();
	hull.points.clear();
	hull.active_faces.clear();
	EPA_history.clear();
	
	terminating_margin = std::numeric_limits<float>::max();	// positive infinity? :P

	hull.add_point(current_simplex.points[3]);
	hull.add_point(current_simplex.points[2]);
	hull.add_point(current_simplex.points[1]);
	hull.add_point(current_simplex.points[0]);
	
	hull.add_face(0, 1, 2);	// ABC
	hull.add_face(0, 2, 3);	// ACD
	hull.add_face(0, 3, 1);	// ADB
	hull.add_face(1, 3, 2);	// BDC

	// manually specifying initial adjacency information. See struct triangle_face definition @ OBB.h
	hull.faces[0].adjacents[0] = &hull.faces[2];
	hull.faces[0].adjacents[1] = &hull.faces[3];
	hull.faces[0].adjacents[2] = &hull.faces[1];

	hull.faces[1].adjacents[0] = &hull.faces[0];
	hull.faces[1].adjacents[1] = &hull.faces[3];
	hull.faces[1].adjacents[2] = &hull.faces[2];

	hull.faces[2].adjacents[0] = &hull.faces[1];
	hull.faces[2].adjacents[1] = &hull.faces[3];
	hull.faces[2].adjacents[2] = &hull.faces[0];

	hull.faces[3].adjacents[0] = &hull.faces[2];	
	hull.faces[3].adjacents[1] = &hull.faces[1];
	hull.faces[3].adjacents[2] = &hull.faces[0];


	hull.active_faces.push_back(&hull.faces[0]);
	hull.active_faces.push_back(&hull.faces[1]);
	hull.active_faces.push_back(&hull.faces[2]);
	hull.active_faces.push_back(&hull.faces[3]);


	// comparator to be used with std::min_element

	vec4 new_p;

	const int EPA_MAX_ITERATIONS = 32;

	for (int i = 0; i < EPA_MAX_ITERATIONS; ++i) {
		
		fprintf(stderr, "\n--------- iteration %d -------\n", i);

		std::sort(hull.active_faces.begin(), hull.active_faces.end(), 
			[](triangle_face const * a, triangle_face const * b) {
					return (a->orthog_distance_from_origin < b->orthog_distance_from_origin); 
			}
		);
		
		EPA_history.push_back(EPA_history_t(hull.active_faces, new_p));

		fprintf(stderr, "BEFORE step execution: face listing:\n");
		int counter = 0;
		for (auto &face : hull.active_faces) {
			fprintf(stderr, "face #%d, orthog_distance = %f, origin_proj = %d, ", counter, face->orthog_distance_from_origin, face->origin_proj_within_triangle);
			fprintf(stderr, "adj: %d, %d, %d\n", face->adjacents[0] - &hull.faces[0], face->adjacents[1] - &hull.faces[0], face->adjacents[2] - &hull.faces[0]);
			
			++counter;
		}

		auto best = hull.active_faces.begin();
	
		while (!(*best)->origin_proj_within_triangle) {
			++best;
			if (best == hull.active_faces.end()) {
				fprintf(stderr, "WARNING! none of the triangles contained the origin projection (this shouldn't be happening!)\n");
				best = hull.active_faces.begin(); 
				break; 
			}
		}
		
		triangle_face *best_tri = *best;
		vec4 search_direction = best_tri->normal;

		new_p = support(search_direction); // the search_direction vec4 (ie. best->normal) is normalized

		//fprintf(stderr, "closest face: orthog_distance = %f\nsearch direction: \n%s -> support point: %s\n", best->orthog_distance_from_origin, 
		//	print_vec4(search_direction).c_str(), print_vec4(new_p).c_str());

		float new_margin = dot3(search_direction, new_p);
		fprintf(stderr, "new_margin = %f, old margin = %f\n", new_margin, terminating_margin);

		if (fabs(new_margin - terminating_margin) < 0.000001) {
			//*outv = best->orthog_distance_from_origin*best->normal;	// we have converged to the penetration depth.
			fprintf(stderr, "EPA_pen: returning EPA_SUCCESS!\n\n");
			fprintf(stderr, "penetration depth = %s\n", print_vec4(best_tri->normal*best_tri->orthog_distance_from_origin).c_str());
			return EPA_SUCCESS;
		}
		terminating_margin = new_margin;	

		// else proceed to the convex hull computation part
		int obsolete_count = 0;
		auto &iter = hull.active_faces.begin();

		// remove faces visible from the new point (new_p)
		while (iter != hull.active_faces.end()) {
			triangle_face *face = (*iter);
			if (face->is_visible_from(new_p)) {
				// the triangle is obsolete
				face->obsolete = 1;
				iter = hull.active_faces.erase(iter);
				++obsolete_count;
			}
			else {
				face->obsolete = 0;
				++iter;
			}
		}
		
		fprintf(stderr, "EPA: marked %d faces as obsolete (visible from new_p)\n", obsolete_count);
		if (obsolete_count < 1) {
			// the point to be added was enclosed by the shape (or already was included in the convex hull)
			// this can easily happen with two discrete (polygonal) shapes
			fprintf(stderr, "WARNING: nothing to delete (the point-to-be-added was enclosed by the shape!)\n");
			//return EPA_INCONCLUSIVE;
			continue;
		}

		int new_index = hull.add_point(new_p);

		std::vector<triangle_face*> new_faces;
		
		// figure out the horizon line loop, add the new faces to the master list
		for (auto &face : hull.active_faces) { // obsolete faces have already been removed
			int edge_mask = 0;
			for (int i = 0; i < 3; ++i) {
				if (face->adjacents[i]->obsolete != 0) {
					int v_index0 = face->points[i%3] - &hull.points[0];
					int v_index1 = face->points[(i+1)%3] - &hull.points[0];
					auto p = hull.faces.push_back(triangle_face(hull.points, v_index1, v_index0, new_index));	// <- note, permutation (1, 0, n) to preserve winding
					
					p->adjacents[0] = face; // the shared two points comprise edge p0->p1 of the new face
					face->adjacents[i] = p;
					new_faces.push_back(p);
				}
			}
			
		}

		// figure out adjacency status for the new faces

		// this part could use some serious commentary.
		for (auto &face_a : new_faces) {
			// adjacents[0] is never NULL, see above loop. adjacents[2] are on the other hand covered by the loop (edges are resolved reciprocally for adj triangles)
				for (auto &face_b : new_faces) { // i'd still say roughly O(n) despite double looping
					if (face_a == face_b) { continue; }
					// adjacent new_faces will always share an edge involving the new_index point (always at index 2). This can easily be verified geometrically.
					// the edge we're checking against consists of pa_1 & pa_2 (there's a strong guarantee that pa_2 == pb_2, ie. the newly added point, so no need to check for that).
					// given ccw winding and the fact that the new faces all share the point new_p, stored at new_index (found at index 2 of all faces),
					// the only case where the triangles share an edge is where pa_1 (the point in face_a "counter-clockwise right before the shared point") 
					// equals pb_0 (the point "counter-clockwise right AFTER the shared point")
					else if (face_a->points[1] == face_b->points[0]) {
							face_a->adjacents[1] = face_b;
							face_b->adjacents[2] = face_a;
					}
				}

		}

		hull.active_faces.insert(hull.active_faces.end(), new_faces.begin(), new_faces.end());

		fprintf(stderr, "EPA: deleted %d faces, added %d new faces\n", obsolete_count, (int)new_faces.size());

		fprintf(stderr, "\nAFTER step execution: face listing:\n");
		counter = 0;
		for (auto &face : hull.active_faces) {
			fprintf(stderr, "face #%d, orthog_distance = %f, contains_origin_proj = %d, ", counter, face->orthog_distance_from_origin, face->origin_proj_within_triangle);
			fprintf(stderr, "adj: %d, %d, %d\n", face->adjacents[0] - &hull.faces[0], face->adjacents[1] - &hull.faces[0], face->adjacents[2] - &hull.faces[0]);
			++counter;
		}


	}
		
	return EPA_INCONCLUSIVE;
}

void EPA_draw_active_step() {

#define COLOR_GREEN do { glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE); glColor4f(0.3, 1.0, 0.2, 0.8); } while(0)
#define COLOR_YELLOW do { glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE); glColor4f(0.8, 0.9, 0.0, 0.95); } while (0)

	vec4 next_new_p = 
		(current_history_index < ((int)EPA_history.size() - 1)) ? 
		EPA_history[current_history_index + 1].new_p : EPA_history[current_history_index].new_p;
		
	//glLightfv(GL_LIGHT0, GL_POSITION, (const GLfloat*)next_new_p.rawData());

	EPA_history_t &cur_his = EPA_history[current_history_index];

	for (auto &face : cur_his.active_faces) {	
		glEnable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		bool is_visible = (face.is_visible_from(next_new_p));
		glEnable(GL_LIGHTING);
		glBegin(GL_TRIANGLES);	

		vec4 n = face.normal;
		for (int i = 0; i < 3; ++i) {
			vec4 p = face.points[i]->p;
			if (is_visible) {
				COLOR_YELLOW;
			} else {
				COLOR_GREEN;
			}
			glNormal3f(n(V::x), n(V::y), n(V::z));
			glVertex3f(p(V::x), p(V::y), p(V::z));
		}	
		glEnd();
		vec4 normal_beg = 0.33333333*(face.points[0]->p + face.points[1]->p + face.points[2]->p);
		vec4 normal_end = normal_beg + 0.5*n;
		// plot normal as well
		
		glDisable(GL_LIGHTING);
		glDisable(GL_BLEND);
		glBegin(GL_LINES);
		glColor3f(0.3, 0.3, 0.3);
		glVertex3f(normal_beg(0), normal_beg(1), normal_beg(2));
		glColor3f(1.0, 1.0, 1.0);
		glVertex3f(normal_end(0), normal_end(1), normal_end(2));
		glEnd();

		glBegin(GL_POINTS);
		glVertex3f(normal_beg(0), normal_beg(1), normal_beg(2));
		glEnd();

	}

	// draw next new point to be added	
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glBegin(GL_POINTS);
	glColor3f(1.0, 1.0, 0.05);
	glVertex3f(next_new_p(0), next_new_p(1), next_new_p(2));
	glEnd();

	if (current_history_index == EPA_history.size() - 1) {
		// draw penetration depth vector, ie. the "best" vector

		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glColor3f(0.0, 0.0, 1.0);
		vec4 pd = cur_his.active_faces[0].orthog_distance_from_origin*cur_his.active_faces[0].normal;
		glVertex3f(pd(0), pd(1), pd(2));
		glEnd();
	}

}

#define QUAD(arr, i1, i2, i3, i4) do {\
	glVertex3f(arr[i1](0), arr[i1](1), arr[i1](2));\
	glVertex3f(arr[i2](0), arr[i2](1), arr[i2](2));\
	glVertex3f(arr[i3](0), arr[i3](1), arr[i3](2));\
	glVertex3f(arr[i4](0), arr[i4](1), arr[i4](2));\
	} while (0)

void drawBoxes() {

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

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


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{	
#define ENABLE_CONSOLE
#ifdef ENABLE_CONSOLE
	if(AllocConsole()) {
	 // for debugging those early-program fatal erreurz. this will screw up our framerate though.
		FILE *dummy;
		freopen_s(&dummy, "CONOUT$", "wt", stderr);

		SetConsoleTitle("debug output");
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED);
	} 
#endif
	if (!CreateGLWindow("GJK_EPA_visualization", WINDOW_WIDTH, WINDOW_HEIGHT, 32, FALSE, hInstance, nCmdShow)) { return 1; }
	if (!initGL()) {
		return 1; 
	}
	
	std::string cpustr(checkCPUCapabilities());
	if (cpustr.find("ERROR") != std::string::npos) { MessageBox(NULL, cpustr.c_str(), "Fatal error.", MB_OK); return -1; }
	
	fprintf(stderr, "%s\n", cpustr.c_str());
	
	static float running = 0;
	MSG msg;
	
	OBBa = OBB(vec4(2.0, 0.8, 1.6, 0.0));
	OBBa.C = vec4(0.8, 0.8, 0.0, 0.0);
	OBBb = OBB(vec4(1.23, 2.3, 1.5, 0.0));
	OBBb.C = vec4(0.0, 0.0, 0.0, 0.0);


	_timer fps_timer;
	fps_timer.begin();

	while(main_loop_running())
	{
		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) > 0)
		{
			if(msg.message == WM_QUIT)
			{
				stop_main_loop();
			}
			else {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		
		if (WM_KEYDOWN_KEYS[VK_ESCAPE])
		{
			ShowCursor(mouse_locked);
			mouse_locked = !mouse_locked;

			if (!mouse_locked) { 
				set_cursor_relative_pos(HALF_WINDOW_WIDTH, HALF_WINDOW_HEIGHT);
			}
			WM_KEYDOWN_KEYS[VK_ESCAPE] = FALSE;
		}

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		control();
		update_c_pos();
		
		glPolygonMode(GL_FRONT_AND_BACK, use_wireframe ? GL_LINE : GL_FILL);
		
		glBegin(GL_POINTS);	
		glColor3f(0.0, 1.0, 0.0);
		glVertex3f(0.0, 0.0, 0.0);
		glEnd();

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf((const GLfloat*)mat4::translate(view_position).rawData());
		glMultMatrixf((const GLfloat*)view.rawData());

		if (EPA_vis_in_progress) {
			EPA_draw_active_step();
		}
		else {
			drawBoxes();
		}

		double wait = fps_timer.get_ms() - 16.666;
		if (wait > 4) {
			Sleep(wait - 2);
		}
		window_swapbuffers(); // calls wglSwapBuffers on the current context. if vsync is enabled, the current thread is blocked
		fps_timer.begin();
	}


	return (msg.wParam);
}