#ifndef WINDOW_CRAP_H
#define WINDOW_CRAP_H

bool active = true;
bool fullscreen = false;
bool keys[256] = { false };

static struct mousePos {
	int x, y;
} cursorPos;

#define HALF_WINDOW_WIDTH WINDOW_WIDTH/2.0
#define HALF_WINDOW_HEIGHT WINDOW_HEIGHT/2.0

bool done = false;

#define KEY_W 0x19
#define KEY_A 0x26
#define KEY_S 0x27
#define KEY_D 0x28
#define KEY_N 0x39
#define KEY_M 0x3A
#define KEY_P 0x21
#define KEY_F 0x29
#define KEY_G 0x2A
#define KEY_H 0x2B
#define KEY_J 0x2C
#define KEY_K 0x2D
#define KEY_L 0x2E
#define KEY_V 0x37
#define KEY_B 0x38
#define KEY_T 0x1C

#define KEY_ESC 0x09

#ifndef M_PI
#define M_PI 3.14159265359
#endif
#define PI_PER_180 M_PI/180.0
#define RADIANS(angle_in_degrees) (angle_in_degrees)*PI_PER_180

Display                 *dpy;
Window                  root;
GLint                   att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
XVisualInfo             *vi;
Colormap                cmap;
XSetWindowAttributes    swa;
Window                  win;
GLXContext              glc;
XWindowAttributes       gwa;
XEvent                  xev;

Pixmap bitmapNoData;
Cursor invisibleCursor;
Cursor visibleCursor;
Cursor currentCursor;

int createWindow() {
	dpy = XOpenDisplay(NULL);
	if(dpy == NULL) {
		printf("createWindow: cannot connect to X server\n");
		exit(0);
	}
	root = DefaultRootWindow(dpy);

	vi = glXChooseVisual(dpy, 0, att);

	if(vi == NULL) {
		printf("no appropriate visual found\n");
		exit(0);
	}
	else {
		printf("glXChooseVisual: visual %p selected\n", (void *)vi->visualid); /* %p creates hexadecimal output like in glxinfo */
	}
	cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);

	swa.colormap = cmap;
	swa.event_mask = ExposureMask | KeyPressMask;

	win = XCreateWindow(dpy, root, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, vi->depth, InputOutput, vi->visual, CWColormap | CWEventMask, &swa);

	XSelectInput(dpy, win, ButtonPressMask|StructureNotifyMask|KeyPressMask|KeyReleaseMask);
	XMapWindow(dpy, win);
	XStoreName(dpy, win, "aglio-olio, biatch!");

	glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
	glXMakeCurrent(dpy, win, glc);
	glClear(GL_COLOR_BUFFER_BIT);
	glXSwapBuffers(dpy, win);

	return 1;
}



GLvoid ResizeGLScene(GLsizei width, GLsizei height)		// Resize And Initialize The GL Window
{
	if (height == 0) {										// Prevent A Divide By Zero By 
		height = 1;										// Making Height Equal One
	}
	glViewport(0, 0, width, height);						// Reset The Current Viewport
	// Calculate The Aspect Ratio Of The Window
	//gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
}

inline int handle_event(XEvent ev) {
	switch(ev.type){
		case KeyPress:
			if (ev.xkey.keycode == KEY_ESC) { 
//				mouseLocked = !mouseLocked;
////				showCursor((int)!mouseLocked);
				done = true;
			}
			keys[ev.xkey.keycode] = true;
			//fprintf(stderr, "%x\n", ev.xkey.keycode);
			break;
		case KeyRelease:
			keys[ev.xkey.keycode] = false;
			break;
		case ButtonPress:
			break;
	}
	return 1;
}


void cleanup() {

	glXMakeCurrent(dpy, None, NULL);
	glXDestroyContext(dpy, glc);
	XDestroyWindow(dpy, win);
	XCloseDisplay(dpy);

//	restoreCursor();

}


void signal_handler(int sig) {
	exit(sig);
}
void swap_buffers() {
	glXSwapBuffers(dpy, win);
}

/*static int initCursor() {
	XColor black;
	static char noData[] = { 0,0,0,0,0,0,0,0 };
	black.red = black.green = black.blue = 0;

	visibleCursor=XCreateFontCursor(dpy, XC_left_ptr);

	bitmapNoData = XCreateBitmapFromData(dpy, win, noData, 8, 8);
	invisibleCursor = XCreatePixmapCursor(dpy, bitmapNoData, bitmapNoData,
			&black, &black, 0, 0);
	XDefineCursor(dpy, win, invisibleCursor);
	return 1;
}

static void restoreCursor() {
	Cursor cursor;
	XFreeCursor(dpy, invisibleCursor);
	XFreePixmap(dpy, bitmapNoData);	// can perhaps be moved to initCursor
	XUndefineCursor(dpy, win);

	cursor=XCreateFontCursor(dpy, XC_left_ptr);
	XDefineCursor(dpy, win, cursor);
}
*/



#endif
