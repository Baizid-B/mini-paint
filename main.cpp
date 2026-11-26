#include <GL/freeglut.h>
#include <vector>
#include <cstdlib>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

const int W = 900, H = 600;
struct Color { unsigned char r, g, b; };

std::vector<Color> fb(W * H, Color{255, 255, 255});
Color cur{0, 0, 0};
int brush = 3, lx = 0, ly = 0;
bool drawing = false;

void setPixel(int x, int y, Color c) {
    if (x < 0 || y < 0 || x >= W || y >= H) return;
    fb[y * W + x] = c;
}

void plotBrush(int x, int y) {
    int r = brush / 2;
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            setPixel(x + dx, y + dy, cur);
}

// Bresenham's line algorithm
void bresenham(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        plotBrush(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    glRasterPos2f(-1, -1);
    glDrawPixels(W, H, GL_RGB, GL_UNSIGNED_BYTE, fb.data());
    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y) {
    if (button != GLUT_LEFT_BUTTON) return;
    y = H - 1 - y;
    if (state == GLUT_DOWN) { drawing = true; lx = x; ly = y; plotBrush(x, y); }
    else drawing = false;
    glutPostRedisplay();
}

void motion(int x, int y) {
    if (!drawing) return;
    y = H - 1 - y;
    bresenham(lx, ly, x, y);
    lx = x; ly = y;
    glutPostRedisplay();
}

void keyboard(unsigned char k, int, int) {
    switch (k) {
        case 'c': std::fill(fb.begin(), fb.end(), Color{255, 255, 255}); break;
        case 'e': cur = Color{255, 255, 255}; break;      // eraser
        case '1': cur = Color{0, 0, 0}; break;
        case '2': cur = Color{255, 0, 0}; break;
        case '3': cur = Color{0, 0, 255}; break;
        case '+': brush++; break;
        case '-': if (brush > 1) brush--; break;
        case 's':
            stbi_flip_vertically_on_write(1);
            stbi_write_png("drawing.png", W, H, 3, fb.data(), W * 3);
            break;
        case 27: exit(0);
    }
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(W, H);
    glutCreateWindow("Mini Paint");
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    glutMainLoop();
}