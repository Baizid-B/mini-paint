// ============================================================================
//  Mini Paint  -  Computer Graphics Lab Project
//  World University of Bangladesh (WUB), Dept. of CSE
//
//  All drawing is done in a software pixel buffer using the algorithms taught
//  in class. OpenGL/FreeGLUT is used ONLY to put the buffer on screen and to
//  draw the user interface.
//
//  Algorithms implemented:
//    - Bresenham line                (line / pencil / brush / rectangle)
//    - Midpoint circle               (circle tool)
//    - Midpoint ellipse (2 regions)  (ellipse tool)
//    - Scan-line span filling        (filled rectangle / circle / ellipse)
//    - Scan-line stack flood fill    (fill tool)
//    - Cohen-Sutherland line clipping
//    - 2D transformations with 3x3 homogeneous matrices (select tool)
//    - Canvas state stack            (undo / redo)
//    - RGB colour model              (palette + RGB sliders)
//
//  Extra: insert image (with transparency), copy/cut/paste, flip, filters,
//  colour picker, airbrush, cubic Bezier curve, scan-line polygon fill.
//
//  Build (Windows / MinGW):
//    g++ -std=c++17 -O2 main.cpp -o minipaint -lfreeglut -lopengl32 -lglu32 -lcomdlg32
//  Build (Linux):
//    g++ -std=c++17 -O2 main.cpp -o minipaint -lglut -lGL
//
//  Needs stb_image.h and stb_image_write.h next to this file.
// ============================================================================

#define NOMINMAX
#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#endif

#include <GL/freeglut.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ============================================================================
//  1. Configuration and basic types
// ============================================================================
namespace cfg {
constexpr int CANVAS_W = 800;
constexpr int CANVAS_H = 520;
constexpr int MARGIN = 10;
constexpr int TB_W = 340;  // toolbar width (two panels)
constexpr int CANVAS_X = TB_W + MARGIN;
constexpr int CANVAS_Y = MARGIN;
constexpr int PAL_X = CANVAS_X;
constexpr int PAL_Y = CANVAS_Y + CANVAS_H + MARGIN;
constexpr int PAL_W = CANVAS_W;
constexpr int PAL_H = 84;
constexpr int STATUS_H = 24;
constexpr int WIN_W = CANVAS_X + CANVAS_W + MARGIN;
constexpr int WIN_H = PAL_Y + PAL_H + STATUS_H;
constexpr size_t UNDO_LIMIT = 40;
constexpr int MAX_BRUSH = 40;
}  // namespace cfg

struct Color {
    uint8_t r = 0, g = 0, b = 0;
};
inline bool operator==(const Color& a, const Color& b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
inline bool operator!=(const Color& a, const Color& b) { return !(a == b); }

static const Color kWhite{255, 255, 255};

class Canvas {
public:
    std::vector<Color> px;
    Canvas() : px(static_cast<size_t>(cfg::CANVAS_W) * cfg::CANVAS_H, kWhite) {}
    static bool inside(int x, int y) { return x >= 0 && y >= 0 && x < cfg::CANVAS_W && y < cfg::CANVAS_H; }
    void set(int x, int y, Color c) {
        if (inside(x, y)) px[static_cast<size_t>(y) * cfg::CANVAS_W + x] = c;
    }
    Color get(int x, int y) const { return px[static_cast<size_t>(y) * cfg::CANVAS_W + x]; }
    void clear(Color c = kWhite) { std::fill(px.begin(), px.end(), c); }
};

// ============================================================================
//  2. Drawing engine  (all algorithms written by hand)
// ============================================================================
namespace gfx {

// ---- Brush stamp (filled disc; size 1 = single pixel) ----------------------
inline void stamp(Canvas& cv, int cx, int cy, int size, Color c) {
    if (size <= 1) {
        cv.set(cx, cy, c);
        return;
    }
    const double r = size / 2.0, rr = r * r;
    const int ri = static_cast<int>(std::ceil(r));
    for (int dy = -ri; dy <= ri; ++dy)
        for (int dx = -ri; dx <= ri; ++dx)
            if (dx * dx + dy * dy <= rr) cv.set(cx + dx, cy + dy, c);
}

// ---- Horizontal span (used by all scan-line fills) -------------------------
inline void hspan(Canvas& cv, int xa, int xb, int y, Color c) {
    if (y < 0 || y >= cfg::CANVAS_H) return;
    if (xa > xb) std::swap(xa, xb);
    xa = std::max(xa, 0);
    xb = std::min(xb, cfg::CANVAS_W - 1);
    for (int x = xa; x <= xb; ++x) cv.px[static_cast<size_t>(y) * cfg::CANVAS_W + x] = c;
}

// ---- Cohen-Sutherland line clipping ----------------------------------------
namespace detail {
inline int outCode(double x, double y, double xmin, double ymin, double xmax, double ymax) {
    int code = 0;
    if (x < xmin) code |= 1;
    else if (x > xmax) code |= 2;
    if (y < ymin) code |= 4;
    else if (y > ymax) code |= 8;
    return code;
}
}  // namespace detail

// Clips the segment to the rectangle. Returns false if it is fully outside.
inline bool clipLine(int& x0, int& y0, int& x1, int& y1, int xmin, int ymin, int xmax, int ymax) {
    double ax = x0, ay = y0, bx = x1, by = y1;
    int c0 = detail::outCode(ax, ay, xmin, ymin, xmax, ymax);
    int c1 = detail::outCode(bx, by, xmin, ymin, xmax, ymax);
    while (true) {
        if (!(c0 | c1)) {
            x0 = static_cast<int>(std::lround(ax));
            y0 = static_cast<int>(std::lround(ay));
            x1 = static_cast<int>(std::lround(bx));
            y1 = static_cast<int>(std::lround(by));
            return true;
        }
        if (c0 & c1) return false;
        const int co = c0 ? c0 : c1;
        double x = 0, y = 0;
        if (co & 8) {
            x = ax + (bx - ax) * (ymax - ay) / (by - ay);
            y = ymax;
        } else if (co & 4) {
            x = ax + (bx - ax) * (ymin - ay) / (by - ay);
            y = ymin;
        } else if (co & 2) {
            y = ay + (by - ay) * (xmax - ax) / (bx - ax);
            x = xmax;
        } else {
            y = ay + (by - ay) * (xmin - ax) / (bx - ax);
            x = xmin;
        }
        if (co == c0) {
            ax = x;
            ay = y;
            c0 = detail::outCode(ax, ay, xmin, ymin, xmax, ymax);
        } else {
            bx = x;
            by = y;
            c1 = detail::outCode(bx, by, xmin, ymin, xmax, ymax);
        }
    }
}

// ---- Bresenham line (integer only, all octants) ----------------------------
inline void drawLine(Canvas& cv, int x0, int y0, int x1, int y1, int size, Color c) {
    const int m = size / 2 + 1;  // margin so thick strokes near the border still show
    if (!clipLine(x0, y0, x1, y1, -m, -m, cfg::CANVAS_W - 1 + m, cfg::CANVAS_H - 1 + m)) return;
    const int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        stamp(cv, x0, y0, size, c);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// ---- Rectangle (scan-line fill + Bresenham outline) ------------------------
inline void drawRect(Canvas& cv, int x0, int y0, int x1, int y1, int size, Color outline, Color fill, bool filled) {
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    if (filled)
        for (int y = y0; y <= y1; ++y) hspan(cv, x0, x1, y, fill);
    drawLine(cv, x0, y0, x1, y0, size, outline);
    drawLine(cv, x1, y0, x1, y1, size, outline);
    drawLine(cv, x1, y1, x0, y1, size, outline);
    drawLine(cv, x0, y1, x0, y0, size, outline);
}

// ---- Midpoint circle: calls plot(x, y) for one octant ----------------------
template <class F>
inline void midpointCircle(int r, F plot) {
    int x = 0, y = r, d = 1 - r;
    plot(x, y);
    while (x < y) {
        ++x;
        if (d < 0) {
            d += 2 * x + 1;
        } else {
            --y;
            d += 2 * (x - y) + 1;
        }
        plot(x, y);
    }
}

inline void drawCircle(Canvas& cv, int cx, int cy, int r, int size, Color outline, Color fill, bool filled) {
    if (r <= 0) {
        stamp(cv, cx, cy, size, outline);
        return;
    }
    if (filled)
        midpointCircle(r, [&](int x, int y) {
            hspan(cv, cx - x, cx + x, cy + y, fill);
            hspan(cv, cx - x, cx + x, cy - y, fill);
            hspan(cv, cx - y, cx + y, cy + x, fill);
            hspan(cv, cx - y, cx + y, cy - x, fill);
        });
    midpointCircle(r, [&](int x, int y) {
        stamp(cv, cx + x, cy + y, size, outline);
        stamp(cv, cx - x, cy + y, size, outline);
        stamp(cv, cx + x, cy - y, size, outline);
        stamp(cv, cx - x, cy - y, size, outline);
        stamp(cv, cx + y, cy + x, size, outline);
        stamp(cv, cx - y, cy + x, size, outline);
        stamp(cv, cx + y, cy - x, size, outline);
        stamp(cv, cx - y, cy - x, size, outline);
    });
}

// ---- Midpoint ellipse (region 1 + region 2): plot(x, y) for one quadrant ---
template <class F>
inline void midpointEllipse(int rx, int ry, F plot) {
    const long long rx2 = 1LL * rx * rx, ry2 = 1LL * ry * ry;
    long long x = 0, y = ry;
    long long px = 0, py = 2 * rx2 * y;
    double p = ry2 - rx2 * ry + 0.25 * rx2;
    plot(static_cast<int>(x), static_cast<int>(y));
    while (px < py) {  // region 1: slope magnitude < 1
        ++x;
        px += 2 * ry2;
        if (p < 0) {
            p += ry2 + px;
        } else {
            --y;
            py -= 2 * rx2;
            p += ry2 + px - py;
        }
        plot(static_cast<int>(x), static_cast<int>(y));
    }
    p = ry2 * (x + 0.5) * (x + 0.5) + rx2 * static_cast<double>(y - 1) * (y - 1) - static_cast<double>(rx2) * ry2;
    while (y > 0) {  // region 2: slope magnitude >= 1
        --y;
        py -= 2 * rx2;
        if (p > 0) {
            p += rx2 - py;
        } else {
            ++x;
            px += 2 * ry2;
            p += rx2 - py + px;
        }
        plot(static_cast<int>(x), static_cast<int>(y));
    }
}

inline void drawEllipse(Canvas& cv, int cx, int cy, int rx, int ry, int size, Color outline, Color fill,
                        bool filled) {
    if (rx == 0 || ry == 0) {  // degenerate: a line or a point
        drawLine(cv, cx - rx, cy - ry, cx + rx, cy + ry, size, outline);
        return;
    }
    if (filled)
        midpointEllipse(rx, ry, [&](int x, int y) {
            hspan(cv, cx - x, cx + x, cy + y, fill);
            hspan(cv, cx - x, cx + x, cy - y, fill);
        });
    midpointEllipse(rx, ry, [&](int x, int y) {
        stamp(cv, cx + x, cy + y, size, outline);
        stamp(cv, cx - x, cy + y, size, outline);
        stamp(cv, cx + x, cy - y, size, outline);
        stamp(cv, cx - x, cy - y, size, outline);
    });
}

// ---- Scan-line stack-based flood fill (no recursion, no stack overflow) ----
inline bool floodFill(Canvas& cv, int sx, int sy, Color fill) {
    if (!Canvas::inside(sx, sy)) return false;
    const Color target = cv.get(sx, sy);
    if (target == fill) return false;
    const int W = cfg::CANVAS_W, H = cfg::CANVAS_H;
    std::vector<std::pair<int, int>> stack;
    stack.emplace_back(sx, sy);
    while (!stack.empty()) {
        auto [x, y] = stack.back();
        stack.pop_back();
        if (cv.get(x, y) != target) continue;
        int l = x, r = x;
        while (l > 0 && cv.get(l - 1, y) == target) --l;
        while (r < W - 1 && cv.get(r + 1, y) == target) ++r;
        for (int i = l; i <= r; ++i) cv.px[static_cast<size_t>(y) * W + i] = fill;
        for (int ny : {y - 1, y + 1}) {
            if (ny < 0 || ny >= H) continue;
            bool run = false;
            for (int i = l; i <= r; ++i) {
                if (cv.get(i, ny) == target) {
                    if (!run) stack.emplace_back(i, ny);
                    run = true;
                } else {
                    run = false;
                }
            }
        }
    }
    return true;
}


// ---- Scan-line polygon fill (even-odd rule) --------------------------------
inline void fillPolygon(Canvas& cv, const std::vector<std::pair<int, int>>& p, Color c) {
    if (p.size() < 3) return;
    int ymin = p[0].second, ymax = p[0].second;
    for (const auto& v : p) {
        ymin = std::min(ymin, v.second);
        ymax = std::max(ymax, v.second);
    }
    ymin = std::max(ymin, 0);
    ymax = std::min(ymax, cfg::CANVAS_H - 1);
    std::vector<double> xs;
    for (int y = ymin; y <= ymax; ++y) {
        xs.clear();
        const double sy = y + 0.5;  // sample at the pixel centre
        for (size_t i = 0; i < p.size(); ++i) {
            const auto a = p[i], b = p[(i + 1) % p.size()];
            if ((a.second <= sy) == (b.second <= sy)) continue;
            xs.push_back(a.first + (sy - a.second) * (b.first - a.first) / double(b.second - a.second));
        }
        std::sort(xs.begin(), xs.end());
        for (size_t i = 0; i + 1 < xs.size(); i += 2)
            hspan(cv, static_cast<int>(std::ceil(xs[i] - 0.5)), static_cast<int>(std::floor(xs[i + 1] - 0.5)), y, c);
    }
}

// ---- Cubic Bezier curve (2..4 control points, padded to 4) -----------------
inline void drawBezier(Canvas& cv, const std::vector<std::pair<int, int>>& ctrl, int size, Color c) {
    if (ctrl.empty()) return;
    if (ctrl.size() == 1) {
        stamp(cv, ctrl[0].first, ctrl[0].second, size, c);
        return;
    }
    std::pair<int, int> P[4];
    for (size_t i = 0; i < 4; ++i) P[i] = ctrl[std::min(i, ctrl.size() - 1)];
    double len = 0;
    for (int i = 0; i < 3; ++i) len += std::hypot(double(P[i + 1].first - P[i].first), double(P[i + 1].second - P[i].second));
    const int steps = std::max(16, std::min(500, static_cast<int>(len / 3)));
    int px = P[0].first, py = P[0].second;
    for (int i = 1; i <= steps; ++i) {
        const double t = double(i) / steps, u = 1 - t;
        const double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
        const int x = static_cast<int>(std::lround(b0 * P[0].first + b1 * P[1].first + b2 * P[2].first + b3 * P[3].first));
        const int y = static_cast<int>(std::lround(b0 * P[0].second + b1 * P[1].second + b2 * P[2].second + b3 * P[3].second));
        drawLine(cv, px, py, x, y, size, c);
        px = x;
        py = y;
    }
}

// ---- Airbrush: random dots inside a disc -----------------------------------
inline void spray(Canvas& cv, int cx, int cy, int radius, int dots, Color c, std::mt19937& rng) {
    std::uniform_real_distribution<double> ud(0.0, 1.0);
    for (int i = 0; i < dots; ++i) {
        const double r = radius * std::sqrt(ud(rng)), a = 6.283185307179586 * ud(rng);
        cv.set(cx + static_cast<int>(std::lround(r * std::cos(a))), cy + static_cast<int>(std::lround(r * std::sin(a))), c);
    }
}

}  // namespace gfx

// ============================================================================
//  3. 2D transformation (3x3 homogeneous matrices)
// ============================================================================
struct Mat3 {
    double m[3][3];

    static Mat3 identity() { return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }
    static Mat3 translate(double tx, double ty) { return {{{1, 0, tx}, {0, 1, ty}, {0, 0, 1}}}; }
    static Mat3 scaling(double sx, double sy) { return {{{sx, 0, 0}, {0, sy, 0}, {0, 0, 1}}}; }
    static Mat3 rotate(double rad) {
        const double c = std::cos(rad), s = std::sin(rad);
        return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = 0;
                for (int k = 0; k < 3; ++k) r.m[i][j] += m[i][k] * o.m[k][j];
            }
        return r;
    }
    Mat3 inverse() const {
        const double a = m[0][0], b = m[0][1], c = m[0][2];
        const double d = m[1][0], e = m[1][1], f = m[1][2];
        const double g = m[2][0], h = m[2][1], i = m[2][2];
        const double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        const double k = std::fabs(det) < 1e-12 ? 0.0 : 1.0 / det;
        return {{{(e * i - f * h) * k, (c * h - b * i) * k, (b * f - c * e) * k},
                 {(f * g - d * i) * k, (a * i - c * g) * k, (c * d - a * f) * k},
                 {(d * h - e * g) * k, (b * g - a * h) * k, (a * e - b * d) * k}}};
    }
    void apply(double x, double y, double& ox, double& oy) const {
        ox = m[0][0] * x + m[0][1] * y + m[0][2];
        oy = m[1][0] * x + m[1][1] * y + m[1][2];
    }
};

// ============================================================================
//  4. Application state
// ============================================================================
enum class Tool { Pencil, Brush, Eraser, Line, Rectangle, Ellipse, Circle, Fill, Select, Picker, Spray, Bezier, Polygon };

static const char* toolName(Tool t) {
    switch (t) {
        case Tool::Pencil: return "Pencil";
        case Tool::Brush: return "Brush";
        case Tool::Eraser: return "Eraser";
        case Tool::Line: return "Line";
        case Tool::Rectangle: return "Rectangle";
        case Tool::Ellipse: return "Ellipse";
        case Tool::Circle: return "Circle";
        case Tool::Fill: return "Fill";
        case Tool::Select: return "Select";
        case Tool::Picker: return "Colour picker";
        case Tool::Spray: return "Spray";
        case Tool::Bezier: return "Bezier curve";
        case Tool::Polygon: return "Polygon";
    }
    return "";
}

static const char* toolHint(Tool t) {
    switch (t) {
        case Tool::Pencil: return "Drag to draw 1px lines. Left = primary, right = secondary colour.";
        case Tool::Brush: return "Drag to paint. Mouse wheel or +/- changes the size.";
        case Tool::Eraser: return "Drag to erase to the background colour.";
        case Tool::Line: return "Drag to draw a straight line (Bresenham).";
        case Tool::Rectangle: return "Drag corner to corner. 'Fill shapes' uses the other colour.";
        case Tool::Ellipse: return "Drag the bounding box of the ellipse (midpoint algorithm).";
        case Tool::Circle: return "Drag from the centre outwards (midpoint algorithm).";
        case Tool::Fill: return "Click inside a closed region to flood fill it.";
        case Tool::Select: return "Drag to select; then drag to move, buttons/keys to rotate and scale.";
        case Tool::Picker: return "Click a pixel to copy its colour (left = primary, right = secondary).";
        case Tool::Spray: return "Hold and drag to spray. Size sets the radius.";
        case Tool::Bezier: return "Click 4 control points. Enter or right-click finishes, Esc cancels.";
        case Tool::Polygon: return "Click vertices. Click the first point, right-click or Enter to close. Esc cancels.";
    }
    return "";
}

// Undo / redo using full canvas snapshots on a bounded stack.
class History {
public:
    std::deque<std::vector<Color>> undo, redo;
    void pushPixels(const std::vector<Color>& px) {
        undo.push_back(px);
        if (undo.size() > cfg::UNDO_LIMIT) undo.pop_front();
        redo.clear();
    }
    void push(const Canvas& c) { pushPixels(c.px); }
};

enum class SelState { None, Marquee, Floating };

struct Selection {
    SelState state = SelState::None;
    int mx0 = 0, my0 = 0, mx1 = 0, my1 = 0;  // marquee corners
    int sx = 0, sy = 0, sw = 0, sh = 0;      // source rectangle
    std::vector<Color> buf;                  // lifted pixels (sw * sh)
    std::vector<uint8_t> alpha;              // optional per-pixel alpha (inserted images)
    bool hole = false;                       // true if the source area was cleared when lifted
    double dx = 0, dy = 0, angle = 0, scale = 1;
    bool moving = false;

    // M = T(centre + offset) * R * S * T(-centre)
    Mat3 matrix() const {
        const double cx = sx + sw / 2.0, cy = sy + sh / 2.0;
        return Mat3::translate(cx + dx, cy + dy) * Mat3::rotate(angle) * Mat3::scaling(scale, scale) *
               Mat3::translate(-cx, -cy);
    }
};

static const Color kPalette[16] = {{0, 0, 0},     {128, 128, 128}, {192, 192, 192}, {255, 255, 255},
                                   {255, 0, 0},   {128, 0, 0},     {255, 165, 0},   {255, 255, 0},
                                   {0, 255, 0},   {0, 128, 0},     {0, 255, 255},   {0, 0, 255},
                                   {0, 0, 128},   {128, 0, 128},   {255, 0, 255},   {139, 69, 19}};

static Canvas g_canvas;  // what is shown
static Canvas g_base;    // canvas without the floating selection / shape preview
static History g_hist;
static Selection g_sel;

static Tool g_tool = Tool::Brush;
static int g_size = 3;
static bool g_filled = false;
static Color g_primary{0, 0, 0};
static Color g_secondary{255, 255, 255};

static bool g_stroking = false;
static int g_button = GLUT_LEFT_BUTTON;
static int g_startX = 0, g_startY = 0, g_lastX = 0, g_lastY = 0;
static Color g_strokeColor, g_fillColor;
static int g_strokeSize = 1;

static int g_mouseX = 0, g_mouseY = 0;
static std::string g_status;

// ============================================================================
//  5. Selection and transformation logic
// ============================================================================
static int clampi(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

static void renderFloating() {
    g_canvas.px = g_base.px;
    const Mat3 M = g_sel.matrix();
    const Mat3 Inv = M.inverse();
    const Selection& s = g_sel;

    double xs[4], ys[4];
    const double cxs[4] = {double(s.sx), double(s.sx + s.sw), double(s.sx + s.sw), double(s.sx)};
    const double cys[4] = {double(s.sy), double(s.sy), double(s.sy + s.sh), double(s.sy + s.sh)};
    for (int i = 0; i < 4; ++i) M.apply(cxs[i], cys[i], xs[i], ys[i]);
    const int x0 = std::max(0, static_cast<int>(std::floor(*std::min_element(xs, xs + 4))));
    const int x1 = std::min(cfg::CANVAS_W - 1, static_cast<int>(std::ceil(*std::max_element(xs, xs + 4))));
    const int y0 = std::max(0, static_cast<int>(std::floor(*std::min_element(ys, ys + 4))));
    const int y1 = std::min(cfg::CANVAS_H - 1, static_cast<int>(std::ceil(*std::max_element(ys, ys + 4))));

    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) {
            double u, v;
            Inv.apply(x + 0.5, y + 0.5, u, v);  // inverse mapping: destination -> source
            if (u >= s.sx && u < s.sx + s.sw && v >= s.sy && v < s.sy + s.sh) {
                const int iu = clampi(static_cast<int>(std::floor(u)) - s.sx, 0, s.sw - 1);
                const int iv = clampi(static_cast<int>(std::floor(v)) - s.sy, 0, s.sh - 1);
                const size_t si = static_cast<size_t>(iv) * s.sw + iu;
                Color& d = g_canvas.px[static_cast<size_t>(y) * cfg::CANVAS_W + x];
                const Color c = s.buf[si];
                if (s.alpha.empty()) {
                    d = c;
                } else if (const int a = s.alpha[si]) {
                    d = Color{uint8_t((c.r * a + d.r * (255 - a)) / 255), uint8_t((c.g * a + d.g * (255 - a)) / 255),
                              uint8_t((c.b * a + d.b * (255 - a)) / 255)};
                }
            }
        }
}

static bool selectionHit(int x, int y) {
    double u, v;
    g_sel.matrix().inverse().apply(x + 0.5, y + 0.5, u, v);
    return u >= g_sel.sx && u < g_sel.sx + g_sel.sw && v >= g_sel.sy && v < g_sel.sy + g_sel.sh;
}

static void commitSelection() {
    if (g_sel.state == SelState::Floating) g_sel.buf.clear();
    g_sel.state = SelState::None;
    g_sel.moving = false;
}

// Cancel = restore the snapshot taken when the selection was lifted.
static void cancelSelection() {
    if (g_sel.state == SelState::Floating && !g_hist.undo.empty()) {
        g_canvas.px = g_hist.undo.back();
        g_hist.undo.pop_back();
    }
    g_sel.buf.clear();
    g_sel.state = SelState::None;
    g_sel.moving = false;
}

static void liftSelection() {
    const int x0 = std::min(g_sel.mx0, g_sel.mx1), x1 = std::max(g_sel.mx0, g_sel.mx1);
    const int y0 = std::min(g_sel.my0, g_sel.my1), y1 = std::max(g_sel.my0, g_sel.my1);
    const int w = x1 - x0 + 1, h = y1 - y0 + 1;
    if (w < 3 || h < 3) {
        g_sel.state = SelState::None;
        return;
    }
    g_hist.push(g_canvas);
    g_sel.sx = x0;
    g_sel.sy = y0;
    g_sel.sw = w;
    g_sel.sh = h;
    g_sel.alpha.clear();
    g_sel.hole = true;
    g_sel.buf.assign(static_cast<size_t>(w) * h, kWhite);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) g_sel.buf[static_cast<size_t>(y) * w + x] = g_canvas.get(x0 + x, y0 + y);
    g_base.px = g_canvas.px;
    for (int y = y0; y <= y1; ++y) gfx::hspan(g_base, x0, x1, y, kWhite);
    g_sel.dx = g_sel.dy = g_sel.angle = 0;
    g_sel.scale = 1;
    g_sel.state = SelState::Floating;
    renderFloating();
}

static void transformRotate(double degrees) {
    if (g_sel.state != SelState::Floating) {
        g_status = "Select an area first (Select tool).";
        return;
    }
    g_sel.angle += degrees * 3.14159265358979323846 / 180.0;
    renderFloating();
}

static void transformScale(double factor) {
    if (g_sel.state != SelState::Floating) {
        g_status = "Select an area first (Select tool).";
        return;
    }
    g_sel.scale = std::max(0.1, std::min(8.0, g_sel.scale * factor));
    renderFloating();
}

static void nudgeSelection(int dx, int dy) {
    if (g_sel.state != SelState::Floating) return;
    g_sel.dx += dx;
    g_sel.dy += dy;
    renderFloating();
}

// ---- Multi-click shapes: cubic Bezier curve and polygon --------------------
static std::vector<std::pair<int, int>> g_pts;
static bool g_shapeActive = false;
static Color g_polyStroke, g_polyFill;

static void renderShape(int cx, int cy, bool withCursor) {
    g_canvas.px = g_base.px;
    std::vector<std::pair<int, int>> pts = g_pts;
    if (withCursor) pts.emplace_back(cx, cy);
    if (pts.empty()) return;
    if (g_tool == Tool::Bezier) {
        gfx::drawBezier(g_canvas, pts, g_size, g_polyStroke);
        return;
    }
    if (pts.size() == 1) gfx::stamp(g_canvas, pts[0].first, pts[0].second, g_size, g_polyStroke);
    if (g_filled && pts.size() >= 3) gfx::fillPolygon(g_canvas, pts, g_polyFill);
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        gfx::drawLine(g_canvas, pts[i].first, pts[i].second, pts[i + 1].first, pts[i + 1].second, g_size, g_polyStroke);
    if (pts.size() >= 3)
        gfx::drawLine(g_canvas, pts.back().first, pts.back().second, pts[0].first, pts[0].second, g_size, g_polyStroke);
}

static void finishShape() {
    if (!g_shapeActive) return;
    renderShape(0, 0, false);
    if (g_pts.size() >= 2) g_hist.pushPixels(g_base.px);
    else g_canvas.px = g_base.px;
    g_shapeActive = false;
    g_pts.clear();
}

static void cancelShape() {
    if (!g_shapeActive) return;
    g_canvas.px = g_base.px;
    g_shapeActive = false;
    g_pts.clear();
}

// Finish whatever is pending (selection / unfinished shape) before another action.
static void settle() {
    commitSelection();
    cancelShape();
}

// ============================================================================
//  6. File input / output
// ============================================================================
namespace fileio {

static std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

static std::string askPath(bool save) {
#ifdef _WIN32
    char buf[MAX_PATH] = {0};
    if (save) std::snprintf(buf, sizeof buf, "drawing.png");
    OPENFILENAMEA ofn;
    ZeroMemory(&ofn, sizeof ofn);
    ofn.lStructSize = sizeof ofn;
    ofn.hwndOwner = GetActiveWindow();
    ofn.lpstrFile = buf;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "png";
    if (save) {
        ofn.lpstrFilter = "PNG image (*.png)\0*.png\0BMP image (*.bmp)\0*.bmp\0JPEG image (*.jpg)\0*.jpg\0\0";
        ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    } else {
        ofn.lpstrFilter = "Images (*.png;*.jpg;*.jpeg;*.bmp)\0*.png;*.jpg;*.jpeg;*.bmp\0All files\0*.*\0\0";
        ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    }
    const BOOL ok = save ? GetSaveFileNameA(&ofn) : GetOpenFileNameA(&ofn);
    return ok ? std::string(buf) : std::string();
#else
    std::cout << (save ? "Save as (path, .png/.bmp/.jpg): " : "Open image (path): ") << std::flush;
    std::string p;
    std::getline(std::cin, p);
    return p;
#endif
}

static bool saveImage(const Canvas& cv, std::string path) {
    std::string ext;
    const size_t dot = path.find_last_of('.');
    const size_t slash = path.find_last_of("/\\");
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) ext = lower(path.substr(dot + 1));
    if (ext.empty()) {
        path += ".png";
        ext = "png";
    }
    const void* data = cv.px.data();
    int ok = 0;
    if (ext == "bmp") ok = stbi_write_bmp(path.c_str(), cfg::CANVAS_W, cfg::CANVAS_H, 3, data);
    else if (ext == "jpg" || ext == "jpeg") ok = stbi_write_jpg(path.c_str(), cfg::CANVAS_W, cfg::CANVAS_H, 3, data, 92);
    else ok = stbi_write_png(path.c_str(), cfg::CANVAS_W, cfg::CANVAS_H, 3, data, cfg::CANVAS_W * 3);
    return ok != 0;
}

struct Image {
    int w = 0, h = 0;
    std::vector<Color> px;
    std::vector<uint8_t> a;  // alpha
};

// Loads an image with alpha. If it is larger than maxFrac of the canvas it is scaled down.
static bool loadRGBA(const std::string& path, double maxFrac, Image& out) {
    int iw = 0, ih = 0, n = 0;
    unsigned char* data = stbi_load(path.c_str(), &iw, &ih, &n, 4);
    if (!data) return false;
    const double k = std::min(1.0, std::min(cfg::CANVAS_W * maxFrac / iw, cfg::CANVAS_H * maxFrac / ih));
    out.w = std::max(1, int(iw * k));
    out.h = std::max(1, int(ih * k));
    out.px.resize(static_cast<size_t>(out.w) * out.h);
    out.a.resize(out.px.size());
    for (int y = 0; y < out.h; ++y)
        for (int x = 0; x < out.w; ++x) {
            const int sx = std::min(iw - 1, int(x / k)), sy = std::min(ih - 1, int(y / k));
            const unsigned char* p = data + (static_cast<size_t>(sy) * iw + sx) * 4;
            out.px[static_cast<size_t>(y) * out.w + x] = Color{p[0], p[1], p[2]};
            out.a[static_cast<size_t>(y) * out.w + x] = p[3];
        }
    stbi_image_free(data);
    return true;
}

// Opens an image as the new canvas (transparent areas become white).
static bool loadImage(Canvas& out, const std::string& path) {
    Image im;
    if (!loadRGBA(path, 1.0, im)) return false;
    out.clear(kWhite);
    for (int y = 0; y < im.h; ++y)
        for (int x = 0; x < im.w; ++x) {
            const size_t i = static_cast<size_t>(y) * im.w + x;
            const int a = im.a[i];
            const Color c = im.px[i];
            out.set(x, y, Color{uint8_t((c.r * a + 255 * (255 - a)) / 255), uint8_t((c.g * a + 255 * (255 - a)) / 255),
                                uint8_t((c.b * a + 255 * (255 - a)) / 255)});
        }
    return true;
}

}  // namespace fileio

// ============================================================================
//  7. Editor actions
// ============================================================================
static void setTool(Tool t) {
    if (t != g_tool) cancelShape();
    if (g_tool == Tool::Select && t != Tool::Select) commitSelection();
    g_tool = t;
    g_status.clear();
}

static void actionUndo() {
    if (g_shapeActive) {
        cancelShape();
        return;
    }
    if (g_sel.state == SelState::Floating) {
        cancelSelection();
        return;
    }
    g_sel.state = SelState::None;
    if (g_hist.undo.empty()) return;
    g_hist.redo.push_back(g_canvas.px);
    g_canvas.px = g_hist.undo.back();
    g_hist.undo.pop_back();
}

static void actionRedo() {
    settle();
    if (g_hist.redo.empty()) return;
    g_hist.undo.push_back(g_canvas.px);
    g_canvas.px = g_hist.redo.back();
    g_hist.redo.pop_back();
}

static void actionClear() {
    settle();
    g_hist.push(g_canvas);
    g_canvas.clear(kWhite);
}

static void actionSave() {
    settle();
    const std::string p = fileio::askPath(true);
    if (p.empty()) return;
    g_status = fileio::saveImage(g_canvas, p) ? "Saved: " + p : "Could not save: " + p;
}

static void actionOpen() {
    settle();
    const std::string p = fileio::askPath(false);
    if (p.empty()) return;
    Canvas tmp;
    if (fileio::loadImage(tmp, p)) {
        g_hist.push(g_canvas);
        g_canvas = tmp;
        g_status = "Opened: " + p;
    } else {
        g_status = "Could not open: " + p;
    }
}

// ---- Floating pixels: insert image, paste ---------------------------------
struct Clip {
    int w = 0, h = 0, x = 0, y = 0;
    std::vector<Color> px;
    std::vector<uint8_t> a;
};
static Clip g_clip;

static void beginFloating(std::vector<Color> px, std::vector<uint8_t> a, int w, int h, int x, int y) {
    g_hist.push(g_canvas);
    g_base.px = g_canvas.px;
    g_sel.sx = clampi(x, 0, std::max(0, cfg::CANVAS_W - w));
    g_sel.sy = clampi(y, 0, std::max(0, cfg::CANVAS_H - h));
    g_sel.sw = w;
    g_sel.sh = h;
    g_sel.buf = std::move(px);
    g_sel.alpha = std::move(a);
    g_sel.hole = false;
    g_sel.dx = g_sel.dy = g_sel.angle = 0;
    g_sel.scale = 1;
    g_sel.moving = false;
    g_sel.state = SelState::Floating;
    renderFloating();
}

static void actionInsertImage() {
    settle();
    const std::string p = fileio::askPath(false);
    if (p.empty()) return;
    fileio::Image im;
    if (!fileio::loadRGBA(p, 0.9, im)) {
        g_status = "Could not open: " + p;
        return;
    }
    setTool(Tool::Select);
    const int w = im.w, h = im.h;
    beginFloating(std::move(im.px), std::move(im.a), w, h, (cfg::CANVAS_W - w) / 2, (cfg::CANVAS_H - h) / 2);
    g_status = "Image inserted: drag to move, rotate/scale, Enter to apply.";
}

static void storeClip() {
    g_clip.w = g_sel.sw;
    g_clip.h = g_sel.sh;
    g_clip.x = g_sel.sx;
    g_clip.y = g_sel.sy;
    g_clip.px = g_sel.buf;
    g_clip.a = g_sel.alpha;
}

static void actionCopy() {
    if (g_sel.state != SelState::Floating) {
        g_status = "Select an area first (Select tool).";
        return;
    }
    storeClip();
    if (g_sel.hole) {  // put the original back so the floating pixels become a duplicate
        for (int y = 0; y < g_sel.sh; ++y)
            for (int x = 0; x < g_sel.sw; ++x)
                g_base.px[static_cast<size_t>(g_sel.sy + y) * cfg::CANVAS_W + g_sel.sx + x] =
                    g_sel.buf[static_cast<size_t>(y) * g_sel.sw + x];
        g_sel.hole = false;
        renderFloating();
    }
    g_status = "Copied. Use Paste (Ctrl+V) to add a copy.";
}

static void actionCut() {
    if (g_sel.state != SelState::Floating) {
        g_status = "Select an area first (Select tool).";
        return;
    }
    storeClip();
    g_canvas.px = g_base.px;
    g_sel.buf.clear();
    g_sel.alpha.clear();
    g_sel.state = SelState::None;
    g_sel.moving = false;
    g_status = "Cut.";
}

static void actionPaste() {
    if (g_clip.px.empty()) {
        g_status = "Clipboard is empty. Select an area and Copy first.";
        return;
    }
    settle();
    setTool(Tool::Select);
    g_clip.x = clampi(g_clip.x + 20, 0, std::max(0, cfg::CANVAS_W - g_clip.w));
    g_clip.y = clampi(g_clip.y + 20, 0, std::max(0, cfg::CANVAS_H - g_clip.h));
    beginFloating(g_clip.px, g_clip.a, g_clip.w, g_clip.h, g_clip.x, g_clip.y);
    g_status = "Pasted: drag to move, Enter to apply.";
}

// ---- Flip -------------------------------------------------------------------
template <class T>
static void flipBuffer(std::vector<T>& v, int w, int h, bool horizontal) {
    if (v.empty()) return;
    if (horizontal) {
        for (int y = 0; y < h; ++y) std::reverse(v.begin() + std::ptrdiff_t(y) * w, v.begin() + std::ptrdiff_t(y + 1) * w);
    } else {
        for (int y = 0; y < h / 2; ++y)
            std::swap_ranges(v.begin() + std::ptrdiff_t(y) * w, v.begin() + std::ptrdiff_t(y + 1) * w,
                             v.begin() + std::ptrdiff_t(h - 1 - y) * w);
    }
}

static void actionFlip(bool horizontal) {
    if (g_sel.state == SelState::Floating) {
        flipBuffer(g_sel.buf, g_sel.sw, g_sel.sh, horizontal);
        flipBuffer(g_sel.alpha, g_sel.sw, g_sel.sh, horizontal);
        renderFloating();
        return;
    }
    settle();
    g_hist.push(g_canvas);
    flipBuffer(g_canvas.px, cfg::CANVAS_W, cfg::CANVAS_H, horizontal);
}

// ---- Image filters (selection if there is one, otherwise the whole canvas) --
enum class Filter { Gray, Invert, Sepia, Blur, Sharpen, Edges, BrightUp, BrightDown, ContrastUp, ContrastDown };

static uint8_t clamp8(double v) { return static_cast<uint8_t>(std::max(0.0, std::min(255.0, v + 0.5))); }

static void convolve3(std::vector<Color>& px, int w, int h, const double k[9], double div, double bias) {
    const std::vector<Color> src = px;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            double r = 0, g = 0, b = 0;
            for (int j = -1; j <= 1; ++j)
                for (int i = -1; i <= 1; ++i) {
                    const Color& c = src[static_cast<size_t>(clampi(y + j, 0, h - 1)) * w + clampi(x + i, 0, w - 1)];
                    const double kv = k[(j + 1) * 3 + (i + 1)];
                    r += c.r * kv;
                    g += c.g * kv;
                    b += c.b * kv;
                }
            px[static_cast<size_t>(y) * w + x] = Color{clamp8(r / div + bias), clamp8(g / div + bias), clamp8(b / div + bias)};
        }
}

static void filterPixels(std::vector<Color>& px, int w, int h, Filter f) {
    static const double kBlur[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};
    static const double kSharpen[9] = {0, -1, 0, -1, 5, -1, 0, -1, 0};
    static const double kEdges[9] = {-1, -1, -1, -1, 8, -1, -1, -1, -1};
    switch (f) {
        case Filter::Blur: convolve3(px, w, h, kBlur, 16, 0); return;
        case Filter::Sharpen: convolve3(px, w, h, kSharpen, 1, 0); return;
        case Filter::Edges: convolve3(px, w, h, kEdges, 1, 0); return;
        default: break;
    }
    for (Color& c : px) {
        const double r = c.r, g = c.g, b = c.b;
        switch (f) {
            case Filter::Gray: {
                const uint8_t l = clamp8(0.299 * r + 0.587 * g + 0.114 * b);
                c = Color{l, l, l};
                break;
            }
            case Filter::Invert: c = Color{uint8_t(255 - c.r), uint8_t(255 - c.g), uint8_t(255 - c.b)}; break;
            case Filter::Sepia:
                c = Color{clamp8(0.393 * r + 0.769 * g + 0.189 * b), clamp8(0.349 * r + 0.686 * g + 0.168 * b),
                          clamp8(0.272 * r + 0.534 * g + 0.131 * b)};
                break;
            case Filter::BrightUp: c = Color{clamp8(r + 25), clamp8(g + 25), clamp8(b + 25)}; break;
            case Filter::BrightDown: c = Color{clamp8(r - 25), clamp8(g - 25), clamp8(b - 25)}; break;
            case Filter::ContrastUp:
                c = Color{clamp8((r - 128) * 1.2 + 128), clamp8((g - 128) * 1.2 + 128), clamp8((b - 128) * 1.2 + 128)};
                break;
            case Filter::ContrastDown:
                c = Color{clamp8((r - 128) / 1.2 + 128), clamp8((g - 128) / 1.2 + 128), clamp8((b - 128) / 1.2 + 128)};
                break;
            default: break;
        }
    }
}

static void applyFilter(Filter f) {
    if (g_sel.state == SelState::Floating) {
        filterPixels(g_sel.buf, g_sel.sw, g_sel.sh, f);
        renderFloating();
        return;
    }
    settle();
    g_hist.push(g_canvas);
    filterPixels(g_canvas.px, cfg::CANVAS_W, cfg::CANVAS_H, f);
}

static void changeSize(int delta) { g_size = clampi(g_size + delta, 1, cfg::MAX_BRUSH); }

// ============================================================================
//  8. User interface (widgets)
// ============================================================================
struct Button {
    int x, y, w, h;
    std::string label;
    std::function<void()> action;
    std::function<bool()> active;
};

struct Slider {
    int x, y, w, lo, hi;
    std::string label;
    Color tint;
    std::function<int()> get;
    std::function<void(int)> set;
};

static std::vector<Button> g_buttons;
static std::vector<Slider> g_sliders;
static int g_dragSlider = -1;
static int g_hoverButton = -1;

static const int kSwatchX = cfg::PAL_X + 80, kSwatchY = cfg::PAL_Y + 10, kSwatchSize = 28, kSwatchStep = 32;

static bool inCanvas(int x, int y) {
    return x >= cfg::CANVAS_X && y >= cfg::CANVAS_Y && x < cfg::CANVAS_X + cfg::CANVAS_W &&
           y < cfg::CANVAS_Y + cfg::CANVAS_H;
}

static void addButton(int x, int y, int w, int h, const std::string& label, std::function<void()> action,
                      std::function<bool()> active = nullptr) {
    g_buttons.push_back({x, y, w, h, label, std::move(action), std::move(active)});
}

static void addToolButton(int x, int y, const char* label, Tool t) {
    addButton(x, y, 75, 28, label, [t] { setTool(t); }, [t] { return g_tool == t; });
}

static void buildUI() {
    const int a1 = 10, a2 = 85, b1 = 180, b2 = 255;
    auto row = [](int r) { return 24 + 32 * r; };

    // ---- Panel A: tools ----
    addToolButton(a1, row(0), "Pencil", Tool::Pencil);
    addToolButton(a2, row(0), "Brush", Tool::Brush);
    addToolButton(a1, row(1), "Eraser", Tool::Eraser);
    addToolButton(a2, row(1), "Line", Tool::Line);
    addToolButton(a1, row(2), "Rectangle", Tool::Rectangle);
    addToolButton(a2, row(2), "Ellipse", Tool::Ellipse);
    addToolButton(a1, row(3), "Circle", Tool::Circle);
    addToolButton(a2, row(3), "Fill", Tool::Fill);
    addToolButton(a1, row(4), "Select", Tool::Select);
    addToolButton(a2, row(4), "Picker", Tool::Picker);
    addToolButton(a1, row(5), "Spray", Tool::Spray);
    addToolButton(a2, row(5), "Curve", Tool::Bezier);
    addToolButton(a1, row(6), "Polygon", Tool::Polygon);
    addButton(a2, row(6), 75, 28, "Fill shapes", [] { g_filled = !g_filled; }, [] { return g_filled; });

    // ---- Panel A: actions ----
    addButton(a1, 310, 75, 28, "Undo", actionUndo);
    addButton(a2, 310, 75, 28, "Redo", actionRedo);
    addButton(a1, 342, 75, 28, "Save", actionSave);
    addButton(a2, 342, 75, 28, "Open", actionOpen);
    addButton(a1, 374, 150, 28, "Clear canvas", actionClear);

    // ---- Panel A: transformations ----
    addButton(a1, 432, 75, 28, "Rotate -", [] { transformRotate(-15); });
    addButton(a2, 432, 75, 28, "Rotate +", [] { transformRotate(15); });
    addButton(a1, 464, 75, 28, "Scale -", [] { transformScale(1 / 1.1); });
    addButton(a2, 464, 75, 28, "Scale +", [] { transformScale(1.1); });
    addButton(a1, 496, 75, 28, "Apply", commitSelection);
    addButton(a2, 496, 75, 28, "Cancel", cancelSelection);

    // ---- Panel B: image ----
    addButton(b1, 24, 150, 28, "Insert image...", actionInsertImage);
    addButton(b1, 56, 75, 28, "Copy", actionCopy);
    addButton(b2, 56, 75, 28, "Paste", actionPaste);
    addButton(b1, 88, 75, 28, "Cut", actionCut);
    addButton(b2, 88, 75, 28, "Rotate 90", [] { transformRotate(90); });
    addButton(b1, 120, 75, 28, "Flip H", [] { actionFlip(true); });
    addButton(b2, 120, 75, 28, "Flip V", [] { actionFlip(false); });

    // ---- Panel B: filters ----
    addButton(b1, 176, 75, 28, "Grayscale", [] { applyFilter(Filter::Gray); });
    addButton(b2, 176, 75, 28, "Invert", [] { applyFilter(Filter::Invert); });
    addButton(b1, 208, 75, 28, "Sepia", [] { applyFilter(Filter::Sepia); });
    addButton(b2, 208, 75, 28, "Blur", [] { applyFilter(Filter::Blur); });
    addButton(b1, 240, 75, 28, "Sharpen", [] { applyFilter(Filter::Sharpen); });
    addButton(b2, 240, 75, 28, "Edges", [] { applyFilter(Filter::Edges); });
    addButton(b1, 272, 75, 28, "Bright +", [] { applyFilter(Filter::BrightUp); });
    addButton(b2, 272, 75, 28, "Bright -", [] { applyFilter(Filter::BrightDown); });
    addButton(b1, 304, 75, 28, "Contrast +", [] { applyFilter(Filter::ContrastUp); });
    addButton(b2, 304, 75, 28, "Contrast -", [] { applyFilter(Filter::ContrastDown); });

    // Brush size slider
    g_sliders.push_back({a1 + 4, 280, 140, 1, cfg::MAX_BRUSH, "Size", Color{200, 200, 200}, [] { return g_size; },
                         [](int v) { g_size = v; }});
    // RGB sliders (edit the primary colour)
    const int sx = cfg::PAL_X + 395, sw = 250;
    g_sliders.push_back({sx, cfg::PAL_Y + 22, sw, 0, 255, "R", Color{230, 70, 70}, [] { return int(g_primary.r); },
                         [](int v) { g_primary.r = uint8_t(v); }});
    g_sliders.push_back({sx, cfg::PAL_Y + 44, sw, 0, 255, "G", Color{70, 200, 90}, [] { return int(g_primary.g); },
                         [](int v) { g_primary.g = uint8_t(v); }});
    g_sliders.push_back({sx, cfg::PAL_Y + 66, sw, 0, 255, "B", Color{80, 120, 240}, [] { return int(g_primary.b); },
                         [](int v) { g_primary.b = uint8_t(v); }});
}

static void setSliderFromX(int idx, int mx) {
    const Slider& s = g_sliders[idx];
    const double t = std::max(0.0, std::min(1.0, double(mx - s.x) / s.w));
    s.set(s.lo + static_cast<int>(std::lround(t * (s.hi - s.lo))));
}

static bool hitRect(int px, int py, int x, int y, int w, int h) { return px >= x && py >= y && px < x + w && py < y + h; }

static int buttonAt(int x, int y) {
    for (size_t i = 0; i < g_buttons.size(); ++i) {
        const Button& b = g_buttons[i];
        if (hitRect(x, y, b.x, b.y, b.w, b.h)) return static_cast<int>(i);
    }
    return -1;
}

// Returns true when the click was consumed by the UI (anything outside the canvas).
static bool uiPress(int button, int x, int y) {
    if (inCanvas(x, y)) return false;
    const bool left = (button == GLUT_LEFT_BUTTON);
    if (left) {
        for (size_t i = 0; i < g_sliders.size(); ++i) {
            const Slider& s = g_sliders[i];
            if (hitRect(x, y, s.x - 6, s.y - 8, s.w + 12, 20)) {
                g_dragSlider = static_cast<int>(i);
                setSliderFromX(g_dragSlider, x);
                return true;
            }
        }
        const int b = buttonAt(x, y);
        if (b >= 0) {
            g_buttons[b].action();
            return true;
        }
    }
    for (int i = 0; i < 16; ++i) {
        const int sx = kSwatchX + (i % 8) * kSwatchStep, sy = kSwatchY + (i / 8) * (kSwatchSize + 8);
        if (hitRect(x, y, sx, sy, kSwatchSize, kSwatchSize)) {
            if (left) g_primary = kPalette[i];
            else if (button == GLUT_RIGHT_BUTTON) g_secondary = kPalette[i];
            return true;
        }
    }
    // Click on the colour preview: swap primary and secondary
    if (left && hitRect(x, y, cfg::PAL_X + 12, cfg::PAL_Y + 12, 52, 52)) std::swap(g_primary, g_secondary);
    return true;
}

// ---- Drawing helpers for the UI (this is the only place plain GL is used) --
static void glColor(Color c) { glColor3ub(c.r, c.g, c.b); }

static void fillRect(int x, int y, int w, int h, Color c) {
    glColor(c);
    glBegin(GL_QUADS);
    glVertex2i(x, y);
    glVertex2i(x + w, y);
    glVertex2i(x + w, y + h);
    glVertex2i(x, y + h);
    glEnd();
}

static void strokeRect(int x, int y, int w, int h, Color c) {
    glColor(c);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x + 0.5f, y + 0.5f);
    glVertex2f(x + w - 0.5f, y + 0.5f);
    glVertex2f(x + w - 0.5f, y + h - 0.5f);
    glVertex2f(x + 0.5f, y + h - 0.5f);
    glEnd();
}

static void drawText(int x, int y, const std::string& s, Color c, void* font = GLUT_BITMAP_HELVETICA_12) {
    glColor(c);
    glRasterPos2i(x, y);
    glutBitmapString(font, reinterpret_cast<const unsigned char*>(s.c_str()));
}

static int textWidth(const std::string& s, void* font = GLUT_BITMAP_HELVETICA_12) {
    return glutBitmapLength(font, reinterpret_cast<const unsigned char*>(s.c_str()));
}

static void drawButtons() {
    for (size_t i = 0; i < g_buttons.size(); ++i) {
        const Button& b = g_buttons[i];
        const bool active = b.active && b.active();
        Color bg = active ? Color{0, 110, 190} : (int(i) == g_hoverButton ? Color{78, 78, 82} : Color{58, 58, 62});
        fillRect(b.x, b.y, b.w, b.h, bg);
        strokeRect(b.x, b.y, b.w, b.h, active ? Color{70, 160, 230} : Color{90, 90, 96});
        drawText(b.x + (b.w - textWidth(b.label)) / 2, b.y + b.h / 2 + 4, b.label, Color{235, 235, 235});
    }
}

static void drawSliders() {
    for (const Slider& s : g_sliders) {
        const int v = s.get();
        const double t = double(v - s.lo) / (s.hi - s.lo);
        const int hx = s.x + static_cast<int>(t * s.w);
        fillRect(s.x, s.y, s.w, 4, Color{90, 90, 96});
        fillRect(s.x, s.y, hx - s.x, 4, s.tint);
        fillRect(hx - 4, s.y - 5, 8, 14, Color{225, 225, 225});
        if (s.label == "Size") {
            drawText(s.x - 4, s.y - 12, "Brush size: " + std::to_string(v), Color{200, 200, 200});
        } else {
            drawText(s.x - 18, s.y + 6, s.label, s.tint);
            drawText(s.x + s.w + 12, s.y + 6, std::to_string(v), Color{210, 210, 210});
        }
    }
}

static void drawPalette() {
    using namespace cfg;
    fillRect(PAL_X, PAL_Y, PAL_W, PAL_H, Color{45, 45, 48});
    // Current colours: secondary behind, primary in front
    fillRect(PAL_X + 30, PAL_Y + 30, 40, 40, g_secondary);
    strokeRect(PAL_X + 30, PAL_Y + 30, 40, 40, Color{200, 200, 200});
    fillRect(PAL_X + 12, PAL_Y + 12, 40, 40, g_primary);
    strokeRect(PAL_X + 12, PAL_Y + 12, 40, 40, Color{255, 255, 255});
    for (int i = 0; i < 16; ++i) {
        const int sx = kSwatchX + (i % 8) * kSwatchStep, sy = kSwatchY + (i / 8) * (kSwatchSize + 8);
        fillRect(sx, sy, kSwatchSize, kSwatchSize, kPalette[i]);
        strokeRect(sx, sy, kSwatchSize, kSwatchSize, Color{110, 110, 115});
    }
    drawText(PAL_X + 4, PAL_Y + PAL_H - 6, "L-click: primary  R-click: secondary", Color{150, 150, 155}, GLUT_BITMAP_HELVETICA_10);
}

static void drawToolbarLabels() {
    const Color dim{150, 150, 158};
    drawText(10, 18, "TOOLS", dim);
    drawText(10, 304, "ACTIONS", dim);
    drawText(10, 426, "TRANSFORM (Select tool)", dim);
    drawText(180, 18, "IMAGE", dim);
    drawText(180, 170, "FILTERS", dim);
    drawText(180, 350, "Filters act on the selection,", Color{130, 130, 138}, GLUT_BITMAP_HELVETICA_10);
    drawText(180, 363, "or on the whole canvas.", Color{130, 130, 138}, GLUT_BITMAP_HELVETICA_10);
    drawText(180, 392, "KEYS", dim);
    const char* keys[] = {"P B E  pencil brush eraser",
                          "L R O C  line rect oval circle",
                          "F fill  S select  I picker",
                          "A spray  V curve  G polygon",
                          "M  toggle fill shapes",
                          "+ -  or wheel: brush size",
                          ", .  rotate    [ ]  scale",
                          "Arrows nudge   Enter apply",
                          "Esc cancel",
                          "Ctrl+Z / Y  undo, redo",
                          "Ctrl+C / X / V  copy cut paste",
                          "Ctrl+I  insert image",
                          "Ctrl+S / O  save, open",
                          "Ctrl+N  clear canvas"};
    int y = 408;
    for (const char* k : keys) {
        drawText(180, y, k, Color{130, 130, 138}, GLUT_BITMAP_HELVETICA_10);
        y += 13;
    }
}

static void drawStatusBar() {
    using namespace cfg;
    fillRect(0, WIN_H - STATUS_H, WIN_W, STATUS_H, Color{0, 90, 160});
    std::string left = std::string(toolName(g_tool)) + ": " + (g_status.empty() ? toolHint(g_tool) : g_status.c_str());
    drawText(10, WIN_H - 8, left, Color{255, 255, 255});
    std::string right;
    if (inCanvas(g_mouseX, g_mouseY))
        right = "x: " + std::to_string(g_mouseX - CANVAS_X) + "  y: " + std::to_string(g_mouseY - CANVAS_Y) + "   ";
    right += "size " + std::to_string(g_tool == Tool::Pencil ? 1 : g_size) + "   " + std::to_string(CANVAS_W) + "x" +
             std::to_string(CANVAS_H);
    drawText(WIN_W - textWidth(right) - 10, WIN_H - 8, right, Color{255, 255, 255});
}

static void drawOverlays() {
    using namespace cfg;
    glEnable(GL_SCISSOR_TEST);
    glScissor(CANVAS_X, WIN_H - CANVAS_Y - CANVAS_H, CANVAS_W, CANVAS_H);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0x00FF);
    glColor3ub(0, 0, 0);
    if (g_sel.state == SelState::Marquee) {
        const int x0 = std::min(g_sel.mx0, g_sel.mx1), x1 = std::max(g_sel.mx0, g_sel.mx1);
        const int y0 = std::min(g_sel.my0, g_sel.my1), y1 = std::max(g_sel.my0, g_sel.my1);
        strokeRect(CANVAS_X + x0, CANVAS_Y + y0, x1 - x0 + 1, y1 - y0 + 1, Color{0, 0, 0});
    } else if (g_sel.state == SelState::Floating) {
        const Mat3 M = g_sel.matrix();
        const double cx[4] = {double(g_sel.sx), double(g_sel.sx + g_sel.sw), double(g_sel.sx + g_sel.sw), double(g_sel.sx)};
        const double cy[4] = {double(g_sel.sy), double(g_sel.sy), double(g_sel.sy + g_sel.sh), double(g_sel.sy + g_sel.sh)};
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 4; ++i) {
            double x, y;
            M.apply(cx[i], cy[i], x, y);
            glVertex2d(CANVAS_X + x, CANVAS_Y + y);
        }
        glEnd();
    }
    glDisable(GL_LINE_STIPPLE);
    if (g_shapeActive)
        for (const auto& p : g_pts) fillRect(CANVAS_X + p.first - 3, CANVAS_Y + p.second - 3, 7, 7, Color{0, 120, 215});
    glDisable(GL_SCISSOR_TEST);
}

// ============================================================================
//  9. Canvas input handling
// ============================================================================
static void drawShapePreview(int x, int y) {
    g_canvas.px = g_base.px;
    const int sx = g_startX, sy = g_startY;
    switch (g_tool) {
        case Tool::Line: gfx::drawLine(g_canvas, sx, sy, x, y, g_size, g_strokeColor); break;
        case Tool::Rectangle: gfx::drawRect(g_canvas, sx, sy, x, y, g_size, g_strokeColor, g_fillColor, g_filled); break;
        case Tool::Circle: {
            const int r = static_cast<int>(std::lround(std::hypot(double(x - sx), double(y - sy))));
            gfx::drawCircle(g_canvas, sx, sy, r, g_size, g_strokeColor, g_fillColor, g_filled);
            break;
        }
        case Tool::Ellipse:
            gfx::drawEllipse(g_canvas, (sx + x) / 2, (sy + y) / 2, std::abs(x - sx) / 2, std::abs(y - sy) / 2, g_size,
                             g_strokeColor, g_fillColor, g_filled);
            break;
        default: break;
    }
}

static std::mt19937 g_rng{12345};

static void doSpray(int x, int y) {
    const int r = g_size * 2 + 2;
    gfx::spray(g_canvas, x, y, r, std::max(6, r * r / 8), g_strokeColor, g_rng);
}

static void canvasPress(int button, int x, int y) {
    if (g_stroking) return;
    g_status.clear();
    g_button = button;
    const Color main = (button == GLUT_LEFT_BUTTON) ? g_primary : g_secondary;
    const Color other = (button == GLUT_LEFT_BUTTON) ? g_secondary : g_primary;
    g_startX = g_lastX = x;
    g_startY = g_lastY = y;

    switch (g_tool) {
        case Tool::Fill:
            if (g_canvas.get(x, y) != main) {
                g_hist.push(g_canvas);
                gfx::floodFill(g_canvas, x, y, main);
            }
            break;
        case Tool::Pencil:
        case Tool::Brush:
        case Tool::Eraser:
            g_hist.push(g_canvas);
            g_stroking = true;
            g_strokeColor = (g_tool == Tool::Eraser) ? kWhite : main;
            g_strokeSize = (g_tool == Tool::Pencil) ? 1 : g_size;
            gfx::stamp(g_canvas, x, y, g_strokeSize, g_strokeColor);
            break;
        case Tool::Line:
        case Tool::Rectangle:
        case Tool::Ellipse:
        case Tool::Circle:
            g_hist.push(g_canvas);
            g_base.px = g_canvas.px;
            g_stroking = true;
            g_strokeColor = main;
            g_fillColor = other;
            break;
        case Tool::Picker: {
            const Color c = g_canvas.get(x, y);
            (button == GLUT_LEFT_BUTTON ? g_primary : g_secondary) = c;
            g_status = "Picked colour rgb(" + std::to_string(c.r) + ", " + std::to_string(c.g) + ", " + std::to_string(c.b) + ")";
            break;
        }
        case Tool::Spray:
            g_hist.push(g_canvas);
            g_stroking = true;
            g_strokeColor = main;
            doSpray(x, y);
            break;
        case Tool::Bezier:
        case Tool::Polygon:
            if (button == GLUT_RIGHT_BUTTON && g_shapeActive) {
                finishShape();
                break;
            }
            if (!g_shapeActive) {
                g_base.px = g_canvas.px;
                g_shapeActive = true;
                g_polyStroke = main;
                g_polyFill = other;
                g_pts.clear();
            }
            if (g_tool == Tool::Polygon && g_pts.size() >= 3 &&
                std::hypot(double(x - g_pts[0].first), double(y - g_pts[0].second)) <= 6.0) {
                finishShape();
                break;
            }
            g_pts.emplace_back(x, y);
            if (g_tool == Tool::Bezier && g_pts.size() == 4) {
                finishShape();
                break;
            }
            renderShape(x, y, false);
            break;
        case Tool::Select:
            g_stroking = true;
            if (g_sel.state == SelState::Floating) {
                if (selectionHit(x, y)) {
                    g_sel.moving = true;
                    break;
                }
                commitSelection();
            }
            g_sel.state = SelState::Marquee;
            g_sel.mx0 = g_sel.mx1 = clampi(x, 0, cfg::CANVAS_W - 1);
            g_sel.my0 = g_sel.my1 = clampi(y, 0, cfg::CANVAS_H - 1);
            break;
    }
}

static void canvasDrag(int x, int y) {
    if (g_shapeActive) {
        renderShape(x, y, true);
        return;
    }
    if (!g_stroking) return;
    switch (g_tool) {
        case Tool::Pencil:
        case Tool::Brush:
        case Tool::Eraser:
            gfx::drawLine(g_canvas, g_lastX, g_lastY, x, y, g_strokeSize, g_strokeColor);
            break;
        case Tool::Line:
        case Tool::Rectangle:
        case Tool::Ellipse:
        case Tool::Circle: drawShapePreview(x, y); break;
        case Tool::Spray: {
            const int r = g_size * 2 + 2;
            const double dist = std::hypot(double(x - g_lastX), double(y - g_lastY));
            const int n = std::max(1, static_cast<int>(dist / (r / 2.0 + 1)));
            for (int i = 1; i <= n; ++i) doSpray(g_lastX + (x - g_lastX) * i / n, g_lastY + (y - g_lastY) * i / n);
            break;
        }
        case Tool::Select:
            if (g_sel.moving) {
                g_sel.dx += x - g_lastX;
                g_sel.dy += y - g_lastY;
                renderFloating();
            } else if (g_sel.state == SelState::Marquee) {
                g_sel.mx1 = clampi(x, 0, cfg::CANVAS_W - 1);
                g_sel.my1 = clampi(y, 0, cfg::CANVAS_H - 1);
            }
            break;
        default: break;
    }
    g_lastX = x;
    g_lastY = y;
}

static void canvasRelease() {
    if (!g_stroking) return;
    g_stroking = false;
    if (g_tool == Tool::Select) {
        if (g_sel.moving) g_sel.moving = false;
        else if (g_sel.state == SelState::Marquee) liftSelection();
        return;
    }
    // Drop the undo entry if the stroke changed nothing.
    if (!g_hist.undo.empty() && g_canvas.px == g_hist.undo.back()) g_hist.undo.pop_back();
}

// ============================================================================
//  10. GLUT callbacks
// ============================================================================
static void onDisplay() {
    using namespace cfg;
    glClearColor(30 / 255.f, 30 / 255.f, 32 / 255.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    fillRect(0, 0, TB_W, WIN_H - STATUS_H, Color{40, 40, 43});
    fillRect(170, 0, TB_W - 170, WIN_H - STATUS_H, Color{47, 47, 52});
    fillRect(CANVAS_X - 1, CANVAS_Y - 1, CANVAS_W + 2, CANVAS_H + 2, Color{0, 0, 0});

    // The canvas pixel buffer is the only thing drawn through glDrawPixels.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelZoom(1.f, -1.f);  // our rows are stored top-down
    glRasterPos2i(CANVAS_X, CANVAS_Y);
    glDrawPixels(CANVAS_W, CANVAS_H, GL_RGB, GL_UNSIGNED_BYTE, g_canvas.px.data());
    glPixelZoom(1.f, 1.f);

    drawOverlays();
    drawPalette();
    drawButtons();
    drawSliders();
    drawToolbarLabels();
    drawStatusBar();
    glutSwapBuffers();
}

static void onReshape(int w, int h) {
    if (w != cfg::WIN_W || h != cfg::WIN_H) {  // fixed-size window
        glutReshapeWindow(cfg::WIN_W, cfg::WIN_H);
        return;
    }
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, w, h, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

static void onMouse(int button, int state, int x, int y) {
    if (button != GLUT_LEFT_BUTTON && button != GLUT_RIGHT_BUTTON) return;
    if (state == GLUT_DOWN) {
        if (!g_stroking && !uiPress(button, x, y) && inCanvas(x, y)) canvasPress(button, x - cfg::CANVAS_X, y - cfg::CANVAS_Y);
    } else {
        g_dragSlider = -1;
        if (button == g_button) canvasRelease();
    }
    glutPostRedisplay();
}

static void onMotion(int x, int y) {
    g_mouseX = x;
    g_mouseY = y;
    if (g_dragSlider >= 0) setSliderFromX(g_dragSlider, x);
    else canvasDrag(x - cfg::CANVAS_X, y - cfg::CANVAS_Y);
    glutPostRedisplay();
}

static void onPassiveMotion(int x, int y) {
    g_mouseX = x;
    g_mouseY = y;
    g_hoverButton = buttonAt(x, y);
    if (g_shapeActive) renderShape(x - cfg::CANVAS_X, y - cfg::CANVAS_Y, inCanvas(x, y));
    glutSetCursor(inCanvas(x, y) ? GLUT_CURSOR_CROSSHAIR : GLUT_CURSOR_LEFT_ARROW);
    glutPostRedisplay();
}

static void onWheel(int, int dir, int, int) {
    changeSize(dir > 0 ? 1 : -1);
    glutPostRedisplay();
}

static void onKeyboard(unsigned char k, int, int) {
    if (glutGetModifiers() & GLUT_ACTIVE_CTRL) {
        switch (k) {
            case 26: actionUndo(); break;   // Ctrl+Z
            case 25: actionRedo(); break;   // Ctrl+Y
            case 19: actionSave(); break;   // Ctrl+S
            case 15: actionOpen(); break;   // Ctrl+O
            case 14: actionClear(); break;  // Ctrl+N
            case 3: actionCopy(); break;    // Ctrl+C
            case 24: actionCut(); break;    // Ctrl+X
            case 22: actionPaste(); break;  // Ctrl+V
            case 9: actionInsertImage(); break;  // Ctrl+I
            default: break;
        }
        glutPostRedisplay();
        return;
    }
    switch (std::tolower(k)) {
        case 'p': setTool(Tool::Pencil); break;
        case 'b': setTool(Tool::Brush); break;
        case 'e': setTool(Tool::Eraser); break;
        case 'l': setTool(Tool::Line); break;
        case 'r': setTool(Tool::Rectangle); break;
        case 'o': setTool(Tool::Ellipse); break;
        case 'c': setTool(Tool::Circle); break;
        case 'f': setTool(Tool::Fill); break;
        case 's': setTool(Tool::Select); break;
        case 'i': setTool(Tool::Picker); break;
        case 'a': setTool(Tool::Spray); break;
        case 'v': setTool(Tool::Bezier); break;
        case 'g': setTool(Tool::Polygon); break;
        case 'm': g_filled = !g_filled; break;
        case '+':
        case '=': changeSize(+1); break;
        case '-':
        case '_': changeSize(-1); break;
        case ',': transformRotate(-5); break;
        case '.': transformRotate(+5); break;
        case '[': transformScale(1 / 1.05); break;
        case ']': transformScale(1.05); break;
        case 13:  // Enter
            if (g_shapeActive) finishShape();
            else commitSelection();
            break;
        case 27:  // Esc
            cancelShape();
            cancelSelection();
            break;
        case 127: actionClear(); break;     // Delete
        default: break;
    }
    glutPostRedisplay();
}

static void onSpecial(int key, int, int) {
    switch (key) {
        case GLUT_KEY_LEFT: nudgeSelection(-1, 0); break;
        case GLUT_KEY_RIGHT: nudgeSelection(+1, 0); break;
        case GLUT_KEY_UP: nudgeSelection(0, -1); break;
        case GLUT_KEY_DOWN: nudgeSelection(0, +1); break;
        default: break;
    }
    glutPostRedisplay();
}

// ============================================================================
//  11. Entry point
// ============================================================================
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(cfg::WIN_W, cfg::WIN_H);
    glutInitWindowPosition(80, 40);
    glutCreateWindow("Mini Paint - CSE Computer Graphics Lab");

    buildUI();
    g_base = g_canvas;

    glutDisplayFunc(onDisplay);
    glutReshapeFunc(onReshape);
    glutMouseFunc(onMouse);
    glutMotionFunc(onMotion);
    glutPassiveMotionFunc(onPassiveMotion);
    glutMouseWheelFunc(onWheel);
    glutKeyboardFunc(onKeyboard);
    glutSpecialFunc(onSpecial);
    glutMainLoop();
    return 0;
}