#include "BlobWatchy.h"

const float VOLTAGE_MIN = 3.4;
const float VOLTAGE_MAX = 4.2;
const float VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;

const int DENSITY = 2;
const int VECTOR_SIZE = 60 * DENSITY;
const double STEP_ANGLE = 360 / VECTOR_SIZE;

const int STEP_MINUTE = DENSITY;
const int STEP_HOUR = VECTOR_SIZE/12;

const Vector CENTER = {100,100};
const int RADIUS = 99;
const int BLOB_RADIUS = 90;

const int HOUR_INDENT_DISTANCE = 10;
const double HOUR_INDENT_STRENGTH = 0.2;

const int MINUTE_INDENT_DISTANCE = 4;
const double MINUTE_INDENT_STRENGTH = 0.35;

const double BATTERY_MIN = 0.35;
const double BATTERY_WARNING = 0.4;
const double BATTERY_RANGE = 1.0 - BATTERY_MIN;

const double HOUR_TICK = 1 - 0.1;
const double MINUTE_TICK = 1 - 0.3;

const unsigned char DITHER_MASK [] PROGMEM = {
	0, 32, 8, 40, 2, 34, 10, 42,
  48, 46, 56, 24, 50, 18, 58, 26,
  12, 44, 4, 36, 14, 46, 6, 38,
  60, 28, 52, 20, 62, 30, 54, 22,
	3, 35, 11, 43, 1, 33, 9, 41,
  51, 19, 59, 27, 49, 17, 57, 25,
  15, 47, 7, 39, 13, 45, 5, 37,
  63, 31, 55, 23, 61, 29, 53, 21
};

Vector EDGE_VECTORS[VECTOR_SIZE];
Vector EDGE_NORMAL[VECTOR_SIZE];

BlobWatchy::BlobWatchy(const watchySettings& s) : Watchy(s)
{
  Vector up = {0.0, -1.0};
  
  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    EDGE_NORMAL[i] = EDGE_VECTORS[i] = Vector::rotateVector(up, i * STEP_ANGLE);
    EDGE_NORMAL[i].normalize();
    EDGE_VECTORS[i].normalize();
  }
}

static double smoothstep(double x) {
  // Evaluate polynomial
  return x * x * (3 - 2 * x);
}

static void Indent(int distance, int centerIndex, double amount, double scale[], Vector normals[])
{
  for (int i = -distance; i <= distance; i++)
  {
    int index = (i + centerIndex + VECTOR_SIZE) % VECTOR_SIZE;
    double strength = amount * smoothstep(1 - abs(i / (double)distance));
    scale[index] -= strength;
  }

  for (int i = -distance - 1; i <= distance + 1; i++)
  {
    int minuteToIndex = i + centerIndex;
    int index = (minuteToIndex + VECTOR_SIZE) % VECTOR_SIZE;
    int prevIndex = (minuteToIndex - 1 + VECTOR_SIZE) % VECTOR_SIZE;
    int nextIndex = (minuteToIndex + 1 + VECTOR_SIZE) % VECTOR_SIZE;

    Vector v0 = EDGE_VECTORS[prevIndex] * scale[prevIndex];
    Vector v1 = EDGE_VECTORS[index] * scale[index];
    Vector v2 = EDGE_VECTORS[nextIndex] * scale[nextIndex];

    Vector normal1 = v0 - v1;
    normal1.normalize();
    normal1 = Vector::rotateVectorByRightAngle(normal1, 1);

    Vector normal2 = v1 - v2;
    normal2.normalize();
    normal2 = Vector::rotateVectorByRightAngle(normal2, 1);

    Vector normal = normal1 + normal2;
    normal.normalize();

    normals[index] = normal;
  }
}

void BlobWatchy::drawWatchFace()
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  double scale[VECTOR_SIZE];
  Vector normals[VECTOR_SIZE];

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    scale[i] = 1.0;
    normals[i] = EDGE_NORMAL[i];
  }

  int hour = currentTime.Hour;
  int minute = currentTime.Minute;
  
  int minuteIndex = minute * STEP_MINUTE;
  int hourIndex = round(((double)(hour % 12) + minute/60.0) * STEP_HOUR);

  Indent(MINUTE_INDENT_DISTANCE, minuteIndex, MINUTE_INDENT_STRENGTH, scale, normals);

  Indent(HOUR_INDENT_DISTANCE, hourIndex, HOUR_INDENT_STRENGTH, scale, normals);

  double batteryFill = getBatteryFill();
  double batteryFillScale = BATTERY_MIN +BATTERY_RANGE * batteryFill;

  for (int i = 0; i < VECTOR_SIZE; i += STEP_MINUTE)
  {
    Vector v1 = EDGE_VECTORS[i] * (RADIUS) + CENTER;
    Vector v2 = EDGE_VECTORS[i] * (RADIUS * MINUTE_TICK) + CENTER;

    display.drawLine(v1.x, v1.y, v2.x, v2.y, GxEPD_BLACK);
  }

  for (int i = 0; i < VECTOR_SIZE; i += STEP_HOUR)
  {
    Vector v1 = EDGE_VECTORS[i] * (RADIUS) + CENTER;
    Vector v2 = EDGE_VECTORS[i] * (RADIUS * HOUR_TICK) + CENTER;

    display.drawLine(v1.x + 1, v1.y, v2.x + 1, v2.y, GxEPD_BLACK);
    display.drawLine(v1.x - 1, v1.y, v2.x - 1, v2.y, GxEPD_BLACK);
    display.drawLine(v1.x, v1.y + 1, v2.x, v2.y + 1, GxEPD_BLACK);
    display.drawLine(v1.x, v1.y - 1, v2.x, v2.y - 1, GxEPD_BLACK);
  }

  display.drawCircle(CENTER.x, CENTER.y, BLOB_RADIUS * BATTERY_WARNING, GxEPD_BLACK);

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    int nextIndex = (i + 1) % VECTOR_SIZE;
    Vector v1 = EDGE_VECTORS[i] * (BLOB_RADIUS * scale[i] * batteryFillScale) + CENTER;
    Vector uv1 = normals[i] * RADIUS + CENTER;
    Vector v2 = EDGE_VECTORS[nextIndex] * (BLOB_RADIUS * scale[nextIndex] * batteryFillScale) + CENTER;
    Vector uv2 = normals[nextIndex] * RADIUS + CENTER;

    fillTriangle(CENTER, CENTER, v1, uv1, v2, uv2, MatCapSource, 200, 200);
    display.drawLine(v1.x, v1.y, v2.x, v2.y, GxEPD_BLACK);
  }
}

double BlobWatchy::getBatteryFill()
{
  float VBAT = getBatteryVoltage();

  // 12 battery states
  double batState = ((VBAT - VOLTAGE_MIN) / VOLTAGE_RANGE);
  if (batState > 1.0)
    batState = 1.0;
  if (batState < 0)
    batState = 0;

  return batState;
}

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#ifndef _swap_vector
#define _swap_vector(a, b)                                                    \
  {                                                                            \
    Vector t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#ifndef _swap_vector_int
#define _swap_vector_int(a, b)                                                    \
  {                                                                            \
    VectorInt t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

static void barycentric(VectorInt p, VectorInt a, VectorInt b, VectorInt c, double &u, double &v, double &w)
{
    VectorInt v0 = b - a, v1 = c - a, v2 = p - a;
    float den = v0.x * v1.y - v1.x * v0.y;
    v = (v2.x * v1.y - v1.x * v2.y) / den;
    w = (v0.x * v2.y - v2.x * v0.y) / den;
    u = 1.0f - v - w;
}

static double clamp(double val, double min, double max)
{
  if (val > max)
    val = max;
  
  if (val < min)
    val = min;

  return val;
}

static void barycentric2(VectorInt p, VectorInt v0, VectorInt v1, VectorInt a, double invDen, double &u, double &v, double &w)
{
    VectorInt v2 = p - a;
    v = (v2.x * v1.y - v1.x * v2.y) * invDen;
    w = (v0.x * v2.y - v2.x * v0.y) * invDen;
    u = 1.0f - v - w;
}


static bool getColor(int16_t x, int16_t y, int16_t xUv, int16_t yUv, const uint8_t *bitmap, int16_t w, int16_t h) 
{
  return bitmap[yUv * w + xUv] > DITHER_MASK[(y % 8) * 8 + x % 8] * 4;
}

void BlobWatchy::drawLine(int x, int y, int w, VectorInt v0, Vector uv0, VectorInt v1, Vector uv1, VectorInt v2, Vector uv2, const uint8_t *bitmap, int16_t bw, int16_t bh)
{
  VectorInt a = v1 - v0, b = v2 - v0;
  double den = a.x * b.y - b.x * a.y;
  double invDen = 1 / den;

  for (int i = 0; i < w; i++)
  {
    double ua, va, wa;
    VectorInt pointA = {x + i, y};
    barycentric2(pointA, a, b, v0, invDen, ua, va, wa);

    Vector uv = uv0 * ua + uv1 * va + uv2 * wa;

    bool white = getColor(x + i, y, uv.x, uv.y, bitmap, bw, bh);
    display.drawPixel(x + i, y, white ? GxEPD_WHITE : GxEPD_BLACK);
  }
}

void BlobWatchy::drawLine2(int x, int y, int w, VectorInt v0, Vector uv0, VectorInt a, Vector uv1, VectorInt b, Vector uv2, double invDen, const uint8_t *bitmap, int16_t bw, int16_t bh)
{
  for (int i = 0; i < w; i++)
  {
    double ua, va, wa;
    VectorInt pointA = {x + i, y};
    barycentric2(pointA, a, b, v0, invDen, ua, va, wa);

    Vector uv = uv0 * ua + uv1 * va + uv2 * wa;

    bool white = getColor(x + i, y, uv.x, uv.y, bitmap, bw, bh);
    display.drawPixel(x + i, y, white ? GxEPD_WHITE : GxEPD_BLACK);
  }
}

void BlobWatchy::fillTriangle(VectorInt v0, Vector uv0, VectorInt v1, Vector uv1, VectorInt v2, Vector uv2, const uint8_t bitmap[], int w, int h)
{
  int16_t a, b, y, last;
  Vector uvA, uvB;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (v0.y > v1.y) {
    _swap_vector_int(v0, v1);
    _swap_vector(uv0, uv1);
  }
  if (v1.y > v2.y) {
    _swap_vector_int(v2, v1);
    _swap_vector(uv2, uv1);
  }
  if (v0.y > v1.y) {
    _swap_vector_int(v0, v1);
    _swap_vector(uv0, uv1);
  }

  display.startWrite();
  if (v0.y == v2.y) { // Handle awkward all-on-same-line case as its own thing
    a = b = v0.x;
    uvA = uv0;
    uvB = uv0;

    if (v1.x < a)
    {
      a = v1.x;
      uvA = uv1;
    }
    else if (v1.x > b)
    { 
      b = v1.x;
      uvB = uv1;
    }
    if (v2.x < a)
    {
      a = v2.x;
      uvA = uv2;
    }
    else if (v2.x > b)
    {
      b = v2.x;
      uvB = uv2;
    }

    writeFastHLineUV(a, v0.y, b - a + 1, uvA, uvB, bitmap, w, h);
    display.endWrite();
    return;
  }

  int16_t dx01 = v1.x - v0.x, dy01 = v1.y - v0.y, 
          dx02 = v2.x - v0.x, dy02 = v2.y - v0.y,
          dx12 = v2.x - v1.x, dy12 = v2.y - v1.y;
  int32_t sa = 0, sb = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (v1.y == v2.y)
    last = v1.y; // Include y1 scanline
  else
    last = v1.y - 1; // Skip it

    
  VectorInt aa = v1 - v0, bb = v2 - v0;
  double invDen = 1 / VectorInt::crossProduct(aa, bb);

  for (y = v0.y; y <= last; y++) {
    a = v0.x + sa / dy01;
    b = v0.x + sb / dy02;

    sa += dx01;
    sb += dx02;

    
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if (a > b)
      _swap_int16_t(a, b);

    drawLine2(a, y, b - a + 1, v0, uv0, aa, uv1, bb, uv2, invDen, bitmap, w, h);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = (int32_t)dx12 * (y - v1.y);
  sb = (int32_t)dx02 * (y - v0.y);
  for (; y <= v2.y; y++) {
    a = v1.x + sa / dy12;
    b = v0.x + sb / dy02;

    sa += dx12;
    sb += dx02;

    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if (a > b)
      _swap_int16_t(a, b);

    drawLine2(a, y, b - a + 1, v0, uv0, aa, uv1, bb, uv2, invDen, bitmap, w, h);
  }
  display.endWrite();
}

void BlobWatchy::writeFastHLine(int16_t x, int16_t y, int16_t w,
                                 uint16_t color, bool even) {
  display.startWrite();

  for (int i = 0; i < w; i++)
  {
    if (((x + i + y) % 2 == 0) == even)
      display.drawPixel(x + i, y, color);
  }

  display.endWrite();
}

void BlobWatchy::writeFastHLineUV(int16_t x, int16_t y, int16_t w, Vector uvA, Vector uvB, const uint8_t bitmap[], int bw, int bh)
{
  display.startWrite();

  for (int i = 0; i < w; i++)
  {
    double lerpVal = i / (w + 1.0);
    Vector uv = (uvA * lerpVal) + (uvB * (1.0 - lerpVal));
    bool white = getColor(x + i, y, uv.x, uv.y, bitmap, bw, bh);
    display.drawPixel(x + i, y, white ? GxEPD_WHITE : GxEPD_BLACK);
  }

  display.endWrite();
}
