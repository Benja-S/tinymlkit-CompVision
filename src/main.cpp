/*
  ============================================================
  main.cpp — Motion Tracker con imagen
  Arduino Nano 33 BLE Rev2 + OV7675 (TinyML Shield)
  ============================================================

  Usa la librería Harvard_TinyMLx que da ~2-3 FPS reales.
  Envía por Serial:
    - Header de sincronía (3 bytes)
    - Datos de tracking JSON compacto (longitud variable + '\n')
    - Píxeles de imagen (19200 bytes)

  Python reconstruye imagen + overlay de centroide/bbox en vivo.

  PROTOCOLO:
    [0xFF 0xAA 0xBB] [JSON\n] [19200 bytes imagen]
*/

#include <Arduino.h>
#include <TinyMLShield.h>

// ════════════════════════════════════════════════════════════
// PROTOTIPOS
// ════════════════════════════════════════════════════════════
void calcularCeldasYDiff();
int  encontrarBlobMasGrande(int &outSumCx, int &outSumCy,
                             int &outMinCx, int &outMaxCx,
                             int &outMinCy, int &outMaxCy);

// ════════════════════════════════════════════════════════════
// RESOLUCIÓN Y PARÁMETROS
// ════════════════════════════════════════════════════════════
#define FRAME_W  160
#define FRAME_H  120
#define FRAME_PX (FRAME_W * FRAME_H)

// Umbral de diferencia de brillo (0-255)
// Subir si hay demasiado ruido, bajar si no detecta movimiento
#define DIFF_THRESHOLD   18

// Mínimo de píxeles activos para reportar movimiento
#define MIN_MOTION_AREA  60

// Celdas de 16×16 → grilla de 10×7 = 70 celdas
// BFS rápido, suficiente resolución para un scooter
#define CELL_SIZE       16
#define CELL_COLS       (FRAME_W / CELL_SIZE)   // 10
#define CELL_ROWS       (FRAME_H / CELL_SIZE)   // 7
#define CELL_THRESHOLD   4

// ════════════════════════════════════════════════════════════
// BUFFERS
// ════════════════════════════════════════════════════════════
static uint8_t frameCurrent[FRAME_PX];
static uint8_t framePrev[FRAME_PX];
static uint8_t cellGrid[CELL_ROWS][CELL_COLS];

// ════════════════════════════════════════════════════════════
// ESTADO
// ════════════════════════════════════════════════════════════
float         prevCx = -1.0f, prevCy = -1.0f;
unsigned long prevFrameTime = 0;
unsigned long frameCount    = 0;

#define FPS_WINDOW 6
unsigned long frameDts[FPS_WINDOW];
uint8_t       fpsIdx = 0;

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(921600);
  delay(2000);

  Serial.println("{\"status\":\"iniciando\"}");

  // 4 argumentos requeridos por Harvard_TinyMLx
  if (!Camera.begin(QQVGA, GRAYSCALE, 5, OV7675)) {
    Serial.println("{\"status\":\"error\"}");
    while (1);
  }

  Serial.println("{\"status\":\"lista\"}");

  Camera.readFrame(framePrev);
  prevFrameTime = millis();
  memset(frameDts, 0, sizeof(frameDts));
}

// ════════════════════════════════════════════════════════════
// calcularCeldasYDiff
// Diff de píxeles + conteo de celdas en un solo loop
// ════════════════════════════════════════════════════════════
void calcularCeldasYDiff() {
  memset(cellGrid, 0, sizeof(cellGrid));

  for (int y = 0; y < FRAME_H; y++) {
    int cellRow = y / CELL_SIZE;
    for (int x = 0; x < FRAME_W; x++) {
      int idx  = y * FRAME_W + x;
      int diff = (int)frameCurrent[idx] - (int)framePrev[idx];
      if (diff < 0) diff = -diff;
      if (diff > DIFF_THRESHOLD) {
        int cellCol = x / CELL_SIZE;
        if (cellGrid[cellRow][cellCol] < 255)
          cellGrid[cellRow][cellCol]++;
      }
    }
  }

  // Convertir conteos a binario
  for (int r = 0; r < CELL_ROWS; r++)
    for (int c = 0; c < CELL_COLS; c++)
      cellGrid[r][c] = (cellGrid[r][c] >= CELL_THRESHOLD) ? 1 : 0;
}

// ════════════════════════════════════════════════════════════
// encontrarBlobMasGrande — BFS sobre grilla 10×7
// ════════════════════════════════════════════════════════════
int encontrarBlobMasGrande(int &outSumCx, int &outSumCy,
                            int &outMinCx, int &outMaxCx,
                            int &outMinCy, int &outMaxCy) {
  static uint8_t visited[CELL_ROWS][CELL_COLS];
  memset(visited, 0, sizeof(visited));

  static uint8_t qCx[CELL_ROWS * CELL_COLS];
  static uint8_t qCy[CELL_ROWS * CELL_COLS];

  int bestArea = 0;
  int bestSumCx = 0, bestSumCy = 0;
  int bestMinCx = CELL_COLS, bestMaxCx = 0;
  int bestMinCy = CELL_ROWS, bestMaxCy = 0;

  const int8_t dx[] = { 1, -1,  0, 0 };
  const int8_t dy[] = { 0,  0,  1,-1 };

  for (int sy = 0; sy < CELL_ROWS; sy++) {
    for (int sx = 0; sx < CELL_COLS; sx++) {
      if (!cellGrid[sy][sx] || visited[sy][sx]) continue;

      int head = 0, tail = 0;
      int area = 0, sumCx = 0, sumCy = 0;
      int minCx = sx, maxCx = sx, minCy = sy, maxCy = sy;

      qCx[tail] = sx; qCy[tail] = sy; tail++;
      visited[sy][sx] = 1;

      while (head < tail) {
        int cx = qCx[head], cy = qCy[head]; head++;
        area++; sumCx += cx; sumCy += cy;
        if (cx < minCx) minCx = cx;
        if (cx > maxCx) maxCx = cx;
        if (cy < minCy) minCy = cy;
        if (cy > maxCy) maxCy = cy;

        for (int d = 0; d < 4; d++) {
          int nx = cx + dx[d], ny = cy + dy[d];
          if (nx < 0 || nx >= CELL_COLS || ny < 0 || ny >= CELL_ROWS) continue;
          if (!cellGrid[ny][nx] || visited[ny][nx]) continue;
          visited[ny][nx] = 1;
          qCx[tail] = nx; qCy[tail] = ny; tail++;
        }
      }

      if (area > bestArea) {
        bestArea  = area;
        bestSumCx = sumCx; bestSumCy = sumCy;
        bestMinCx = minCx; bestMaxCx = maxCx;
        bestMinCy = minCy; bestMaxCy = maxCy;
      }
    }
  }

  outSumCx = bestSumCx; outSumCy = bestSumCy;
  outMinCx = bestMinCx; outMaxCx = bestMaxCx;
  outMinCy = bestMinCy; outMaxCy = bestMaxCy;
  return bestArea;
}

// ════════════════════════════════════════════════════════════
// LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  // 1. Capturar fotograma
  Camera.readFrame(frameCurrent);

  unsigned long t1 = millis();
  unsigned long dt = t1 - prevFrameTime;
  prevFrameTime = t1;

  // 2. FPS
  frameDts[fpsIdx] = dt;
  fpsIdx = (fpsIdx + 1) % FPS_WINDOW;
  unsigned long sumDt = 0; uint8_t vf = 0;
  for (int i = 0; i < FPS_WINDOW; i++)
    if (frameDts[i] > 0) { sumDt += frameDts[i]; vf++; }
  float fps = (vf > 0 && sumDt > 0) ? (vf * 1000.0f / sumDt) : 0.0f;

  // 3. Diff + celdas
  calcularCeldasYDiff();

  // 4. BFS
  int sumCx, sumCy, minCx, maxCx, minCy, maxCy;
  int blobArea = encontrarBlobMasGrande(sumCx, sumCy, minCx, maxCx, minCy, maxCy);
  long pixelArea = (long)blobArea * CELL_SIZE * CELL_SIZE;

  // 5. Centroide y velocidad
  float cx = -1.0f, cy = -1.0f, speed = 0.0f;
  bool motion = (pixelArea >= MIN_MOTION_AREA);

  if (motion) {
    cx = ((float)sumCx / blobArea) * CELL_SIZE + CELL_SIZE * 0.5f;
    cy = ((float)sumCy / blobArea) * CELL_SIZE + CELL_SIZE * 0.5f;
    if (prevCx >= 0 && dt > 0) {
      float ddx = cx - prevCx, ddy = cy - prevCy;
      speed = sqrtf(ddx*ddx + ddy*ddy) / (dt / 1000.0f);
    }
    prevCx = cx; prevCy = cy;
  } else {
    speed = 0.0f;
  }

  frameCount++;

  // 6. Enviar paquete:
  //    [0xFF 0xAA 0xBB] [JSON\n] [19200 bytes]
  //
  // Python busca el header, lee hasta '\n' para el JSON,
  // luego lee exactamente 19200 bytes de imagen.

  int bbX = minCx * CELL_SIZE;
  int bbY = minCy * CELL_SIZE;
  int bbW = (maxCx - minCx + 1) * CELL_SIZE;
  int bbH = (maxCy - minCy + 1) * CELL_SIZE;

  // Header
  Serial.write(0xFF); Serial.write(0xAA); Serial.write(0xBB);

  // JSON en una línea
  Serial.print("{");
  Serial.print("\"cx\":"); Serial.print(cx, 1);         Serial.print(",");
  Serial.print("\"cy\":"); Serial.print(cy, 1);         Serial.print(",");
  Serial.print("\"area\":"); Serial.print(pixelArea);   Serial.print(",");
  Serial.print("\"spd\":"); Serial.print(speed, 1);     Serial.print(",");
  Serial.print("\"fps\":"); Serial.print(fps, 1);       Serial.print(",");
  Serial.print("\"motion\":"); Serial.print(motion?1:0);Serial.print(",");
  Serial.print("\"bx\":"); Serial.print(motion?bbX:0);  Serial.print(",");
  Serial.print("\"by\":"); Serial.print(motion?bbY:0);  Serial.print(",");
  Serial.print("\"bw\":"); Serial.print(motion?bbW:0);  Serial.print(",");
  Serial.print("\"bh\":"); Serial.print(motion?bbH:0);  Serial.print(",");
  Serial.print("\"frame\":"); Serial.print(frameCount);
  Serial.println("}");  // '\n' marca fin del JSON

  // Imagen cruda (19200 bytes)
  Serial.write(frameCurrent, FRAME_PX);

  // 7. Swap buffers
  memcpy(framePrev, frameCurrent, FRAME_PX);
}