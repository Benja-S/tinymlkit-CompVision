"""
visualizador.py
===============
Visualizador de movimiento con imagen en vivo.
Recibe imagen + datos de tracking del Arduino por Serial.

PROTOCOLO ESPERADO:
    [0xFF 0xAA 0xBB] [JSON\n] [19200 bytes imagen]

INSTALACIÓN:
    pip install pygame pyserial numpy

USO:
    python visualizador.py
"""

import pygame
import serial
import serial.tools.list_ports
import numpy as np
import json
import math
import time
import sys
import collections

# ════════════════════════════════════════════════════════════
# CONFIGURACIÓN
# ════════════════════════════════════════════════════════════
PUERTO    = None
BAUD_RATE = 921600

CAM_W = 160
CAM_H = 120
ESCALA_VISOR = 4

FOV_H_GRADOS = 55.0

# ── Filtros de velocidad ──────────────────────────────────
# Suavizado exponencial del centroide antes de calcular velocidad.
# 0.0 = sin suavizado (ruidoso), 0.8 = muy suavizado (lento de responder)
SMOOTH_ALPHA = 0.6

# Velocidad máxima creíble en m/s. Valores por encima = ruido, se descartan.
# 54 km/h = 15 m/s, suficiente para un scooter eléctrico
MAX_SPEED_MS = 15.0

# Segundos que dura el rastro de trayectoria antes de desaparecer
TRAIL_DURACION = 2.5

# ════════════════════════════════════════════════════════════
# COLORES
# ════════════════════════════════════════════════════════════
COLOR_FONDO         = (10,  12,  20)
COLOR_PANEL         = (18,  22,  36)
COLOR_BORDE         = (40,  55,  90)
COLOR_TEXTO         = (200, 210, 230)
COLOR_TEXTO_DIM     = (80,  100, 130)
COLOR_ACENTO        = (0,   200, 180)
COLOR_ACENTO2       = (255, 160,  30)
COLOR_VERDE         = (50,  220,  80)
COLOR_ROJO          = (220,  60,  60)
COLOR_BBOX          = (0,   200, 180)
COLOR_CENTROIDE     = (255, 220,  50)
COLOR_TRAYECTORIA   = (60,  140, 255)
COLOR_GRAFICA_FONDO = (14,  18,  30)
COLOR_GRAFICA_LINEA = (0,   200, 180)

# ════════════════════════════════════════════════════════════
# DIMENSIONES
# ════════════════════════════════════════════════════════════
VISOR_W = CAM_W * ESCALA_VISOR
VISOR_H = CAM_H * ESCALA_VISOR
PANEL_W = 320
MARGEN  = 16
ALTO_GRAFICA = 140

VENTANA_W = VISOR_W + PANEL_W + MARGEN * 3
VENTANA_H = VISOR_H + MARGEN * 2

FRAME_PX    = CAM_W * CAM_H
HEADER      = bytes([0xFF, 0xAA, 0xBB])
HEADER_LEN  = 3
MAX_TRAIL   = 120
MAX_SPEED_H = 200   # puntos en la gráfica

# ════════════════════════════════════════════════════════════
# HELPERS
# ════════════════════════════════════════════════════════════
def detectar_puerto():
    puertos = serial.tools.list_ports.comports()
    for p in puertos:
        desc = (p.description or "").lower()
        fab  = (p.manufacturer or "").lower()
        if any(x in desc or x in fab for x in ["arduino", "bossa", "samd", "nano"]):
            print(f"  Arduino en: {p.device}")
            return p.device
    if puertos:
        print(f"  Usando: {puertos[0].device}")
        return puertos[0].device
    return None

def px_a_ms(vel_px, distancia_m):
    """Convierte velocidad en px/s a m/s usando geometría del FOV."""
    if distancia_m <= 0: return 0.0
    fov_rad = math.radians(FOV_H_GRADOS)
    ancho_real = 2.0 * distancia_m * math.tan(fov_rad / 2.0)
    return vel_px * (ancho_real / CAM_W)

# ════════════════════════════════════════════════════════════
# CLASE: Slider
# ════════════════════════════════════════════════════════════
class Slider:
    def __init__(self, x, y, ancho, val_min, val_max, val_inicial, etiqueta):
        self.rect     = pygame.Rect(x, y, ancho, 6)
        self.val_min  = val_min
        self.val_max  = val_max
        self.valor    = val_inicial
        self.etiqueta = etiqueta
        self.dragging = False

    @property
    def pos_knob(self):
        t = (self.valor - self.val_min) / (self.val_max - self.val_min)
        return int(self.rect.x + t * self.rect.width)

    def manejar_evento(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN:
            if pygame.Rect(self.pos_knob-8, self.rect.y-6, 16, 18).collidepoint(ev.pos):
                self.dragging = True
        elif ev.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif ev.type == pygame.MOUSEMOTION and self.dragging:
            t = (ev.pos[0] - self.rect.x) / self.rect.width
            self.valor = self.val_min + max(0.0, min(1.0, t)) * (self.val_max - self.val_min)

    def dibujar(self, surface, font):
        pygame.draw.rect(surface, COLOR_BORDE, self.rect, border_radius=3)
        r = pygame.Rect(self.rect.x, self.rect.y,
                        self.pos_knob - self.rect.x, self.rect.height)
        pygame.draw.rect(surface, COLOR_ACENTO, r, border_radius=3)
        pygame.draw.circle(surface, COLOR_ACENTO, (self.pos_knob, self.rect.centery), 8)
        pygame.draw.circle(surface, COLOR_FONDO,  (self.pos_knob, self.rect.centery), 5)
        txt = font.render(f"{self.etiqueta}: {self.valor:.1f} m", True, COLOR_TEXTO_DIM)
        surface.blit(txt, (self.rect.x, self.rect.y - 18))

# ════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════
def main():
    print("=" * 50)
    print("  Motion Tracker — Nano 33 BLE + OV7675")
    print("=" * 50)

    puerto = PUERTO or detectar_puerto()
    if not puerto:
        print("ERROR: no se encontró puerto serial"); sys.exit(1)

    print(f"\nConectando a {puerto} @ {BAUD_RATE} bps...")
    try:
        ser = serial.Serial(puerto, BAUD_RATE, timeout=2.0)
        time.sleep(2.0)
        ser.reset_input_buffer()
        print("  Conectado.\n")
    except serial.SerialException as e:
        print(f"ERROR: {e}"); sys.exit(1)

    pygame.init()
    ventana = pygame.display.set_mode((VENTANA_W, VENTANA_H))
    pygame.display.set_caption("Motion Tracker — Arduino OV7675")
    reloj = pygame.time.Clock()

    font_grande = pygame.font.SysFont("monospace", 32, bold=True)
    font_medio  = pygame.font.SysFont("monospace", 18, bold=True)
    font_normal = pygame.font.SysFont("monospace", 15)
    font_small  = pygame.font.SysFont("monospace", 13)

    surf_cam = pygame.Surface((CAM_W, CAM_H))

    # hist_trail guarda (timestamp, x, y) para poder expirar por tiempo
    hist_trail    = collections.deque(maxlen=MAX_TRAIL)
    hist_speed_ms = collections.deque(maxlen=MAX_SPEED_H)

    panel_x = VISOR_W + MARGEN * 2
    slider_dist = Slider(panel_x, VENTANA_H - 80, PANEL_W - MARGEN * 2,
                         0.5, 20.0, 5.0, "Distancia")

    ultimo_dato     = {}
    buf             = bytearray()
    conectado       = True
    ultimo_frame_ts = time.time()
    frames_recibidos = 0
    perdidos        = 0

    # ── Suavizado del centroide ───────────────────────────
    # Aplicamos un filtro exponencial al centroide antes de
    # calcular la velocidad — reduce enormemente el ruido.
    # cx_smooth = alpha * cx_nuevo + (1 - alpha) * cx_anterior
    cx_smooth = -1.0
    cy_smooth = -1.0
    spd_ms_smooth = 0.0   # también suavizamos la velocidad para el display

    # Parser de protocolo
    STATE_HEADER = 0
    STATE_JSON   = 1
    STATE_IMAGE  = 2
    estado       = STATE_HEADER
    json_buf     = bytearray()
    dato_pendiente = {}

    print("Esperando datos del Arduino...")

    running = True
    while running:

        # ── Eventos ───────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: running = False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE: running = False
            slider_dist.manejar_evento(ev)

        # ── Lectura serial ────────────────────────────────
        disponible = ser.in_waiting
        if disponible > 0:
            buf.extend(ser.read(disponible))

        # ── Parser de protocolo ───────────────────────────
        i = 0
        while i < len(buf):
            if estado == STATE_HEADER:
                idx = buf.find(HEADER, i)
                if idx == -1:
                    buf = buf[max(0, len(buf) - 2):]
                    i = len(buf); break
                i = idx + HEADER_LEN
                estado = STATE_JSON
                json_buf = bytearray()

            elif estado == STATE_JSON:
                while i < len(buf):
                    b = buf[i]; i += 1
                    if b == ord('\n'):
                        try:
                            dato_pendiente = json.loads(
                                json_buf.decode('utf-8', errors='ignore'))
                        except json.JSONDecodeError:
                            perdidos += 1
                            estado = STATE_HEADER; break
                        estado = STATE_IMAGE; break
                    json_buf.append(b)
                    if len(json_buf) > 512:
                        perdidos += 1
                        estado = STATE_HEADER; break

            elif estado == STATE_IMAGE:
                if len(buf) - i < FRAME_PX: break
                pixeles = bytes(buf[i : i + FRAME_PX])
                i += FRAME_PX
                estado = STATE_HEADER

                # ── Paquete completo ──────────────────────
                ultimo_dato = dato_pendiente
                ultimo_frame_ts = time.time()
                frames_recibidos += 1

                # Actualizar imagen
                try:
                    img = np.frombuffer(pixeles, dtype=np.uint8).reshape((CAM_H, CAM_W))
                    img_rgb = np.stack([img, img, img], axis=2)
                    pygame.surfarray.blit_array(surf_cam, img_rgb.swapaxes(0, 1))
                except Exception:
                    perdidos += 1

                # ── Velocidad con suavizado ───────────────
                motion = ultimo_dato.get("motion", 0)

                if motion:
                    cx_raw = ultimo_dato.get("cx", -1.0)
                    cy_raw = ultimo_dato.get("cy", -1.0)

                    if cx_raw >= 0:
                        if cx_smooth < 0:
                            # Primera detección — inicializar sin filtro
                            cx_smooth = cx_raw
                            cy_smooth = cy_raw
                        else:
                            # Filtro exponencial: suaviza posición
                            # SMOOTH_ALPHA alto = más suavizado
                            cx_smooth = SMOOTH_ALPHA * cx_smooth + (1 - SMOOTH_ALPHA) * cx_raw
                            cy_smooth = SMOOTH_ALPHA * cy_smooth + (1 - SMOOTH_ALPHA) * cy_raw

                        # Velocidad en px/s desde el Arduino (ya calculada)
                        spd_px = ultimo_dato.get("spd", 0.0)
                        spd_ms_raw = px_a_ms(spd_px, slider_dist.valor)

                        # Descartar valores imposibles (ruido de centroide)
                        if spd_ms_raw > MAX_SPEED_MS:
                            spd_ms_raw = spd_ms_smooth  # usar último valor válido

                        # Suavizar también la velocidad mostrada
                        spd_ms_smooth = (SMOOTH_ALPHA * spd_ms_smooth +
                                         (1 - SMOOTH_ALPHA) * spd_ms_raw)

                        hist_speed_ms.append(spd_ms_smooth)

                        # Rastro con timestamp
                        sx = MARGEN + int(cx_smooth * ESCALA_VISOR)
                        sy = MARGEN + int(cy_smooth * ESCALA_VISOR)
                        hist_trail.append((time.time(), sx, sy))
                else:
                    # Sin movimiento — resetear suavizado para no
                    # arrastrar posición vieja cuando vuelva a aparecer
                    cx_smooth = -1.0
                    cy_smooth = -1.0
                    spd_ms_smooth = max(0.0, spd_ms_smooth * 0.7)  # decaer suave
                    hist_speed_ms.append(spd_ms_smooth)

        if i > 0:
            buf = buf[i:]

        # Expirar puntos del rastro más viejos que TRAIL_DURACION segundos
        ahora = time.time()
        while hist_trail and ahora - hist_trail[0][0] > TRAIL_DURACION:
            hist_trail.popleft()

        conectado = (ahora - ultimo_frame_ts) < 4.0

        # ════════════════════════════════════════════════
        # DIBUJO
        # ════════════════════════════════════════════════
        ventana.fill(COLOR_FONDO)

        # ── Imagen de cámara ──────────────────────────────
        visor_rect = pygame.Rect(MARGEN, MARGEN, VISOR_W, VISOR_H)
        img_escalada = pygame.transform.scale(surf_cam, (VISOR_W, VISOR_H))
        ventana.blit(img_escalada, (MARGEN, MARGEN))
        pygame.draw.rect(ventana, COLOR_BORDE, visor_rect, 1)

        # ── Rastro con fade temporal ──────────────────────
        if len(hist_trail) > 1:
            pts = list(hist_trail)
            for idx in range(1, len(pts)):
                # Fade basado en edad: más reciente = más brillante
                edad = ahora - pts[idx][0]
                alpha = int(255 * max(0.0, 1.0 - edad / TRAIL_DURACION))
                r = int(COLOR_TRAYECTORIA[0] * alpha / 255)
                g = int(COLOR_TRAYECTORIA[1] * alpha / 255)
                b = int(COLOR_TRAYECTORIA[2] * alpha / 255)
                pygame.draw.line(ventana, (r, g, b),
                                 (pts[idx-1][1], pts[idx-1][2]),
                                 (pts[idx][1],   pts[idx][2]), 2)

        # ── Bounding box y centroide ──────────────────────
        if ultimo_dato.get("motion", 0) and ultimo_dato.get("bw", 0) > 0:
            bx = MARGEN + int(ultimo_dato["bx"] * ESCALA_VISOR)
            by = MARGEN + int(ultimo_dato["by"] * ESCALA_VISOR)
            bw = int(ultimo_dato["bw"] * ESCALA_VISOR)
            bh = int(ultimo_dato["bh"] * ESCALA_VISOR)
            pygame.draw.rect(ventana, COLOR_BBOX, pygame.Rect(bx, by, bw, bh), 2)
            # Esquinas estilo HUD
            corner = 14

            corners = [(bx, by), (bx+bw, by), (bx, by+bh), (bx+bw, by+bh)]
            dirs    = [(1,1),    (-1,1),        (1,-1),       (-1,-1)]
            for (cx2, cy2), (ddx, ddy) in zip(corners, dirs):
                pygame.draw.line(ventana, COLOR_ACENTO,
                                 (cx2, cy2), (cx2+ddx*corner, cy2), 2)
                pygame.draw.line(ventana, COLOR_ACENTO,
                                 (cx2, cy2), (cx2, cy2+ddy*corner), 2)

            # Centroide suavizado
            if cx_smooth >= 0:
                sx = MARGEN + int(cx_smooth * ESCALA_VISOR)
                sy = MARGEN + int(cy_smooth * ESCALA_VISOR)
                pygame.draw.line(ventana, COLOR_CENTROIDE, (sx-12, sy), (sx+12, sy), 2)
                pygame.draw.line(ventana, COLOR_CENTROIDE, (sx, sy-12), (sx, sy+12), 2)
                pygame.draw.circle(ventana, COLOR_CENTROIDE, (sx, sy), 5, 1)

        # ── Panel derecho ─────────────────────────────────
        panel_rect = pygame.Rect(panel_x - MARGEN//2, MARGEN,
                                  PANEL_W, VENTANA_H - MARGEN*2)
        pygame.draw.rect(ventana, COLOR_PANEL, panel_rect, border_radius=6)
        pygame.draw.rect(ventana, COLOR_BORDE, panel_rect, 1, border_radius=6)

        yp = MARGEN * 2

        def sep():
            nonlocal yp
            pygame.draw.line(ventana, COLOR_BORDE,
                             (panel_x, yp), (panel_x + PANEL_W - MARGEN, yp))
            yp += 12

        def txt(texto, font, color, dy=0):
            nonlocal yp
            ventana.blit(font.render(texto, True, color), (panel_x, yp))
            yp += dy or font.get_height() + 4

        txt("MOTION TRACKER", font_medio, COLOR_ACENTO, 30)
        sep()

        txt("● CONECTADO" if conectado else "● SIN SEÑAL",
            font_normal, COLOR_VERDE if conectado else COLOR_ROJO, 26)
        txt(f"FPS camara:  {ultimo_dato.get('fps', 0.0):.1f}",
            font_normal, COLOR_TEXTO, 22)
        txt(f"Frame: {ultimo_dato.get('frame',0)}   Perdidos: {perdidos}",
            font_small, COLOR_TEXTO_DIM, 28)
        sep()

        motion = ultimo_dato.get("motion", 0)
        txt("MOVIMIENTO DETECTADO" if motion else "sin movimiento",
            font_normal, COLOR_VERDE if motion else COLOR_TEXTO_DIM, 22)
        txt(f"Area blob: {ultimo_dato.get('area',0)} px",
            font_small, COLOR_TEXTO_DIM, 22)
        cx_v = ultimo_dato.get("cx", -1)
        cy_v = ultimo_dato.get("cy", -1)
        txt(f"Centroide: ({cx_v:.0f}, {cy_v:.0f}) px" if cx_v >= 0 else "Centroide: —",
            font_small, COLOR_TEXTO_DIM, 30)
        sep()

        # Velocidad suavizada
        spd_kmh = spd_ms_smooth * 3.6
        vel_color = COLOR_ACENTO2 if spd_ms_smooth > 5.0 else COLOR_ACENTO

        spd_px_display = ultimo_dato.get("spd", 0.0)
        txt("Velocidad (px/s):", font_small, COLOR_TEXTO_DIM, 18)
        txt(f"{spd_px_display:.0f}", font_grande, vel_color, 42)
        txt("Velocidad (m/s):", font_small, COLOR_TEXTO_DIM, 18)
        txt(f"{spd_ms_smooth:.2f}", font_grande, vel_color, 42)
        txt(f"{spd_kmh:.1f} km/h", font_medio, COLOR_TEXTO_DIM, 36)
        sep()

        # Gráfica
        txt("Velocidad — ultimos frames", font_small, COLOR_TEXTO_DIM, 18)
        graf = pygame.Rect(panel_x, yp, PANEL_W - MARGEN, ALTO_GRAFICA)
        pygame.draw.rect(ventana, COLOR_GRAFICA_FONDO, graf)
        pygame.draw.rect(ventana, COLOR_BORDE, graf, 1)
        if len(hist_speed_ms) > 1:
            max_v = max(max(hist_speed_ms), 0.1)
            pts_g = [(graf.x + int(idx * graf.width / MAX_SPEED_H),
                      graf.bottom - int((v / max_v) * (ALTO_GRAFICA - 6)))
                     for idx, v in enumerate(hist_speed_ms)]
            if len(pts_g) > 1:
                pygame.draw.lines(ventana, COLOR_GRAFICA_LINEA, False, pts_g, 2)
            ventana.blit(font_small.render(f"{max_v:.1f}", True, COLOR_TEXTO_DIM),
                         (graf.right - 38, graf.top + 2))
        yp = graf.bottom + MARGEN

        # Slider
        slider_dist.rect.x = panel_x
        slider_dist.rect.y = yp + 20
        slider_dist.dibujar(ventana, font_small)
        ventana.blit(font_small.render("Ajustar distancia = velocidad real",
                     True, COLOR_TEXTO_DIM), (panel_x, slider_dist.rect.y + 18))

        ventana.blit(font_small.render(f"UI: {reloj.get_fps():.0f} fps  ESC=salir",
                     True, COLOR_TEXTO_DIM), (MARGEN, VENTANA_H - 20))

        pygame.display.flip()
        reloj.tick(60)

    ser.close()
    pygame.quit()
    print("\nVisualizador cerrado.")

if __name__ == "__main__":
    main()