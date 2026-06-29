# Tracker de Movimiento — Arduino Nano 33 BLE + OV7675
**Proyecto de Visión por Computadora | Clase de IA**

---

## Descripción

Este proyecto detecta movimiento en tiempo real usando una cámara OV7675
conectada a un Arduino Nano 33 BLE. Calcula la velocidad de un objeto
(en este caso, una persona en scooter eléctrico) y la visualiza en
una pantalla de PC con pygame.

**Técnica principal:** Diferencia de fotogramas (*frame differencing*) +
filtro de blob por BFS.

---

## Archivos

| Archivo | Descripción |
|---|---|
| `motion_tracker.ino` | Sketch de Arduino: captura, procesamiento, envío Serial |
| `visualizador.py` | Visualizador Python: recibe datos y los muestra en pantalla |

---

## Requisitos

### Arduino
- Placa: **Arduino Nano 33 BLE**
- Cámara: **OV7675**
- Librería: `Arduino_OV767X`
  - Instalar desde: *Sketch → Include Library → Manage Libraries → buscar "Arduino_OV767X"*

### Python
```
pip install pygame pyserial
```
- Python 3.8 o superior

---

## Configuración de hardware

```
OV7675  3.3V  → Nano 3.3V
OV7675  GND   → Nano GND
OV7675  SIOC  → A5  (SCL)
OV7675  SIOD  → A4  (SDA)
OV7675  VSYNC → 8
OV7675  HREF  → A1
OV7675  PCLK  → A0
OV7675  XCLK  → 9
OV7675  D7–D0 → 4, 6, 5, 3, 2, 1, 0, 10
OV7675  RESET → 3.3V (siempre en alto)
OV7675  PWDN  → GND  (siempre en bajo)
```

---

## Cómo usar

1. Conectar el Arduino por USB a la PC
2. Abrir `motion_tracker.ino` en el Arduino IDE
3. Subir el sketch al Arduino (**no abrir el Monitor Serial** — Python lo usará)
4. Ejecutar el visualizador:
   ```
   python visualizador.py
   ```
5. Si no detecta el puerto automáticamente, editar esta línea en `visualizador.py`:
   ```python
   PUERTO_SERIAL = "COM3"        # Windows
   PUERTO_SERIAL = "/dev/ttyACM0"  # Linux
   PUERTO_SERIAL = "/dev/cu.usbmodem1101"  # macOS
   ```

---

## Calibración de velocidad

La velocidad en px/s que calcula el Arduino se convierte a m/s usando
la geometría del campo visual de la cámara:

```
ancho_real = 2 × distancia × tan(FOV/2)
vel_real = vel_px × (ancho_real / 160)
```

donde `FOV = 55°` (OV7675 horizontal).

**Procedimiento:**
1. Mide la distancia real entre la cámara y el trayecto del scooter
2. Ajusta el slider "Distancia (m)" en la interfaz a ese valor
3. La conversión se aplica en tiempo real

---

## Cómo funciona (resumen técnico)

```
[OV7675]
   ↓ fotograma QQVGA (160×120) en escala de grises
[frame differencing]
   ↓ mapa de píxeles que cambiaron respecto al fotograma anterior
[grilla de celdas 20×15]
   ↓ pooling: cada celda resume si su región tiene movimiento
[BFS — blob más grande]
   ↓ filtramos ruido, nos quedamos con el objeto principal
[centroide + velocidad]
   ↓ posición X,Y del objeto y velocidad en px/s
[Serial JSON → Python]
   ↓ visualizador pygame en tiempo real
```

---

## Parámetros ajustables (en el .ino)

| Parámetro | Valor por defecto | Efecto |
|---|---|---|
| `DIFF_THRESHOLD` | 20 | Sensibilidad del detector. Bajar = más sensible |
| `MIN_MOTION_AREA` | 80 | Mínimo de px activos para reportar movimiento |
| `CELL_SIZE` | 8 | Tamaño de celdas del filtro de blob |
| `CELL_THRESHOLD` | 4 | Px activos mínimos para activar una celda |

---

## Limitaciones conocidas

- La velocidad es una **estimación** — el ángulo de la cámara y la
  distancia real afectan la precisión.
- Si el scooter sale del cuadro y vuelve a entrar, el primer frame
  después de reentrar tendrá velocidad artificialmente alta (el centroide
  "saltó"). Se puede filtrar con un umbral de velocidad máxima razonable.
- El fondo en movimiento (viento, sombras) puede generar falsos positivos.
  Subir `DIFF_THRESHOLD` o `MIN_MOTION_AREA` en ese caso.
