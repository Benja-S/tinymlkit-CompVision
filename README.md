# Tracker de Movimiento — Arduino Nano 33 BLE Rev2 + OV7675
**Proyecto de Visión por Computadora | Clase de IA**

---

## Descripción

Detecta movimiento en tiempo real usando la cámara OV7675 del **Arduino Tiny Machine Learning Shield**
montado sobre un **Arduino Nano 33 BLE Rev2**. Calcula la velocidad de un objeto en movimiento
(diseñado para trackear una persona en scooter eléctrico) y lo visualiza en PC con pygame,
incluyendo imagen en vivo de la cámara con overlay de bounding box y trayectoria.

**Técnica:** Diferencia de fotogramas (*frame differencing*) + filtro de blob por BFS + suavizado exponencial.

---

## Hardware

| Componente | Modelo |
|---|---|
| Microcontrolador | Arduino Nano 33 BLE **Rev2** |
| Cámara | OV7675 (incluida en el TinyML Shield) |
| Shield | Arduino Tiny Machine Learning Shield |

### Conexiones

**Si usas el TinyML Shield:** simplemente encaja el Nano y la cámara en sus slots.
El shield hace todas las conexiones internamente. No hay cables que hacer.

**Si conectas la cámara directamente** (sin shield), la librería espera estos pines fijos
del Nano 33 BLE — no se pueden cambiar sin modificar la librería:

| OV7675 | Nano 33 BLE |
|---|---|
| 3.3V | 3.3V |
| GND | GND |
| SIOC | A5 (SCL) |
| SIOD | A4 (SDA) |
| VSYNC | 8 |
| HREF | A1 |
| PCLK | A0 |
| XCLK | 9 |
| D0–D7 | 10, 1, 0, 2, 3, 5, 6, 4 |
| RESET | 3.3V |
| PWDN | GND |

---

## Archivos

| Archivo | Descripción |
|---|---|
| `src/main.cpp` | Firmware Arduino: captura, procesamiento, envío Serial |
| `visualizador.py` | Visualizador Python: imagen en vivo + tracking + velocidad |
| `camtest.cpp` | Test de cámara: mide FPS real y muestra imagen cruda |
| `camtest_viewer.py` | Visor para camtest: diagnóstico de imagen y latencia |

---

## Dependencias

### PlatformIO (`platformio.ini`)

```ini
[env:nano33ble]
platform = nordicnrf52
board = nano33ble
framework = arduino
monitor_speed = 115200
lib_deps =
    harvard-tlx/Harvard_TinyMLx @ ^1.1.0-Alpha
```

**Notas importantes:**
- La librería oficial `Arduino_OV767X` (v0.0.2 en PIO) **no funciona** con el Nano 33 BLE Rev2
  — `Camera.begin()` se cuelga indefinidamente.
- `Harvard_TinyMLx` es la librería del curso de TinyML de Harvard/Arduino,
  con soporte nativo para el TinyML Shield y el Rev2.
- El include cambia a `#include <TinyMLShield.h>` y `Camera.begin()` requiere 4 argumentos:
  `Camera.begin(QQVGA, GRAYSCALE, 5, OV7675)`

### Python

```bash
pip install pygame pyserial numpy
```

---

## Uso

1. Abre el proyecto en VSCode con PlatformIO
2. Asegúrate de que `src/` contiene solo `main.cpp`
3. **Upload** (no usar "Upload and Monitor" — pueden pelear por el puerto)
4. Espera "SUCCESS", luego abre el monitor o corre el visualizador
5. Ejecuta el visualizador:
   ```bash
   python visualizador.py
   ```
6. Ajusta el slider **Distancia (m)** a la distancia real entre la cámara y el objeto

### Quirks del Nano 33 BLE Rev2 con PlatformIO

- El monitor de PIO no muestra output si no tiene `monitor_speed = 115200` en `platformio.ini`
- "Upload and Monitor" a veces falla — hacer Upload primero, luego abrir monitor por separado
- Si el monitor sigue sin mostrar nada, usar el script de diagnóstico Python directamente
- Si PIO parece compilar código viejo: borrar la carpeta `.pio/` y hacer clean antes de upload
- El archivo siempre debe llamarse `main.cpp` dentro de `src/` — PIO lo requiere como entry point
- Agregar `#include <Arduino.h>` al inicio y prototipos de funciones antes de `setup()` —
  el IDE de Arduino los generaba automáticamente, PIO no

---

## Protocolo Serial

El Arduino envía un paquete por fotograma a **921600 bps**:

```
[0xFF 0xAA 0xBB] [JSON\n] [19200 bytes imagen grayscale]
```

El JSON contiene:
```json
{"cx":82.3,"cy":61.1,"area":420,"spd":14.2,"fps":2.5,
 "motion":1,"bx":70,"by":50,"bw":30,"bh":25,"frame":143}
```

| Campo | Descripción |
|---|---|
| `cx`, `cy` | Centroide del blob de movimiento (píxeles) |
| `area` | Área del blob en píxeles² |
| `spd` | Velocidad en píxeles/segundo |
| `fps` | FPS promedio de la cámara (ventana de 6 frames) |
| `motion` | 1 si hay movimiento, 0 si no |
| `bx`,`by`,`bw`,`bh` | Bounding box del objeto |
| `frame` | Contador de fotogramas |

**Baud rate:** se probaron 115200, 500000, y 2000000 bps.
921600 es el punto óptimo — 2Mbaud genera demasiadas corrupciones en Windows con este chip.

---

## Cómo funciona

```
[OV7675]
   ↓ fotograma QQVGA (160×120) grayscale — ~350ms por frame (~2.5 FPS)
[frame differencing]
   ↓ resta píxel a píxel vs fotograma anterior
[grilla de celdas 10×7]
   ↓ pooling: cada celda de 16×16px resume si su región tiene movimiento
   ↓ diff y conteo de celdas en un solo loop (optimización)
[BFS — blob más grande]
   ↓ descarta ruido aislado, conserva el objeto principal
[centroide + velocidad]
   ↓ posición suavizada con filtro exponencial (α=0.6)
   ↓ velocidad capped a MAX_SPEED_MS para descartar saltos de ruido
[Serial 921600 bps → Python]
   ↓ imagen + JSON → visualizador pygame en tiempo real
```

---

## Parámetros ajustables

### Arduino (`main.cpp`)

| Parámetro | Default | Efecto |
|---|---|---|
| `DIFF_THRESHOLD` | 18 | Sensibilidad al movimiento. Bajar = más sensible, más ruido |
| `MIN_MOTION_AREA` | 60 | Píxeles mínimos para reportar movimiento |
| `CELL_SIZE` | 16 | Tamaño de celdas. Bajar = más resolución, más lento |
| `CELL_THRESHOLD` | 4 | Píxeles activos mínimos para activar una celda |

### Python (`visualizador.py`)

| Parámetro | Default | Efecto |
|---|---|---|
| `SMOOTH_ALPHA` | 0.6 | Suavizado del centroide (0=sin suavizado, 0.9=muy suavizado) |
| `MAX_SPEED_MS` | 15.0 | Velocidad máxima creíble en m/s — valores mayores se descartan |
| `TRAIL_DURACION` | 2.5 | Segundos que dura el rastro de trayectoria |
| `BAUD_RATE` | 921600 | Debe coincidir con `Serial.begin()` en el Arduino |

---

## Limitaciones conocidas

- **FPS:** ~2.5 FPS real. El límite es el hardware — el nRF52840 no tiene periférico
  de captura de imagen y la librería lee los píxeles por GPIO (~350ms por frame).
- **Velocidad:** es una estimación. Requiere calibrar la distancia con el slider.
  El ángulo de la cámara respecto al trayecto afecta la precisión.
- **Iluminación:** la OV7675 necesita buena luz para auto-exposición correcta.
  Con poca luz la imagen es muy oscura y el detector falla.
- **Fondo en movimiento:** sombras o variaciones de luz generan falsos positivos.
  Subir `DIFF_THRESHOLD` en ese caso.