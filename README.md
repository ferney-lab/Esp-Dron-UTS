# ESP-DRONE UTS: Rutinas de Vuelo Automatizadas con ESP32-S2 y MPU6050

<p align="center">
  <img src="assets/app_interfaz.png" alt="ESP-DRONE UTS Panel Principal" width="550"/>
</p>

Repositorio oficial del proyecto de grado para optar al título de **Tecnólogo en Gestión de Sistemas de Telecomunicaciones** en las **Unidades Tecnológicas de Santander (UTS)**.

---

## 👥 Autores y Dirección
* **Desarrolladores:** 
  * Jhon Erick Jesús Ramírez Ortega (C.C. 1098824785)
  * Ferney Jair Esparza Castro (C.C. 1095300127)
* **Directora:** Ing. Vivian Paola Triana Galeano
* **Codirector:** Ing. Johan Leandro Téllez Garzón
* **Grupo de Investigación:** GNET — Unidades Tecnológicas de Santander (UTS)

---

## 📌 Descripción del Proyecto
Este proyecto implementa y valida un sistema embebido de monocontrol para un minidrón cuadricóptero educativo basado en el chip **ESP32-S2-MINI-2** y la unidad inercial (IMU) **MPU6050**. Se diseñaron rutinas de vuelo autónomas secuenciales (despegue gradual, sustentación estacionaria/hover, traslación coordinada y aterrizaje suave con corte automático) comandadas y monitoreadas de forma inalámbrica vía WiFi.

El minidrón cuenta con un sistema de control dual:
1. **Aplicación Móvil dedicada ("ESP DRONE UTS")**: para telemetría en tiempo real y disparo de rutinas de vuelo.
2. **Acceso Web Universal Directo**: el ESP32-S2 aloja un servidor web HTTP en su memoria; al conectarse a la red WiFi del dron, cualquier dispositivo (móvil, tablet o PC) puede abrir su navegador e ingresar a la dirección IP **`http://192.168.43.42`** para acceder al panel HTML de control completo, telemetría y parada de emergencia sin instalar ninguna aplicación adicional.

---

## 🎥 Videos Demostrativos en Vuelo (YouTube Shorts)

| Rutina | Maniobra de Vuelo | Video Demostrativo |
| :--- | :--- | :---: |
| **Rutina 1** | Despegue vertical con rampa de empuje, hover estacionario estabilizado y descenso suave con corte de seguridad | [Ver en YouTube](https://youtube.com/shorts/9VxdEOls3PE) |
| **Rutina 2** | Elevación progresiva, sustentación prolongada y compensación activa de actitud ante perturbaciones | [Ver en YouTube](https://youtube.com/shorts/wIUHFMTDS0Y) |
| **Rutina 3** | Trayectoria coordinada completa y control de descenso en área delimitada | [Ver en YouTube](https://youtube.com/shorts/wp47Jiy3K9M) |

---

## 🔨 Construcción y Ensamble del Dron (Capítulo 4 del Informe)

En el Capítulo 4 (*"Desarrollo del Trabajo de Grado"*) se detalla el proceso físico de manufactura, integración electrónica y ensamblaje del hardware del prototipo **ESP32-S2 Drone V1.2**:

<p align="center">
  <img src="assets/minidron_ensamblado.png" alt="Minidron ESP32-S2 Drone V1.2 Ensamblado" width="450"/>
</p>

### 1. Arquitectura de Chasis y Disposición en "X"
El dron utiliza una estructura de cuadricóptero con disposición en **"X"**, lo que proporciona un balance simétrico de empuje entre los ejes de alabeo (Roll) y cabeceo (Pitch). 

### 2. Soldadura y Polarización de los Motores
La correcta conexión de los cuatro motores coreless (brushed) a la tarjeta PCB principal es un factor crítico para el despegue estable:
* **M1 (Delantero Derecho):** Giro Antihorario (CCW) — Cables negro y blanco.
* **M2 (Trasero Derecho):** Giro Horario (CW) — Cables azul y rojo.
* **M3 (Trasero Izquierdo):** Giro Antihorario (CCW) — Cables negro y blanco.
* **M4 (Delantero Izquierdo):** Giro Horario (CW) — Cables azul y rojo.

<p align="center">
  <img src="assets/soldadura_motores.png" alt="Orden de colores para soldadura de motores" width="450"/>
</p>

> ⚠️ *Advertencia técnica:* Si se invierte la polaridad de un motor o se suelda en la posición equivocada, el motor girará en sentido inverso, generando empuje descendente o impidiendo el equilibrio del par aerodinámico de sustentación.

### 3. Orientación y Dirección de Giro de las Hélices
Para contrarrestar el torque de arrastre (*Drag Torque*) y evitar la rotación descontrolada sobre el eje de guiñada (Yaw), se colocan dos hélices con paso horario (CW) y dos con paso antihorario (CCW) según el sentido de rotación de cada motor:

<p align="center">
  <img src="assets/giro_helices.png" alt="Dirección de giro de hélices" width="450"/>
</p>

### 4. Alimentación y Sistema de Carga
* **Batería:** Batería LiPo de celda única (1S 3.7 V, 300-500 mAh) con tasa de descarga mínima de 25C para suministrar los picos de corriente demandados por los 4 motores simultáneamente.
* **Módulo de Carga:** Integración de módulo cargador micro USB basado en el circuito integrado **TP4056** con protección contra sobrecorriente y sobredescarga.

---

## 🌐 Conexión Inalámbrica y Control Dual

El microcontrolador ESP32-S2 genera de forma autónoma una red inalámbrica local:
* **SSID (Red WiFi):** `ESP-DRONE-UTS`
* **Contraseña:** `12345678`
* **Dirección IP Fija del Servidor Web Embebido:** `192.168.43.42`

### Métodos de Acceso:
1. **Acceso Web mediante Navegador (Cualquier Dispositivo):**  
   Una vez conectado a la red WiFi del dron, abre tu navegador web (Google Chrome, Edge, Safari, Firefox) y escribe en la barra de direcciones:
   ```
   http://192.168.43.42
   ```
   El servidor embebido servirá de forma instantánea el panel HTML5/JavaScript responsivo sin necesidad de conexión a Internet ni instalaciones previas. Desde allí puedes iniciar rutinas, calibrar trims y activar la **Parada de Emergencia**.

2. **Aplicación Móvil Especializada ("ESP DRONE UTS"):**  
   Desarrollada para smartphones, ofrece comandos táctiles dedicados y telemetría de actitud angular (Roll, Pitch, Yaw).

<p align="center">
  <img src="assets/app_guia_funcional.png" alt="Distribución Funcional de la Aplicación" width="550"/>
</p>

---

## 🛠️ Arquitectura del Firmware Embebido (`esp_drone_firmware.ino`)
* **Monocontrol Embebido:** Centralización del lazo de actitud, muestreo I²C a 1 kHz y servidor HTTP en el chip mononúcleo Xtensa LX7 (240 MHz).
* **Fusión Sensorial Inercial:** Filtro complementario / Mahony a 250 Hz ($T = 4\,\text{ms}$) para estimar ángulos estables y mitigar la deriva del giroscopio.
* **Control PID en Cascada:** Dual-Loop Controller (Lazo de Ángulo exterior y Lazo de Velocidad angular interior).
* **Seguridad Activa y Watchdog:** Corte de motores por umbral de inclinación excesiva (*Tilt Cut* a 50° en vuelo libre y 22° durante la fase de aterrizaje).

---

## 📖 Manual Técnico-Operativo (Anexo B)
El manual formal del proyecto se encuentra disponible en la carpeta [`manual/`](manual/):
* 📘 [Ver Manual en PDF](manual/Manual_Tecnico_MiniDron_ESP32.pdf)
* 📝 [Ver Manual en Word](manual/Manual_Tecnico_MiniDron_ESP32.docx)
* 📑 [Guía resumida en Markdown](manual/README.md)

<p align="center">
  <img src="assets/portada_manual.png" alt="Portada del Manual Técnico" width="380"/>
</p>

---

## 📂 Estructura del Repositorio

```bash
esp-drone-uts/
├── assets/                 # Fotos del informe, esquemas de motores y diagramas
├── firmware/               # Código fuente para ESP32-S2 (Arduino IDE / C++)
│   └── esp_drone_firmware.ino
├── manual/                 # Manual técnico-operativo de transferencia (PDF y DOCX)
│   ├── Manual_Tecnico_MiniDron_ESP32.pdf
│   ├── Manual_Tecnico_MiniDron_ESP32.docx
│   └── README.md
└── web_interface/          # Panel de control web embebido HTTP
    └── control_drone.html
```

---

## 📄 Licencia y Créditos
Proyecto desarrollado con fines investigativos y pedagógicos en el marco del grupo de investigación **GNET** de las **Unidades Tecnológicas de Santander (UTS)**, Bucaramanga, Colombia.
