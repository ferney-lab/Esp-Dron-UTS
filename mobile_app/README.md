# 📱 Aplicación Móvil ESP-DRONE UTS

En esta carpeta se encuentra la aplicación móvil oficial **ESP DRONE UTS** para el control inalámbrico, monitoreo de telemetría y ejecución de rutinas autónomas del minidrón ESP32-S2.

---

## 📦 Archivos Disponibles
* ⬇️ **[Descargar ESP_DRONE_UTS_v1.0.apk (Enlace Directo)](https://github.com/ferney-lab/Esp-Dron-UTS/raw/main/mobile_app/ESP_DRONE_UTS_v1.0.apk)** (6.99 MB)

> ℹ️ **Nota si abres el archivo en la web de GitHub:**  
> Si haces clic en el archivo `.apk` dentro de la lista de archivos de GitHub, la pantalla mostrará el mensaje: *"Disculpen las molestias, pero en este momento no podemos mostrar archivos de este tamaño"*. Esto ocurre porque GitHub no puede previsualizar archivos binarios de más de 1 MB. Para descargarlo a tu celular o PC, simplemente pulsa el botón **`Download`** (o **`View raw`**) que aparece en esa misma pantalla, o usa el enlace directo de arriba.

---

## 🚀 Guía de Instalación

### 🤖 1. Instalación en Dispositivos Android (Archivo .APK)
Los teléfonos Android permiten instalar paquetes directamente mediante su archivo `.apk`. Siga estos pasos:

1. **Descargar el archivo:**
   * Descargue el archivo **`ESP_DRONE_UTS_v1.0.apk`** directamente desde este repositorio a la memoria de su teléfono.
2. **Habilitar fuentes desconocidas:**
   * Al abrir el archivo descargado, si el sistema lo solicita, autorice la instalación desde fuentes externas:
     * *Ajustes* ➔ *Seguridad / Privacidad* ➔ *Instalar aplicaciones desconocidas* (o marque *Permitir desde esta fuente* en el navegador o administrador de archivos utilizado).
3. **Completar la instalación:**
   * Presione **Instalar** y espere a que finalice el proceso.
4. **Conexión con el dron:**
   * Encienda el dron y espere unos segundos mientras calibra en reposo.
   * En su celular, abra los ajustes de Wi-Fi y conéctese a la red del dron:
     * **SSID:** `ESP-DRONE_XXXX` (o el nombre emitido según la dirección MAC)
     * **Contraseña:** `12345678`
   * Si Android muestra el aviso *"Esta red no tiene acceso a internet"*, seleccione **Mantener conexión**.
5. **Iniciar la aplicación:**
   * Abra la aplicación **ESP-DRONE UTS** y comience la operación.

---

### 🍏 2. Instalación en Dispositivos Apple iOS / iPhone (Modo PWA)

> ⚠️ **¿Por qué no se puede instalar un archivo .apk en iPhone?**  
> Los archivos con extensión `.apk` (*Android Package Kit*) están compilados en código y librerías nativas diseñadas **únicamente para Android**. El sistema operativo de Apple (iOS) es una plataforma cerrada con otra arquitectura de software y solo permite instalar apps firmadas desde la *App Store* (paquetes `.ipa`). Por esta razón, si intenta abrir un `.apk` en un iPhone, el sistema no lo reconocerá y mostrará una pantalla negra de datos sin opción de ejecución.

#### ✅ Solución Oficial para iPhone: Progressive Web App (PWA)
Para que los usuarios de iPhone puedan tener exactamente la misma experiencia de una app nativa, el firmware del dron incorpora una **PWA completa** servida directamente desde la memoria interna del microcontrolador ESP32-S2. Para instalarla como app en la pantalla de inicio de su iPhone:

1. **Conectarse a la red del dron:**
   * Encienda el dron sobre una superficie plana.
   * En su iPhone, vaya a **Ajustes ➔ Wi-Fi** y seleccione la red:
     * **SSID:** `ESP-DRONE_XXXX`
     * **Contraseña:** `12345678`
   * Si iOS indica *"Sin conexión a internet"*, pulse sobre la red y asegúrese de que permanezca conectada.
2. **Abrir Safari:**
   * Abra el navegador **Safari** (es fundamental utilizar Safari, ya que iOS gestiona la instalación de PWAs a través de este navegador).
   * En la barra de direcciones escriba:  
     👉 **`http://192.168.43.42`** y pulse *Ir*.
3. **Añadir a la Pantalla de Inicio:**
   * En la barra inferior de Safari, toque el botón **Compartir** (el ícono del cuadrado con la flecha apuntando hacia arriba 📤).
   * Desplace el menú hacia abajo y seleccione la opción **"Agregar a pantalla de inicio"** (o *Add to Home Screen*).
   * El sistema le sugerirá el nombre de la app (puede dejar **ESP-DRONE UTS**) y pulse **"Agregar"** en la esquina superior derecha.
4. **Uso como App Nativa:**
   * En la pantalla principal de su iPhone aparecerá el ícono de **ESP-DRONE UTS**.
   * Al tocarlo, la aplicación se abrirá en **pantalla completa** (sin marcos ni barras de Safari), con guardado en caché local, controles táctiles optimizados y respuesta en tiempo real idéntica a una aplicación nativa descargada de la App Store.

