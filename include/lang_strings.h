/* Message strings for multiple EU languages */
/* Use the LANG enumerated values to index the correct string */

const char *szImageTooBig[] = 
{
"The image file from this URL is too large.\n%s\nPNG images can be a maximum of\n%d bytes each and 1 or 2-bpp",
"La imagen de esta URL es demasiado grande.\n%s\nLas imágenes PNG pueden tener um máximo de\n%d bytes cada una y 1 o 2-bpp",
};

const char *szFirmwareAvailable[] = 
{
"Firmware update available! Starting now...",
"¡Actualización de firmware disponible!\n¡Empezando ahora!"
};

const char *szFirmwareSuccess[] = 
{
"Firmware update success. Device will restart..",
"Actualización de firmware exitosa. El dispositivo se reiniciará...",
};

const char *szFirmwareFailure[] =
{
"Firmware update failed. Device will restart...",
"Error en la actualización del firmware.\nEl dispositivo se reiniciará..."
};

const char *szWiFiWeak[] = 
{
"WiFi connected but signal is weak",
"WiFi conectado pero la señal es débil"
};

const char *szAPISizeError[] = 
{
"WiFi connected, TRMNL content malformed.\nWait or reset by holding button on back.",
"WiFi conectado, contenido TRMNL mal formado.\nEspere o reinicie manteniendo presionado en el botón de atrás."
};

const char *szAPIError[] = 
{
"WiFi connected, TRMNL not responding.\nShort click the button on back,\notherwise check your internet.",
"WiFi conectado, TRMNL no responde.\nHaga clic brevemente en el botón de atrás;\nde lo contrario, verifique su conexión a Internet."
};

const char *szWiFiInternalError[] = 
{
"WiFi connected, but API connection cannot be\nestablished. Try to refresh,or scan QR Code for help.",
"Hay conexión WiFi, pero no se puede establecer la conexión API.\nIntente actualizar o escanear el código QR para obtener ayuda."
};

const char *szWiFiFailed[] =
{
"Can't establish WiFi connection. Hold button on the back\nto reset WiFi, or scan QR Code for help.",
"No se puede establecer la conexión WiFi. Mantenga presionado\nel botón de atrás para restablecer la conexión\no escanee el código QR para obtener ayuda."
};

const char *szWiFiConnect[] =
{
"Connect to TRMNL WiFi\non your phone or computer",
"Conéctese a TRMNL WiFi\nen su teléfono o computadora",
};

const char *szImageDecodeError[] =
{
"An error occurred decoding the image from this URL.\n%s\nPlease ensure that the image is PNG and\nencoded as a compatible size and bit depth",
"Sucedio un error al decodificar la imagen desde esta URL.\n%s\nAsegúrese de que la imagen sea PNG y esté codificada\ncon un tamaño y una profundidad de bits compatibles",
};

const char *szFormatError[] = 
{
"The image file from this URL is incompatible.\n%s\nPNG images with 1 or 2 bits per pixel\nare the only currently supported format",
"El archivo de imagen de esta URL es incompatible\n%s\nLas imágenes PNG con 1 o 2 bits por píxel\nson el único formato compatible actualmente"
};