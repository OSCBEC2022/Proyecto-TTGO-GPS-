# Proyecto TTGO GPS + MPU + LoRaWAN

Sistema IoT basado en TTGO LoRa32 (ESP32) que envía datos de aceleración y ubicación vía LoRaWAN (TTN).

## Hardware
- TTGO LoRa32 T3 V1.6
- GPS NEO-6M
- MPU6050

## Pines utilizados
- GPS TX → GPIO39 (UART1 RX)
- MPU6050 SDA → GPIO21
- MPU6050 SCL → GPIO22
- LoRa:
  - NSS → GPIO18
  - RST → GPIO14
  - DIO0 → GPIO26
  - DIO1 → GPIO33
  - DIO2 → GPIO32

## Librerías
- LMIC (LoRaWAN)
- NMEAGPS
- Adafruit MPU6050
- Adafruit Sensor
- SPI / Wire

## Configuración
- Arduino IDE: 2.3.7
- Frecuencia: AU915 (Chile)
- Intervalo de envío: 30 s
- Baudrate GPS: 9600

## Datos enviados
- Aceleración: AX, AY, AZ
- Latitud y Longitud (GPS)
- Timestamp (desde GPS)

## Decoder TTN

```javascript
function decodeUplink(input) {
  var bytes = input.bytes;

  if (bytes.length !== 14) {
    return {
      errors: ["Payload inválido: se esperaban 14 bytes y llegaron " + bytes.length]
    };
  }

  function toInt16(msb, lsb) {
    let value = (msb << 8) | lsb;
    if (value & 0x8000) value -= 0x10000;
    return value;
  }

  function toInt32(b0, b1, b2, b3) {
    let value = ((b0 << 24) >>> 0) | (b1 << 16) | (b2 << 8) | b3;
    if (value > 0x7FFFFFFF) value -= 0x100000000;
    return value;
  }

  var ax = toInt16(bytes[0], bytes[1]);
  var ay = toInt16(bytes[2], bytes[3]);
  var az = toInt16(bytes[4], bytes[5]);

  var lat_i = toInt32(bytes[6], bytes[7], bytes[8], bytes[9]);
  var lon_i = toInt32(bytes[10], bytes[11], bytes[12], bytes[13]);

  return {
    data: {
      ax: ax,
      ay: ay,
      az: az,
      lat: lat_i / 100000.0,
      lon: lon_i / 100000.0
    }
  };
}
