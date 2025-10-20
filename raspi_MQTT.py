#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi ↔ MQTT (roles separados, con servo silencioso usando pigpio)
- IR: SOLO ESCRITURA → publica 'IR' (0|1) al topic MQTT.
- Servo: SOLO LECTURA → escucha 'angle' (0|90) en MQTT y mueve el servo.
- Mantiene opciones de --hold, trims, etc.
"""

import os
import time
import argparse
from datetime import datetime
import json
import sys

# --- GPIO para IR ---
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

# --- pigpio para SERVO (microsegundos, estable) ---
try:
    import pigpio
    HAS_PIGPIO = True
except Exception:
    HAS_PIGPIO = False

# --- MQTT (paho) ---
try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except Exception:
    HAS_MQTT = False

# ===========================
# CONFIGURACIÓN MQTT / TOPICS
# ===========================
MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_KEEPALIVE = 60
MQTT_USER = os.environ.get("MQTT_USER", None)
MQTT_PASS = os.environ.get("MQTT_PASS", None)
USE_TLS = os.environ.get("MQTT_TLS", "0") == "1"

# Cada dispositivo usa su ID para topics
DEVICE_ID = os.environ.get("DEVICE_ID", "raspi-01")
TOPIC_BASE = f"devices/{DEVICE_ID}"
TOPIC_IR = f"{TOPIC_BASE}/IR"        # publica "0" o "1"
TOPIC_ANGLE = f"{TOPIC_BASE}/angle"  # recibe "0" o "90"

# LWT (Last Will) topic and payload
LWT_TOPIC = f"{TOPIC_BASE}/status"
LWT_PAYLOAD = "offline"
ONLINE_PAYLOAD = "online"

# IR (digital)
PIN_IR = 17  # BCM 17

# Servo (señal) en BCM 18 (pin físico 12). pigpio controla por µs.
PIN_SERVO = 18

# Anchos de pulso recomendados (ajustables). La mayoría de servos aceptan 500–2500 µs.
PULSE_0_US  = 500    # pulso para 0°
PULSE_90_US = 1500   # pulso para 90° (aprox mitad del recorrido)

# Tiempo de espera para que el servo llegue a posición antes de soltar (si no se usa --hold)
SETTLE_S = 0.45

# ===========================
# HELPERS GPIO / IR
# ===========================
def gpio_setup():
    if not HAS_GPIO:
        raise RuntimeError("RPi.GPIO no disponible. Instálalo o evita modos con GPIO.")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

def ir_setup():
    gpio_setup()
    GPIO.setup(PIN_IR, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def read_ir_level(active_high=False) -> int:
    level = GPIO.input(PIN_IR)
    if active_high:
        return 1 if level == GPIO.HIGH else 0
    else:
        return 1 if level == GPIO.LOW else 0

# ===========================
# SERVO con pigpio
# ===========================
def pigpio_connect():
    if not HAS_PIGPIO:
        raise RuntimeError("No encuentro 'pigpio'. Instala: sudo apt-get install -y pigpio python3-pigpio")
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("El daemon pigpio no está corriendo. Ejecuta: sudo systemctl start pigpiod")
    return pi

def servo_set_pulse_us(pi, pulse_us: int):
    pulse_us = max(400, min(2600, int(pulse_us)))
    pi.set_servo_pulsewidth(PIN_SERVO, pulse_us)

def servo_to_angle(pi, angle: int, hold: bool = False, trim0_us: int = 0, trim90_us: int = 0):
    if angle == 0:
        pulse = PULSE_0_US + int(trim0_us)
    elif angle == 90:
        pulse = PULSE_90_US + int(trim90_us)
    else:
        print(f"[WARN] ángulo inválido recibido ({angle}). Se ignora.")
        return
    servo_set_pulse_us(pi, pulse)
    time.sleep(SETTLE_S)
    if hold:
        print(f"[{datetime.now()}] Servo a {angle}° (hold ON, pulso={pulse} µs)")
    else:
        pi.set_servo_pulsewidth(PIN_SERVO, 0)
        print(f"[{datetime.now()}] Servo a {angle}° (hold OFF, liberado)")

# ===========================
# MQTT WRAPPERS
# ===========================
class MQTTClient:
    def __init__(self, host, port, user=None, passwd=None, use_tls=False, client_id=None):
        if not HAS_MQTT:
            raise RuntimeError("paho-mqtt no está instalado. `pip install paho-mqtt`")
        self.client = mqtt.Client(client_id=client_id)
        if user and passwd:
            self.client.username_pw_set(user, passwd)
        if use_tls:
            self.client.tls_set()  # para TLS simple; para certs personalizados ajustar
        # Set callbacks externally: on_connect, on_message
        # Set LWT (offline)
        self.client.will_set(LWT_TOPIC, payload=LWT_PAYLOAD, qos=1, retain=True)
        self._connected = False

    def connect(self, host, port, keepalive=60):
        self.client.connect(host, port, keepalive)
        # start loop in background
        self.client.loop_start()

    def publish(self, topic, payload, qos=1, retain=False):
        return self.client.publish(topic, payload=str(payload), qos=qos, retain=retain)

    def subscribe(self, topic, qos=1):
        return self.client.subscribe(topic, qos=qos)

    def set_callbacks(self, on_connect=None, on_message=None, on_disconnect=None):
        if on_connect:
            self.client.on_connect = on_connect
        if on_message:
            self.client.on_message = on_message
        if on_disconnect:
            self.client.on_disconnect = on_disconnect

    def disconnect(self):
        try:
            self.client.publish(LWT_TOPIC, ONLINE_PAYLOAD, qos=1, retain=True)
        except Exception:
            pass
        self.client.loop_stop()
        self.client.disconnect()

# ===========================
# MODOS: loop_publish_ir / watch_servo (via MQTT)
# ===========================
def loop_publish_ir(mqtt_client: MQTTClient, poll_ms=50, active_high=False, retain=True):
    if not HAS_GPIO:
        raise RuntimeError("RPi.GPIO no disponible. Instálalo o evita --loop-ir.")
    ir_setup()
    prev = None
    print("Loop IR iniciado (solo publicación MQTT). Ctrl+C para salir.")
    try:
        while True:
            ir = read_ir_level(active_high=active_high)
            if ir != prev:
                mqtt_client.publish(TOPIC_IR, payload=str(ir), qos=1, retain=retain)
                prev = ir
                print(f"[{datetime.now()}] Publicado IR -> {ir}")
            time.sleep(poll_ms / 1000.0)
    except KeyboardInterrupt:
        print("Saliendo loop IR…")
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass

def watch_servo(mqtt_client: MQTTClient, hold=False, trim0_us=0, trim90_us=0):
    pi = pigpio_connect()

    last_angle = None

    def on_message(client, userdata, msg):
        nonlocal last_angle
        try:
            payload = msg.payload.decode().strip()
            # Acepta JSON {"angle":90} o payload plano "90"
            angle = None
            try:
                if payload.startswith("{"):
                    j = json.loads(payload)
                    angle = int(j.get("angle"))
                else:
                    angle = int(payload)
            except Exception:
                print(f"[WARN] Payload no parseable en {msg.topic}: '{payload}'")
                return

            if angle in (0, 90) and angle != last_angle:
                servo_to_angle(pi, angle, hold=hold, trim0_us=trim0_us, trim90_us=trim90_us)
                last_angle = angle
            else:
                if angle is not None and angle not in (0, 90):
                    print(f"[WARN] Valor 'angle' no permitido: {angle}. Ignorado.")
        except Exception as e:
            print("[ERR] on_message:", e)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"[{datetime.now()}] Conectado al broker MQTT. Suscribiendo a {TOPIC_ANGLE}")
            mqtt_client.subscribe(TOPIC_ANGLE, qos=1)
            # publicar status online (retain)
            mqtt_client.publish(LWT_TOPIC, ONLINE_PAYLOAD, qos=1, retain=True)
        else:
            print("[ERR] Conexión MQTT fallida, rc=", rc)

    mqtt_client.set_callbacks(on_connect=on_connect, on_message=on_message)
    print("Escuchando cambios de 'angle' vía MQTT (solo lectura)… Ctrl+C para salir.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Saliendo watch servo…")
    finally:
        try:
            pi.set_servo_pulsewidth(PIN_SERVO, 0)
            pi.stop()
        except Exception:
            pass
        if HAS_GPIO:
            try:
                GPIO.cleanup()
            except Exception:
                pass

# ===========================
# CLI
# ===========================
def main():
    parser = argparse.ArgumentParser(description="IR solo pub / Servo solo sub via MQTT (pigpio silencioso)")
    parser.add_argument("--loop-ir", action="store_true", help="Publica IR (0/1) al cambiar (requiere GPIO)")
    parser.add_argument("--active-high", action="store_true", help="Marca si tu IR es activo en ALTO (HIGH=1)")
    parser.add_argument("--watch-servo", action="store_true", help="Escucha 'angle' por MQTT y mueve el servo (solo lectura)")
    parser.add_argument("--hold", action="store_true", help="Mantener pulsos (par de retención). Si no se usa, se libera el servo (cero zumbido).")
    parser.add_argument("--trim0", type=int, default=0, help="Ajuste fino µs para 0° (ej. 5, -5)")
    parser.add_argument("--trim90", type=int, default=0, help="Ajuste fino µs para 90° (ej. 8, -6)")
    parser.add_argument("--broker", type=str, default=MQTT_HOST, help="Host del broker MQTT")
    parser.add_argument("--port", type=int, default=MQTT_PORT, help="Puerto del broker MQTT")
    parser.add_argument("--user", type=str, default=None, help="Usuario MQTT (si aplica)")
    parser.add_argument("--password", type=str, default=None, help="Password MQTT (si aplica)")
    parser.add_argument("--client-id", type=str, default=f"raspi-{os.getpid()}", help="Client ID MQTT")
    parser.add_argument("--retain-ir", action="store_true", help="Publicar IR con retain (útil para reconexiones)")
    args = parser.parse_args()

    if not HAS_MQTT:
        print("ERROR: paho-mqtt no instalado. Ejecuta: pip install paho-mqtt")
        sys.exit(1)

    mqtt_client = MQTTClient(host=args.broker, port=args.port, user=args.user or MQTT_USER,
                             passwd=args.password or MQTT_PASS, use_tls=USE_TLS, client_id=args.client_id)
    mqtt_client.connect(args.broker, args.port, keepalive=MQTT_KEEPALIVE)

    ran = False
    try:
        if args.loop_ir:
            loop_publish_ir(mqtt_client, active_high=args.active_high, retain=args.retain_ir)
            ran = True
        if args.watch_servo:
            # Se registran callbacks dentro de watch_servo
            watch_servo(mqtt_client, hold=args.hold, trim0_us=args.trim0, trim90_us=args.trim90)
            ran = True
        if not ran:
            parser.print_help()
    finally:
        try:
            mqtt_client.disconnect()
        except Exception:
            pass
        if HAS_GPIO:
            try:
                GPIO.cleanup()
            except Exception:
                pass

if __name__ == "__main__":
    main()
