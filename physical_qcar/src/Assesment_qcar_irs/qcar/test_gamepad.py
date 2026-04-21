#!/usr/bin/env python3
import struct

JSDEV = "/dev/input/js0"
EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

print(f"Escuchando {JSDEV} ... presiona botones o mueve los sticks")

with open(JSDEV, "rb") as js:
    while True:
        evbuf = js.read(EVENT_SIZE)
        if evbuf:
            time, value, type, number = struct.unpack(EVENT_FORMAT, evbuf)
            
            if type & 0x01:  # Botón
                print(f"Botón {number} {'presionado' if value else 'liberado'}")
            if type & 0x02:  # Eje
                print(f"Eje {number} valor={value}")

