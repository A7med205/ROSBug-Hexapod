[Project home](../README.md) · [Documentation index](README.md)

# Hardware design

## CAD

The editable mechanical model is available in the
[ROSBug Hexapod Onshape document](https://cad.onshape.com/documents/2a9acbb79b23a0da46d8b498/w/10c7141c859961b97a14543a/e/498922900873c82099a5cf37).
Print-ready meshes are included in [`cad/stl`](../cad/stl).

![Complete ROSBug Hexapod CAD assembly](full_cad.png)

## Electronics

```mermaid
graph LR
    Battery-->|7.4V|Connector
    Battery-->|GND|Connector

    Battery-->|3 PIN|Alarm[Voltage<br>Alarm]

    Connector-->|"7.4V<br>(solder)"|Buck[Buck<br>Converter]
    Connector-->|"GND<br>(solder)"|Buck
    Connector-->|"7.4V<br>(solder)"|UBEC
    Connector-->|"GND<br>(solder)"|UBEC

    Buck-->|"5V<br>(MicroUSB)"|RPi[-<br>RPi<br>-]
    Buck-->|"GND<br>(MicroUSB)"|RPi
    RPi-->|USB-C|Servo2040[-<br>Servo 2040<br>-]
    RPi-->CAM

    UBEC-->|6V|Servo2040
    UBEC-->|GND|Servo2040
    Servo2040-->|6V|Servos[18× servo]
    Servo2040-->|GND|Servos
    Servo2040-->|Signal|Servos
```

Parts, printed components, fasteners, and tools are listed in the
[bill of materials](bill-of-materials.md).
