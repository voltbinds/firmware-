[README.md](https://github.com/user-attachments/files/28595572/README.md)
# VoltBinds Firmware

Lokale, cloud-freie Kopplung von Heimspeichersystemen auf Basis des ESP32-C3.

**Aktueller Stand: v1.4.1**

---

## Was ist VoltBinds?

VoltBinds verbindet einen unabhaengigen Stromzaehler mit einem oder mehreren
Batteriespeichern und regelt Laden/Entladen automatisch - ohne Cloud, ohne
Abo, ohne Internet.

Das Geraet laeuft vollstaendig lokal im Heimnetz und ist ueber den Browser
erreichbar.

---

## Unterstuetzte Geraete

### Smartmeter (Stromzaehler)
| Geraet | Protokoll | Status |
|---|---|---|
| Sonnenbatterie (integrierter Zaehler) | REST API lokal | Unterstuetzt |
| Shelly 3EM / Pro 3EM | REST API lokal | Unterstuetzt |

### Speicher
| Geraet | Protokoll | Status |
|---|---|---|
| Marstek Venus C/E (V1/V2) | Modbus TCP | Unterstuetzt |

---

## Hardware

| Komponente | Details |
|---|---|
| Board | ESP32-C3 mit OLED (0.42", SSD1306 72x40px) |
| Display-Pins | SDA=GPIO5, SCL=GPIO6 |
| Flash | 4MB |

---

## Features

- **Lokale Regellogik** - laeuft vollstaendig ohne Internet
- **Captive Portal** - Erstkonfiguration per WLAN-Setup ohne App
- **WiFi-Scan** - verfuegbare Netze werden automatisch angezeigt
- **Dashboard** - erreichbar unter `http://voltbinds.local`
- **OLED Display** - 3 rotierende Screens (Energie, SOC, Status)
- **OTA Updates** - Firmware-Update ueber WLAN (ArduinoOTA)
- **Modbus TCP** - direkte Kommunikation mit Marstek Venus
- **Tag/Nacht-Erkennung** - automatische Umschaltung der Regellogik
- **Soft-Rampe** - sanftes An- und Abfahren der Speicherleistung
- **Manuel Override** - direkte Leistungsvorgabe pro Geraet im Dashboard

---

## Installation

### Voraussetzungen

- Arduino IDE 2.x oder PlatformIO
- Board-Support: `esp32` by Espressif (Version 3.x)

### Libraries (Arduino Library Manager)

```
U8g2          by olikraus
ArduinoJson   by Benoit Blanchon  (v7.x)
```

### Board-Einstellungen (Arduino IDE)

```
Board:            ESP32C3 Dev Module
USB CDC On Boot:  Enabled
Flash Size:       4MB
Upload Speed:     460800
```

### Erster Flash

1. ESP32-C3 per USB-C anschliessen
2. In Arduino IDE den richtigen Port waehlen (NICHT debug-console)
3. Falls kein Port erscheint: BOOT-Taste halten, RESET kurz druecken, BOOT loslassen
4. Sketch hochladen

### OTA (ab dem zweiten Update)

```
Hostname:  voltbinds
Passwort:  voltbinds2026
```

Oder direkt ueber `http://voltbinds.local` im Browser (ab v1.5).

---

## Erstkonfiguration

1. Nach dem Flash oeffnet VoltBinds den WLAN Access Point **`VoltBinds-Setup`**
2. Damit verbinden - Browser oeffnet automatisch die Setup-Seite
3. WLAN-Netzwerk aus dem Scan-Dropdown waehlen
4. Sonnenbatterie-IP + Token eingeben
5. Marstek-IP(s) eingeben
6. Speichern - ESP startet neu und ist unter `http://voltbinds.local` erreichbar

---

## Architektur

```
IPowermeter  <-- SonnenAdapter   (REST API, Status + Steuerung)
             <-- ShellyAdapter   (REST API, nur Messwerte)

IStorage     <-- MarstekAdapter  (Modbus TCP)

RuleEngine   arbeitet ausschliesslich ueber die Interfaces
```

Die Adapter-Architektur erlaubt spaetere Erweiterungen (neue Speicher,
neue Zaehler) ohne die Regellogik anzufassen.

---

## Regellogik

**Tag-Modus** (PV > 150W stabil):
- Sonnenbatterie laedt zuerst (Prioritaet)
- Sobald SB-Ladeleistung >= Schwellwert oder SB voll: Marstek mit PV-Ueberschuss laden
- Bei Netzbezug trotz vollem SB: Marstek unterstuetzt

**Nacht-Modus**:
- Marstek entlaedt anteilig basierend auf gegleattetem Verbrauch (5-Werte-Puffer)
- Sonnenbatterie entlaedt auf Rest-Bedarf
- Soft-Stop wenn SOC nahe Minimum

---

## Lizenz

GPL v3 - siehe [LICENSE](LICENSE)

---

## Projekt-Links

- Website: [voltbinds.com](https://voltbinds.com)
- Dokumentation: [docs.voltbinds.com](https://docs.voltbinds.com)
