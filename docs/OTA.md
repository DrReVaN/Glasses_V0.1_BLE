# OTA für STM32WB35CE

## Speicheraufteilung

| Bereich | Adresse | Größe / Zweck |
|---|---|---|
| CPU1-Bootloader | 0x08000000–0x0800DFFF | 56 KiB, bleibt bei OTA erhalten |
| Geräteschlüssel | 0x0800E000–0x0800EFFF | 4 KiB, einmalige Zufallsschlüssel für BLE |
| Image-Metadaten | 0x0800F000–0x0800FFFF | 4 KiB, Länge, CRC32 und abschließender Gültigkeitsmarker |
| CPU1-Anwendung | 0x08010000–0x0803FFFF | maximal 192 KiB |
| Oberer Flash | ab 0x08040000 | von diesem Updater nicht beschrieben; CPU2/FUS bleiben erhalten |

Die 64-KiB-Reservierung vor der Anwendung unterscheidet sich bewusst vom kleineren ST-Beispiel. So passen Bluetooth-Bindung, Anzeige, Wiederherstellung und geprüfte Flash-Zugriffe in den Bootloader. Die Linker begrenzen beide Images. Vor jedem Flash-Zugriff wird zusätzlich die tatsächliche CPU2-Schutzgrenze **SFSA** geprüft. Vor Erstinstallation muss SFSA mindestens 0x40 sein.

Der Bootloader startet die Anwendung nur bei passendem Format, gültigem Stack-/Reset-Vektor und korrekter Prüfsumme über das gesamte Image. Das GATT-Layout ist in Bootloader und Anwendung identisch, damit vorhandene BLE-Bindungen und Service-Caches weiter verwendbar bleiben.

## Einmalige Erstinstallation mit ST-Link

1. Boardbezeichnung **STM32WB35CE mit 512 KiB Flash** prüfen. Vorhandenen Flash und Option Bytes sichern. Die vorhandene CPU2/FUS-Installation zunächst beibehalten.
2. Beide Images mit `tools/build.py` bauen und `tools/package.py` ausführen. Bei verändertem Quellcode immer neu bauen und paketieren.
3. In STM32CubeProgrammer per SWD verbinden. CPU2-Stack-Version und SFSA auslesen. Für die hier mitgelieferte Auswahl ist `STM32WB3x_FW+BT Stack/stm32wb3x_BLE_Stack_full_fw.bin`, **V1.13.0**, vorgesehen. Laut zugehörigen Release Notes liegt die Installationsadresse auf dem 512-KiB-Gerät bei **0x08053000**. Eine notwendige Stack-Installation erfolgt nach den mitgelieferten ST/FUS-Schritten; ein normales Schreiben eines fremden WB55-Binaries ersetzt diese Schritte nicht.
4. **Keinen Full-Chip-Erase durchführen.** Nur CPU1-Seiten unter 0x08040000 ändern. `build/install.hex` laden und verifizieren. Die Datei enthält Bootloader, Anwendung und Metadaten; sie enthält weder CPU2/FUS noch die Schlüssel-Seite.
5. Gerät neu starten. Auf der Brille die Uhr/BLE-Anzeige prüfen. Beim ersten BLE-Start werden individuelle Schlüssel erzeugt. Die Brille zeigt beim Pairing eine sechsstellige Zahl; nur bei Übereinstimmung mit dem Handy/PC Pad 1 eine Sekunde halten. Pad 3 lehnt ab.
6. Nach erfolgreicher Hardwareprüfung Bootloader-Seiten **0–13** über WRP1A schützen. Die Seiten 14 (Schlüssel) und 15 (Metadaten) müssen beschreibbar bleiben. Die Option-Byte-Bezeichnung und Grenzwerte vor dem Setzen am konkreten WB35 prüfen. SWD für die Entwicklung verfügbar lassen.

Bei Erstinstallation werden alte ungeschützte BLE-Verbindungen nicht automatisch vertrauenswürdig. Alte Bindungen am Handy gegebenenfalls entfernen und neu mit Zahlenvergleich koppeln. Firmware und App sollten gemeinsam aus ihren Dev-Branches verwendet werden.

## Updates über Bluetooth

Python 3.10+ und einen Bluetooth-Adapter verwenden:

```text
python -m pip install -r tools/requirements-ota.txt
python tools/ota.py build/application/smartglasses.bin --check
python tools/ota.py build/application/smartglasses.bin --address AA:BB:CC:DD:EE:FF --enter
```

Android-App während des PC-Updates trennen. Auf der Brille erscheint `OTA?`; Pad 1 eine Sekunde halten, um den Neustart in den Bootloader zu bestätigen. Pad 3 verwirft die Anfrage. Anschließend:

```text
python tools/ota.py build/application/smartglasses.bin --address AA:BB:CC:DD:EE:FF
```

Auf macOS die vom Betriebssystem bereitgestellte Geräte-UUID verwenden. Falls der PC noch nicht gebunden ist, am Gerät zuerst Pairing öffnen: Pads 1 und 3 gleichzeitig drei Sekunden halten. Den Zahlenvergleich auf beiden Geräten ausdrücklich bestätigen.

Der Client prüft Ziel, Adresse, Größe, SHA-256, CRC32 und Vektoren lokal. Jede Übertragung verwendet einen authentifizierten, verschlüsselten ATT Write Request und wartet auf die Antwort. Datenpakete enthalten den Byte-Offset; Duplikate und falsche Reihenfolge werden abgewiesen. Nach Erase/Schreiben prüft die Brille die CRC32 erneut und schreibt den Gültigkeitsmarker zuletzt. Erst danach bestätigt sie den Abschluss und startet neu.

## Wiederherstellung und Grenzen

- Abbruch, Stromausfall oder CRC-/Flash-Fehler vor dem abschließenden Marker lassen das neue Image ungültig. Der Bootloader bleibt für einen erneuten vollständigen Download erreichbar.
- Pad 2 während eines MCU-Resets halten, um den Update-Modus manuell zu öffnen. Dies auf dem realen CAP1203 prüfen; bei Fehlern ist SWD weiterhin der Rückweg.
- Ein aufgezeichneter CPU1-Fault oder Watchdog-Reset führt beim nächsten Start ebenfalls in den Bootloader. Ein erfolgreiches Update löscht den Fault-Marker.
- Der Updater hat **einen Anwendungsslot**. Während des Updates steht die vorige Anwendung nicht zur Verfügung. Es gibt keinen automatischen Rücksprung auf die vorherige Version.
- CRC32 und SHA-256 erkennen beschädigte Dateien. Es gibt **keine Firmware-Signatur und keinen Anti-Rollback-Zähler**. Ein physisch bestätigter, gebundener Besitzer darf die Anwendung ersetzen. Pakete deshalb aus einem vertrauenswürdigen Build beziehen.
- **CPU2-Bluetooth-Stack und FUS werden hier nicht OTA aktualisiert.** Dafür gelten andere ST-Verfahren, Speicheranforderungen und Versionsabhängigkeiten.
- Das eigene Protokoll ist mit dem unveränderten ST BLE Sensor OTA-Client nicht kompatibel. Den mitgelieferten Client verwenden.

Grundlagen: [ST AN5247](https://www.st.com/resource/en/application_note/an5247-overtheair-application-and-wireless-firmware-update-for-stm32wb-series-microcontrollers-stmicroelectronics.pdf), [ST BLE_Ota v1.13.3](https://github.com/STMicroelectronics/STM32CubeWB/tree/v1.13.3/Projects/P-NUCLEO-WB55.Nucleo/Applications/BLE/BLE_Ota), [Bleak-Client API](https://bleak.readthedocs.io/en/stable/api/client.html).
