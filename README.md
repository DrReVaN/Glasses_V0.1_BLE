# Smartglasses firmware 0.2.0 – V1-Display, Sonderzeichen und OTA

Firmware für die STM32WB35CE-Brille aus dem [Hackster-Projekt](https://www.hackster.io/team-smart-glasses/diy-smart-glasses-20a2bf).

Dieser Entwicklungsstand repariert Empfang, Interruptbehandlung, Bedienzeiten, OFF/ON, Lauftext, Uhr und Treiberfehler. Geschützte BLE-Schreibzugriffe benötigen eine am Gerät bestätigte Bindung. Ein eigener CPU1-Bootloader ermöglicht anschließend Firmware-Updates über BLE.

Die V1-Displaykorrektur stellt die ursprünglichen Koordinaten und Zeichenabstände für Uhr, Datum, BLE-Warten und das erste Lauftextbild wieder her. Pairing und OTA verwenden denselben kleinen Inhaltsbereich. BLE- und Manifestversion bleiben für die vorhandene Android-App bei 0.2.0; korrigierte Dateien tragen den Zusatz `display-fix` und eigene Prüfsummen. Zum Korrigieren auch der Bootloader-Anzeige beide Images über SWD installieren; ein BLE-OTA ersetzt nur die Anwendung.

Die anschließende OTA-Startkorrektur (`ota-start-fix`) schreibt nach dem geprüften Abschluss einen einmaligen Startauftrag in den RTC-Backupbereich. Der Bootloader prüft das Image erneut und startet es unabhängig von alten Touch-Zuständen. Eine manuelle Pad-2-Anwahl benötigt zwei bestätigte Messungen. **Auch diese Änderung benötigt einmal die Installation des neuen Bootloaders über SWD.** Der Besitzer konnte die Anwendung nach einem vollständigen Stromneustart starten; ob diese Korrektur den beobachteten Warmstart auf seiner Brille behebt, muss dort noch geprüft werden.

**Bei der alten Originalfirmware einmal Bootloader und Anwendung über SWD installieren.** Der alte Firmwarestand allein kann noch kein OTA. Nach Installation der Löschkorrektur `f55c081` bestätigte der Besitzer einen erfolgreichen OTA-Durchlauf. Weitere Hardwareprüfungen, insbesondere absichtliche Unterbrechungen, bleiben offen.

Die aktuelle OTA-Löschkorrektur (`ota-erase-fix`) hält die CPU2-Löschfreigabe während blockierter Versuche und aller Seiten aktiv und beendet sie vor der Datenübertragung. Der bisherige ON/OFF-Wechsel bei jedem Versuch konnte die Freigabe immer wieder verzögern. Kurze Bootloader-Verbindungsintervalle werden auf 80–100 ms angefragt. OTA-Empfänger und Produktions-Flash-Treiber werden hierfür gemeinsam mit einer zeitversetzten CPU2-Freigabe getestet. Auch diese Bootloader-Korrektur wird einmal per Wemos/SWD installiert.

## Einstieg

Der aktuelle Nachrichtenfix (`text-fix`) ergänzt eigene Euro-/Latin-1-Glyphen, setzt häufige Unicode-Satzzeichen lesbar um und richtet Nachrichten wie die Uhr bei X=6 aus. Sechs Zeichen passen bei gleicher 7×10-Schriftgröße in die bestehende Zeile. Mit dem bereits funktionierenden Löschkorrektur-Bootloader genügt das Update der Anwendung über OTA; Android-App 1.1.2 bleibt verwendbar. [OTA-Anleitung und Zeichenumfang](docs/TEXT_UPDATE.md).

1. Arm GNU Toolchain **13.2.Rel1** (GCC 13.2.1) und Python 3.10+ bereitstellen.
2. Im Repository ausführen:

```text
python tools/build.py --toolchain-bin <ARM-Toolchain>/bin
python tools/package.py
python tools/ota.py build/application/smartglasses.bin --check
```

Das erzeugt zwei getrennte Images, Prüfsummen und `build/install.hex` für die Erstinstallation. Build-Produkte werden nicht im Quellcode-Branch versioniert. GitHub Actions erzeugt dieselben Dateien als Build-Artefakt.

- [OTA, Speicheraufteilung und Erstinstallation](docs/OTA.md)
- [BLE-Protokoll und Bedienung](docs/PROTOCOL.md)
- [Tests, behobene Befunde und Hardwareabnahme](docs/VALIDATION.md)
- [Bestückte V1-Hardware und Abgleich mit dem veröffentlichten V2-Schaltplan](docs/HARDWARE.md)
- [V1-Displaykorrektur mit dem vorbereiteten Wemos installieren](docs/DISPLAY_UPDATE.md)
- [OTA-Startkorrektur mit dem vorbereiteten Wemos installieren](docs/OTA_START_UPDATE.md)
- [Aktuelle OTA-Löschkorrektur mit dem vorbereiteten Wemos installieren](docs/OTA_ERASE_UPDATE.md)
- [Sonderzeichen und ausgerichteten Nachrichtentext per OTA installieren](docs/TEXT_UPDATE.md)
- [Passende Android-App auf dem Dev-Branch](https://github.com/DrReVaN/SmartGlasses-App/tree/dev/firmware-fixes-ota)

## Quellcode und Build

`Core/Src/glasses_core.c` enthält den portablen Empfang, die lokale Uhr und Touch-Auswertung. `glasses_app.c` steuert die Hardware mit kurzen Arbeitsschritten; `glasses_display.c` zeichnet das V1-Layout; `glasses_ota.c` implementiert den Update-Empfang. `glasses_flash.c` koordiniert Flash-Zugriffe mit CPU2 und erzeugt einmalig individuelle BLE-Schlüssel.

Die aktiven Touch-/OLED-Treiber liegen unter `MDK-ARM/*.c`; ihre Header liegen unter `MDK-ARM/RTE`. Die abweichenden RTE-Implementierungskopien und alten Build-Produkte wurden entfernt.

Das vorhandene Keil-Projekt besitzt die Targets `Smartglasses-application` und `Smartglasses-bootloader` mit getrennten Scatter-Dateien. Die ursprüngliche Referenz ist ARMCC 5.06 Update 7 und Keil.STM32WBxx_DFP 1.2.0. **Der tatsächlich geprüfte Build ist der GCC-Build.** Keil wurde hier nicht ausgeführt. CubeMX-Regeneration kann manuelle GATT- und Linkeränderungen überschreiben; Änderungen daran anschließend gezielt abgleichen.

ST-HAL/CMSIS/Middleware und die CPU2-Binaries stammen aus dem bestehenden Repository. Die Flash-Arbitration orientiert sich an der offiziellen ST-Referenz [STM32CubeWB v1.13.3, BLE_Ota](https://github.com/STMicroelectronics/STM32CubeWB/tree/v1.13.3/Projects/P-NUCLEO-WB55.Nucleo/Applications/BLE/BLE_Ota). Das verwendete OTA-Protokoll ist projektspezifisch.
