# Smartglasses firmware 0.2.0

Firmware für die STM32WB35CE-Brille aus dem [Hackster-Projekt](https://www.hackster.io/team-smart-glasses/diy-smart-glasses-20a2bf).

Dieser Entwicklungsstand repariert Empfang, Interruptbehandlung, Bedienzeiten, OFF/ON, Lauftext, Uhr und Treiberfehler. Geschützte BLE-Schreibzugriffe benötigen eine am Gerät bestätigte Bindung. Ein eigener CPU1-Bootloader ermöglicht anschließend Firmware-Updates über BLE.

**Vor der Verwendung einmal Bootloader und Anwendung über ST-Link installieren.** Der alte Firmwarestand allein kann noch kein OTA. Diese Implementierung wurde gebaut und am Rechner getestet; die Hardwareabnahme steht aus.

## Einstieg

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
- [Passende Android-App auf dem Dev-Branch](https://github.com/DrReVaN/SmartGlasses-App/tree/dev/firmware-fixes-ota)

## Quellcode und Build

`Core/Src/glasses_core.c` enthält den portablen Empfang, die lokale Uhr und Touch-Auswertung. `glasses_app.c` steuert die Hardware mit kurzen Arbeitsschritten; `glasses_ota.c` implementiert den Update-Empfang. `glasses_flash.c` koordiniert Flash-Zugriffe mit CPU2 und erzeugt einmalig individuelle BLE-Schlüssel.

Die aktiven Touch-/OLED-Treiber liegen unter `MDK-ARM/*.c`; ihre Header liegen unter `MDK-ARM/RTE`. Die abweichenden RTE-Implementierungskopien und alten Build-Produkte wurden entfernt.

Das vorhandene Keil-Projekt besitzt die Targets `Smartglasses-application` und `Smartglasses-bootloader` mit getrennten Scatter-Dateien. Die ursprüngliche Referenz ist ARMCC 5.06 Update 7 und Keil.STM32WBxx_DFP 1.2.0. **Der tatsächlich geprüfte Build ist der GCC-Build.** Keil wurde hier nicht ausgeführt. CubeMX-Regeneration kann manuelle GATT- und Linkeränderungen überschreiben; Änderungen daran anschließend gezielt abgleichen.

ST-HAL/CMSIS/Middleware und die CPU2-Binaries stammen aus dem bestehenden Repository. Die Flash-Arbitration orientiert sich an der offiziellen ST-Referenz [STM32CubeWB v1.13.3, BLE_Ota](https://github.com/STMicroelectronics/STM32CubeWB/tree/v1.13.3/Projects/P-NUCLEO-WB55.Nucleo/Applications/BLE/BLE_Ota). Das verwendete OTA-Protokoll ist projektspezifisch.
