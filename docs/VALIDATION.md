# Änderungen und Verifikation

Ausgangspunkt: Firmware-main `4b314ee3e5207037aa04d2b64c7a0a70b461ff6c`; App-main `ffeecca218615d59145256f00cd9af490690e08a`. Die Änderungen gehören ausschließlich auf neue Dev-Branches.

| Review-Befund | Behebung |
|---|---|
| F1: Paket-/Textüberläufe | Header-, Reihenfolge-, Längen- und UTF-8-Prüfung; begrenzter Staging- und Displaypuffer |
| F2: extern int statt uint8_t | Alte Statusfelder entfernt; gemeinsames, typisiertes volatile IRQ-Ereignis, Zustandswechsel nur im Vordergrund |
| F3: Low-Power-Run bei 32 MHz/RF | LPR-Aufrufe entfernt; CPU1 Sleep, SysTick und CPU2 bleiben aktiv |
| F4: wiederholtes OFF | eigener boolescher OFF-Zustand und einmalige Aktion pro entprelltem Langdruck |
| F5: Anzeige vor Empfangsabschluss | Veröffentlichung erst nach letztem gültigen Fragment; Queue für fertige Texte |
| F6: alte Scrollposition | neue Nachricht initialisiert Scrollposition und Frist gemeinsam |
| F7: lastabhängige Zähler/lange Delays | Millisekundenfristen; 50-ms-Touch-Sampling; nicht blockierender OLED-Warmstart und Vibrationsimpuls |
| F8: ungeschützte Dienste/automatisches YES | authentifizierte verschlüsselte Schreibrechte, Secure Connections, Bonding, tatsächlicher Zahlenvergleich, individuelle persistente Root-Schlüssel |
| F9: verschluckte Busfehler/unendliches SPI-Warten | realer HAL-I2C-Handle, Fehlerresultate, kurze Timeouts, Backoff, Watchdog und Resetdiagnose |
| F10: statische Handy-Uhr | geprüfter Uhr-/Datums-Parser und lokale Zeitfortschreibung mit Kalenderwechsel |
| UTF-8/ASCII | definierte Transliteration/Ersatzdarstellung und byteweise begrenzte App-Fragmente |
| App-Endlosschleife | GATT-Queue mit Antworten, zehn Startversuchen, 5-s-Operationsfrist, 60-s-Pairingfrist, Abbruch beim Disconnect |
| CAP1203-ALERT# | fallende Flanke und zusätzliche Statusabfrage; tatsächliche Boardpolarität noch messen |
| OLED-Grenzen | gedrehte 64×128-Koordinaten begrenzt; Zeichen-/Fontgrenzen geprüft |
| RTC-Wartebedingung | WUTWF-Stabilisierung wiederhergestellt und im unerwarteten IRQ begrenzt; kein Wakeup-Start vor Timer-Server-Initialisierung |
| Wartbarkeit/OTA | getrennte Core-/Hardwaremodule, GCC-Build, Keil-Targets, Linkergrenzen, Metadaten, eigener Updater, CI und Dokumentation |

## Am Rechner ausgeführt

- Beide ARM-Images mit Arm GNU Toolchain **13.2.1** übersetzt und gelinkt; am Ende ohne Compiler-/Linkerwarnungen.
- OLED-Befehlsfolge, Bildschirmtransfer, SPI-Konfiguration, Spiegelung und Puffergröße mit dem zuvor funktionierenden main-Stand verglichen; diese Hardwareparameter sind unverändert. Neue Bestätigungstexte sind kompakter und der Lauftext verwendet wieder fünf Zeichen pro Fenster.
- BSS besitzt in beiden GCC-Linkern einen eigenen RAM-Segment-Eintrag. Unterschiedliche Daten-/BSS-Ausrichtung erzeugt dadurch keinen fehlerhaften Segment-Eintrag; Linkerwarnungen brechen den Build ab.
- Portablen C-Empfänger, Uhr, Touch und CRC getestet, einschließlich 100.000 fehlerhafter Pakete. Die erste lokale Ausführung verwendete UndefinedBehaviorSanitizer.
- Den tatsächlichen OTA-Empfänger mit simuliertem Flash ausgeführt: Busy/Retry, Größen-/Adressgrenzen, falscher Offset/Duplikat, Disconnect, CRC-Fehler, Schreibfehler, letzte unvollständige Doppelwortgruppe, abschließender Metadatenmarker und Timeout.
- Python-Paketvalidierung und Intel-HEX-Prüfsummen getestet; erzeugtes Installationsimage und OTA-Manifest geprüft.
- Android-Debug-APK mit JDK 11, SDK 31 und Build Tools 30.0.3 gebaut; Gradle-Unit-Tests bestanden. Vorhandene Deprecated-/Unchecked-Hinweise stammen aus dem alten Android-Projekt.
- Java-Framing und GATT-Queue ausgeführt: UTF-8-Grenzen, Serialisierung, begrenzte Versuche, Timeout, Disconnect und Wiederverbindung.

Reproduzierbar:

```text
python tools/test.py --cc gcc --sanitize
python -m unittest discover -s tests -p "test_*.py"
python tools/build.py --toolchain-bin <ARM-Toolchain>/bin
python tools/package.py
python tools/ota.py build/application/smartglasses.bin --check
```

Der Sanitizer-Aufruf ist für Linux/GCC gedacht; auf Windows kann `--cc "zig cc"` ohne AddressSanitizer verwendet werden. Die Tests führen den C-Code aus und ersetzen keine Messungen an der MCU.

## Hardwareabnahme vor Alltagsbetrieb

Die vom Besitzer bestätigte und fotografierte Controllerplatine ist **V1 mit STM32WB35**. Der zuvor funktionierende OLED-Code dient als Kompatibilitätsreferenz; die sichtbare Panelauflösung wurde nicht identifiziert. Der veröffentlichte V2-Schaltplan ist daher keine vollständige Verifikation dieser V1-Platine. Details und unveränderte Displayparameter stehen im [Hardwareabgleich](HARDWARE.md).

1. Startup, CPU2-Stack, LSE-Anlauf, Timer-Server und Uhr über mehrere Disconnects sowie Tages-/Monatswechsel prüfen.
2. OLED-Drehung, Reset-Puls und Versorgung messen; alle Anzeige-/OFF-/ON-Abläufe wiederholt testen.
3. CAP1203-ALERT#-Polarität am Board messen, alle Pads/Chords und manuelle Bootloader-Anwahl prüfen.
4. Kurze, lange und fragmentierte Unicode-Nachrichten, schnelle Nachrichtenfolgen und Abbruch nach jedem Fragment testen.
5. Neuen/bekannten/fremden BLE-Client prüfen: falschen Zahlenvergleich und Timeout ablehnen; ungepaarte Schreibzugriffe müssen scheitern; Bond nach Reset/OTA erhalten.
6. I2C-/SPI-Fehler provozieren. Prüfen, dass BLE weiter bedienbar bleibt oder Watchdog/Fault in den Bootloader führen; Diagnosecharakteristik lesen.
7. OTA mindestens zweimal ausführen. Verbindung und Versorgung während Erase, Datenübertragung, CRC-Prüfung und Metadatencommit unterbrechen; vollständigen Download erneut starten.
8. Beschädigtes Image, falsche Zieladresse, ungültige Vektoren und zu große Images zurückweisen. WRP/SFSA prüfen; CPU2/FUS dürfen sich nicht ändern.
9. Nach Update Uhr/App-Protokoll und gespeicherte Bindung prüfen. MCU-Reset mit Pad 2 und Wiederherstellung über SWD erproben.
10. Stromaufnahme bei verbundenem BLE, Advertising, OLED-an/-aus und OFF messen. CPU1-Sleep ist sicher vorbereitet; Stop/Standby und weitere Verbrauchsoptimierungen benötigen diese Messungen.

Kein Flash-Vorgang und kein realer BLE-OTA-Transfer wurden in dieser Arbeitsumgebung durchgeführt. Die ursprüngliche Reset-Ursache ist deshalb trotz reparierter Codefehler nicht als auf Hardware nachgewiesen behoben zu bezeichnen.
