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
| V1-Linsenbereich | ursprüngliche Uhr-/Datumspositionen und Zeichenabstände, BLE-Warteanzeige und erstes Lauftextbild wiederhergestellt; Pairing und OTA innerhalb der alten Inhaltsgrenzen |
| OTA-Warmstart / Touch-Anwahl | einmaliger, zurückgelesener Anwendungsstartauftrag nach erfolgreichem Commit; manuelle Pad-2-Anwahl mit zwei Messungen; Bootentscheidung wird im Hosttest tatsächlich ausgeführt |
| OTA vor erstem Löschen blockiert | CPU2-Flashsteuerung vor BLE-/Schlüsselinitialisierung auf SEM7 umgestellt; abgelehnte Initialisierung sperrt Schreibzugriffe; Produktions-Flash-Treiber einschließlich PESD-Ausgangszustand getestet |
| Löschfenster bei jedem Busy neu gestartet | CPU2-Löschfreigabe über Retries und alle Seiten aktiv gehalten; Bereinigung im Vordergrund vor Datenübertragung und nach Abbruch; kurze Bootloader-Verbindungsintervalle auf 80–100 ms angefragt |
| RTC-Wartebedingung | WUTWF-Stabilisierung wiederhergestellt und im unerwarteten IRQ begrenzt; kein Wakeup-Start vor Timer-Server-Initialisierung |
| Wartbarkeit/OTA | getrennte Core-/Hardwaremodule, GCC-Build, Keil-Targets, Linkergrenzen, Metadaten, eigener Updater, CI und Dokumentation |

## Am Rechner ausgeführt

- Beide ARM-Images mit Arm GNU Toolchain **13.2.1** übersetzt und gelinkt; am Ende ohne Compiler-/Linkerwarnungen.
- OLED-Befehlsfolge, Bildschirmtransfer, SPI-Konfiguration, Spiegelung und Puffergröße mit dem zuvor funktionierenden main-Stand verglichen; diese Hardwareparameter sind unverändert. Neue Bestätigungstexte sind kompakter und der Lauftext verwendet wieder fünf Zeichen pro Fenster.
- BSS besitzt in beiden GCC-Linkern einen eigenen RAM-Segment-Eintrag. Unterschiedliche Daten-/BSS-Ausrichtung erzeugt dadurch keinen fehlerhaften Segment-Eintrag; Linkerwarnungen brechen den Build ab.
- Portablen C-Empfänger, Uhr, Touch und CRC getestet, einschließlich 100.000 fehlerhafter Pakete. Die erste lokale Ausführung verwendete UndefinedBehaviorSanitizer.
- Den tatsächlichen OTA-Empfänger mit simuliertem Flash ausgeführt: Busy/Retry, Größen-/Adressgrenzen, falscher Offset/Duplikat, Disconnect, CRC-Fehler, Schreibfehler, letzte unvollständige Doppelwortgruppe, abschließender Metadatenmarker und Timeout.
- Den tatsächlichen Flash-Treiber ausgeführt: fehlende/abgelehnte CPU2-Initialisierung, PESD vor der ersten Metadatenlöschung, SEM2/6/7-Konflikte, nachträgliche PESD-Sperre, HAL- und SHCI-Fehler einschließlich Cleanup, Erhaltung des Interruptzustands, Adress-/SFSA-/Ausrichtungsgrenzen, Schreibprüfung und erhaltene BLE-Schlüssel. Die bisherigen OTA-Tests simulierten diesen Treiber und konnten die fehlende CPU2-Konfiguration daher nicht erkennen.
- OTA-Empfänger und Produktions-Flash-Treiber gemeinsam ausgeführt: zeitversetzte CPU2-Freigabe nach Funkereignis, eine Anmeldung für mehrere Löschseiten, vollständiger Transfer/Commit/Boot, Schreibsperre vor Ende der Löschaktivität sowie Bereinigung im Vordergrund nach Disconnect/Timeout und bei blockierter SEM2-Bereinigung. Der neue Funkfenster-Test scheiterte mit dem bisherigen Produktionscode und besteht mit der Korrektur.
- Den tatsächlichen Bootpfad ausgeführt: gewöhnlicher Start, gehaltenes Pad 2, gelöster/gespeicherter Touch-Zustand, einmalige OTA- und Anwendungsstartaufträge, beschädigte CRC, falsche Metadaten/Format/Länge/Vektoren, CPU-Fault/Watchdog und fehlgeschlagene RTC-Rückleseprüfung. Nach Commit bleibt der verzögerte Reset auch bei BLE-Disconnect aktiv; ein fehlgeschlagener Startauftrag meldet einen ATT-Fehler und löst keinen erfolgreichen OTA-Neustart aus.
- Python-Paketvalidierung und Intel-HEX-Prüfsummen getestet; erzeugtes Installationsimage und OTA-Manifest geprüft.
- Android-Debug-APK mit JDK 21, SDK 36 und Build Tools 35.0.0 gebaut; Gradle-Unit-Tests bestanden. Frameworktests laufen auf SDK 28 und 35; veraltete native API-Überladungen bleiben für ältere Android-Versionen nötig.
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

Für die V1-Displaykorrektur werden zusätzlich die tatsächlich übertragenen SPI-Framebuffer geprüft: alle 1.440 Uhrzeiten, Datumsfelder, BLE-Warten und erste Lauftextbilder stimmen pixelweise mit den ursprünglichen Zeichenpositionen überein. Pairing (alle 1.000 dreistelligen Gruppen in beiden Zeilen), OTA-Frage, Bootloader, fehlende Uhrzeit und alle Lauftextpositionen bleiben innerhalb der alten Inhaltsgrenzen. Hierfür werden Produktionsrenderer, Treiber und Fonts verwendet; die Prüfung ist in `tools/test.py` und damit in GitHub Actions eingebunden.

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

Der Besitzer hat den vorherigen 0.2.0-Stand über Wemos/SWD installiert, ein erfolgreiches Verifikationslog gemeldet und nach Neustart ein Bild gesehen. Dabei wurde die Abweichung vom alten optischen Anzeigebereich festgestellt. Die anschließende V1-Displaykorrektur ist am Rechner geprüft; ihre Sichtbarkeit durch die konkrete Linse und ein realer BLE-OTA-Transfer sind noch an der Brille zu bestätigen. Die ursprüngliche Reset-Ursache ist trotz reparierter Codefehler nicht als auf Hardware nachgewiesen behoben zu bezeichnen.

Am 30.09.2026 meldete der Besitzer nach einem OTA-Versuch weiterhin den Wiederherstellungsmodus mit `Reset: 14000000`, `Fehler: 0`. Auch der anschließend per Wemos installierte Stand `c3fc937` zeigte beim OTA der Display-Fix-Datei dieses Verhalten. Die folgende SWD-Auslesung bestätigt: installierter Bootloader und Anwendung stimmen mit dem Wemos-Paket überein; Metadaten bleiben gültig und unverändert; empfangene Firmwarebytes sind 0; der BEGIN-Auftrag wartet noch auf die Metadatenlöschung; `FLASH_SR=0x00080000` (PESD), Flash-HAL-Zustand unbenutzt, SEM2/6/7 frei. CPU1 stand beim kurzen Diagnose-Halt in der normalen WFI-Schleife und wurde danach ohne Reset fortgesetzt. Laufende RAM-Lesungen waren unzuverlässig und wurden nicht für diese Diagnose verwendet. Nach Entfernen des Wemos und vollständiger Versorgungstrennung bestätigte der Besitzer erneut „Verbunden und bereit“.

Damit setzt die bisherige Startkorrektur für diesen gemessenen Fehler zu spät an: Es wurde noch kein neues Image committed. Die fehlende Auswahl `SHCI_C2_SetFlashActivityControl(FLASH_ACTIVITY_CONTROL_SEM7)` vor `APP_BLE_Init` wurde anhand des offiziellen CubeWB-v1.13.3-OTA-Beispiels und der SHCI-Schnittstellendokumentation korrigiert. CPU2 verwendet standardmäßig PESD; der foreground Flash-Treiber benötigt die passende SEM7-Steuerung. [Installationsanleitung und verbleibende Hardwareprüfung](OTA_FLASH_UPDATE.md). Ein erfolgreicher echter OTA-Transfer, Warmstart und zweiter Transfer sind nach dieser Korrektur noch offen; die vorherige Touch-Hypothese ist für den gemessenen Stillstand nicht belegt.

Die folgende Installation `af18762` wurde über das erfolgreiche Wemos-Log und den bytegenauen Bootloader-Dump bestätigt. Nach einem weiteren abgebrochenen BEGIN meldete App 1.1.2 „Der Update-Start wurde nicht bestätigt“; die MCU-Auslesung zeigt `flash_ready=1`, `FLASH_SR=0`, unbenutzten Flash-HAL-Zustand und unveränderte Metadaten (35564 B, CRC 5979a42b). Es wurde weiterhin keine Seite gelöscht. Der letzte Auftrag in RAM enthält 35412 B/CRC 0059e90c, passend zur älteren Display-Datei. Damit ist die fehlende Initialisierung behoben, der folgende Lösch-Handshake aber noch fehlerhaft. Der zusätzliche Test reproduziert den ON/OFF-Wechsel mit dem bisherigen Produktionscode und führt nach der Korrektur den echten Empfänger samt Flash-Treiber bis Commit und Boot aus. [Neues Paket und ausstehender Hardwaretest](OTA_ERASE_UPDATE.md).
