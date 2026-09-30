# OTA-Startkorrektur für deine V1-Brille

Die Anwendung startet bei dir nach vollständiger Versorgungstrennung. Nach dem OTA blieb der bisherige Bootloader jedoch im Wiederherstellungsmodus. Dieses Paket enthält einen korrigierten Bootloader: Nach erfolgreicher Prüfung des Updates erhält der nächste Start einen einmaligen Anwendungsstartauftrag. Alte Touch-Zustände können diesen Start nicht mehr in den manuellen Update-Modus umleiten. Prüfsumme, Vektoren, CPU-Fault und Watchdog werden weiterhin geprüft. Bei manueller Pad-2-Anwahl werden zwei Messungen ausgewertet.

Die konkrete Ursache auf deiner Brille ist noch nicht gemessen. Diese Korrektur wurde gebaut und am Rechner getestet; der automatische OTA-Start muss danach auf deiner Brille bestätigt werden.

**Paket: Smartglasses-0.2.0-ota-start-fix-Wemos-Update.zip.** Es enthält die V1-Displaykorrektur und die neue Startkorrektur. BLE-/Manifestversion bleiben 0.2.0; eigene Dateinamen und SHA-256 unterscheiden diesen Stand. Die Android-App 1.1.1 kann weiterverwendet werden. Nur eine OTA-BIN zu übertragen genügt für diese Bootloader-Änderung nicht.

## Einmal mit deinem vorbereiteten Wemos installieren

1. Entpacke das ZIP in einen **neuen Ordner**, zum Beispiel `C:\Users\Toaster\Downloads\Smartglasses-OTA-Start-Fix`. Behalte die bisherigen Programmer-Ordner samt Sicherungen. Verwende die neue `Upgrade.ps1` aus diesem ZIP.
2. Trenne die Brillenversorgung vollständig und ziehe den Wemos vom USB ab. Verbinde die vier Leitungen:

   | Wemos S2 Mini | Brille |
   |---|---|
   | GND | Pad 4 – GND |
   | GPIO4 | Pad 6 – CLK |
   | GPIO5 | Pad 7 – DIO |
   | GPIO6 | Pad 8 – RST |

   Pad 3 (+3V) bleibt frei. Die Brille verwendet ihre eigene Versorgung. Die vorhandene CMSIS-DAP-Firmware auf deinem Wemos bleibt verwendbar.
3. Versorge die Brille wieder und warte, bis die Anwendung normal läuft. Verbinde dann den Wemos mit USB. Schließe den Web-Flasher und andere Programme, die den Wemos verwenden. Bei `CMD_INFO`/USB-Fehlern Wemos-USB fünf Sekunden trennen und erneut verbinden.
4. Öffne PowerShell im neu entpackten Ordner. Sichere zuerst den aktuell installierten Stand:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Pruefen
   ```

   Nur bei `PRUEFUNG UND SICHERUNG ERFOLGREICH` fortfahren. Die CPU bleibt nach dieser Sicherung angehalten.
5. Ohne zwischenzeitlichen Brillen-Neustart im selben Ordner installieren:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Installieren
   ```

   Nur `INSTALLATION UND VERIFIKATION ERFOLGREICH` bestätigt den Abschluss. Das Skript prüft die Paketdatei, dieselbe MCU, Sicherung und Flashbereiche. Es ersetzt Bootloader, Metadaten und Anwendung, erhält die Schlüsselseite 14 und verifiziert die Installation. CPU2/FUS und Option Bytes werden nicht programmiert. Bei Abbruch das Log aufbewahren und die Ursache klären.
6. **Brillenversorgung vollständig trennen**, Wemos-USB abziehen, Leitungen entfernen und mindestens zehn Sekunden warten. Dann die Brille wieder versorgen. Die Installation lässt die CPU absichtlich angehalten; nur die Leitungen zu entfernen genügt nicht.
7. Mit App 1.1.1 verbinden. Es soll „Verbunden und bereit“ erscheinen. Uhr/BLE-Warten bleiben im alten Linsenbereich. Ein neuer Zahlenvergleich muss mit dem Handy übereinstimmen; die obere und untere Dreiergruppe bilden zusammen die sechsstellige Zahl.

## Automatischen OTA-Start prüfen

1. Übertrage die neue `Firmware/Smartglasses-0.2.0-ota-start-fix-OTA.bin` zusammen mit ihrer gleichnamigen JSON-Datei aufs Handy. Verwende die Dateien aus demselben Paket.
2. Starte das Firmware-Update in App 1.1.1. Bestätige `OTA?` an der Brille mit Pad 1 für etwa eine Sekunde. Lasse danach alle Pads los und warte auf Übertragung und Wiederverbindung.
3. Nach dem Abschluss soll die Brille **ohne Versorgungstrennung** zur Anwendung wechseln. Die App soll wieder „Verbunden und bereit“ anzeigen. Die reine Anzeige „Update abgeschlossen“ genügt nicht, wenn der Verbindungsstatus weiterhin Wiederherstellungsmodus meldet.
4. Wiederhole das Update einmal mit denselben Dateien. Prüfe erneut den automatischen Start, Uhr und Nachrichtendarstellung.
5. Bleibt sie dennoch im Wiederherstellungsmodus, lies **vor einer Versorgungstrennung** mit „Brillenstatus lesen“ die gesamte Zeile aus und bewahre sie auf. Danach ist ein vollständiger Stromneustart der bereits auf deiner Brille erprobte Rückweg. Der Fehler wäre damit noch nicht behoben; die Diagnose soll nicht als erfolgreiche Hardwareabnahme gewertet werden.
