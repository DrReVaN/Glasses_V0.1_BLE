# OTA-Löschfreigabe: neues Wemos-Paket

Die Auslesung deiner weiterhin eingeschalteten Brille vom 30.09.2026 bestätigt den installierten Flash-Fix-Bootloader bytegenau. Die SEM7-Initialisierung ist erfolgreich (`flash_ready=1`), PESD ist nicht mehr gesetzt (`FLASH_SR=0`), trotzdem wurde beim OTA-Start noch keine Seite gelöscht. Metadaten und Anwendung sind unverändert. Die App 1.1.2 meldet deshalb korrekt, dass BEGIN nicht bestätigt wurde. Dies ist noch kein Fehler beim anschließenden Anwendungsstart.

Der bisherige Treiber sendet bei jedem Versuch `ERASE_ACTIVITY_ON`. CPU2 nimmt daraufhin SEM7 bis zu einem späteren Funkereignis. Wird sofort BUSY gemeldet und `ERASE_ACTIVITY_OFF` gesendet, beginnt beim nächsten Versuch dieselbe Wartephase von vorne. Die Löschfreigabe muss über blockierte Versuche und alle zu löschenden Seiten hinweg bestehen bleiben; erst danach wird sie beendet. Dies entspricht dem Ablauf in [ST AN5289, Abschnitt Flash-Arbitration](https://www.st.com/resource/en/application_note/an5289-how-to-build-wireless-applications-with-stm32wb-mcus-stmicroelectronics.pdf) und im CubeWB-Flash-Treiber. Ein zusätzlicher Test mit dem bisherigen Produktionscode reproduziert dieses Wiederholen der Freigabe ohne Fortschritt.

Die Korrektur hält diese Freigabe nun aktiv, räumt sie nach Abschluss/Fehler/Disconnect/Timeout im Vordergrund auf und erlaubt Daten- oder Metadaten-Schreibzugriffe erst nach ihrem Ende. Der Bootloader fordert bei kurzen Verbindungsintervallen außerdem 80–100 ms an, damit CPU2 eine für das Löschen nötige Funkpause von mindestens 25 ms einplanen kann. Das Handy entscheidet über die Verhandlung dieser Parameter.

Die Tests führen jetzt **OTA-Empfänger und Produktions-Flash-Treiber gemeinsam** aus. Sie simulieren eine erst später eintreffende CPU2-Freigabe, mehrere Seiten mit nur einer Löschanmeldung, vollständigen Transfer, Commit, Bootentscheidung und verzögerte Bereinigung nach Disconnect/Timeout. Die neue Korrektur wurde am Rechner geprüft; der echte OTA-Durchlauf auf deiner Brille ist noch offen.

**Verwende Smartglasses-0.2.0-ota-erase-fix-Wemos-Update.zip.** Es ersetzt Bootloader und Anwendung und enthält alle bisherigen Display-/Start-/Flashkorrekturen. Android-App **1.1.2** bleibt verwendbar. BLE-/Manifestversion bleiben 0.2.0. Nur eine BIN per OTA zu übertragen ersetzt den fehlerhaften Bootloader nicht.

## Einmal per Wemos installieren

1. Entpacke dieses neue ZIP in einen neuen Ordner, beispielsweise `C:\Users\Toaster\Downloads\Smartglasses-OTA-Erase-Fix`. Bewahre die alten Pakete und Sicherungen auf. Verwende ausschließlich Skripte und Firmwaredateien aus diesem neuen ZIP.
2. Trenne zunächst Brillenversorgung und Wemos-USB, entferne die Diagnoseleitungen und warte mindestens zehn Sekunden. Versorge nur die Brille wieder, berühre beim Start keine Pads und warte auf die normal laufende Anwendung.
3. Versorge den vorbereiteten Wemos per USB und verbinde GND zuerst, danach die übrigen drei Leitungen:

   | Wemos S2 Mini | Brille |
   |---|---|
   | GND | Pad 4 – GND |
   | GPIO4 | Pad 6 – CLK |
   | GPIO5 | Pad 7 – DIO |
   | GPIO6 | Pad 8 – RST |

   **RST gehört für die Installation wieder dazu.** Pad 3 (+3V) bleibt frei. Die Brille wird weiterhin separat versorgt. Schließe Web-Flasher und andere Programme mit Wemos-Zugriff.
4. Öffne PowerShell im neuen Paketordner. Erstelle eine frische Sicherung:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Pruefen
   ```

   Nur bei `PRUEFUNG UND SICHERUNG ERFOLGREICH` fortfahren. Die CPU bleibt angehalten. Bei einem USB-/`CMD_INFO`-Fehler wurden noch keine Firmwaredaten installiert; Wemos bei abgetrennten Programmierleitungen fünf Sekunden vom USB trennen und neu verbinden, anschließend die Prüfung wiederholen. Bei anderen Fehlern das Log behalten.
5. Ohne zwischenzeitlichen Brillen-Neustart im gleichen Ordner installieren:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Installieren
   ```

   Nur `INSTALLATION UND VERIFIKATION ERFOLGREICH` bestätigt den Abschluss. Das Skript prüft Gerät, Sicherung und Paket, ersetzt Bootloader/Metadaten/Anwendung und verifiziert die Schlüsselseite 14 unverändert. CPU2/FUS und Option Bytes werden nicht programmiert.
6. Brillenversorgung vollständig trennen, Wemos-USB abziehen, alle Leitungen entfernen und mindestens zehn Sekunden warten. Dann nur die Brille wieder versorgen, ohne Pads zu berühren. App 1.1.2 muss „Verbunden und bereit“ anzeigen.

## Den neuen OTA-Versuch vorbereiten

1. Kopiere diese **beiden neuen Dateien** aus dem Paket in einen neuen Handyordner:

   - `Firmware/Smartglasses-0.2.0-ota-erase-fix-OTA.bin`
   - `Firmware/Smartglasses-0.2.0-ota-erase-fix-OTA.json`

2. Wähle in App 1.1.2 BIN und JSON erneut aus. Kontrolliere die angezeigte Byteanzahl anhand von `size` in der JSON-Datei. Verlasse dich nicht allein auf „Firmware 0.2.0“, weil die bisherigen Pakete dieselbe Protokollversion tragen.

   Die letzte Auslesung enthielt einen Auftrag für **35412 B, CRC 0059e90c** — das entspricht der älteren Display-Fix-Datei. Der installierte Flash-Fix-Stand hatte **35564 B, CRC 5979a42b**. Die Korrektur der Löschfreigabe ist unabhängig davon nötig; der nächste Versuch soll eindeutig die Dateien aus diesem neuen Paket verwenden.
3. Starte OTA und bestätige `OTA?` mit Pad 1 für etwa eine Sekunde. Danach Pads loslassen. Brille eingeschaltet lassen; Wemos und Programmierleitungen bleiben entfernt.
4. Warte auf steigenden Fortschritt, Abschlussprüfung und Wiederverbindung zur Anwendung. **100 % allein ist kein Erfolg.** Erst die bestätigte Abschlussprüfung und „Verbunden und bereit“ zeigen einen abgeschlossenen Durchlauf.
5. Prüfe Uhr/Text und wiederhole einmal das OTA mit denselben Dateien, jeweils ohne Versorgungstrennung nach dem Upload. Bleibt ein Fehler, notiere vor einem Stromneustart die ganze App-Meldung, Prozentzahl, ausgewählte Paketgröße und „Brillenstatus lesen“.

Nach tatsächlich begonnenem Löschen gibt es bei diesem einzelnen Anwendungsslot keinen automatischen Rücksprung zur alten Anwendung. Ein unterbrochener Transfer braucht einen vollständigen neuen Upload im Bootloader. Die bisher ausgelesenen fehlgeschlagenen BEGIN-Versuche hatten noch nichts gelöscht.
