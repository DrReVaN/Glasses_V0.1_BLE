# OTA-Flashkorrektur für deine V1-Brille

Die Auslesung vom 30.09.2026 zeigt, dass der Bootloader den Auftrag für die Display-Fix-Datei angenommen hat, aber noch kein Firmwarebyte geschrieben hat. Die Metadaten und die zuvor per Wemos installierte Anwendung sind unverändert. `FLASH_SR = 0x00080000` zeigt die CPU2-Flashsperre PESD. Die Löschfunktion wurde noch nicht aufgerufen. Nach vollständiger Versorgungstrennung hast du die Anwendung wieder als „Verbunden und bereit“ bestätigt.

Im bisherigen Code fehlte die Auswahl der Semaphore-Steuerung des Bluetooth-Prozessors. Der neue Stand ruft vor dem Bluetooth-Start `SHCI_C2_SetFlashActivityControl(FLASH_ACTIVITY_CONTROL_SEM7)` auf, wie das [offizielle ST-CubeWB-OTA-Beispiel](https://github.com/STMicroelectronics/STM32CubeWB/blob/v1.13.3/Projects/P-NUCLEO-WB55.Nucleo/Applications/BLE/BLE_Ota/Core/Src/app_entry.c). Wird diese Auswahl abgelehnt, wird kein Flashzugriff freigegeben; die Diagnose meldet Fehler 11. Der Treiber prüft weiterhin die CPU2-Sperren und gibt nur selbst übernommene Semaphoren frei. Fehler beim Beenden der Löschaktivität oder beim erneuten Sperren des Flash werden weitergegeben.

**Paket: Smartglasses-0.2.0-ota-flash-fix-Wemos-Update.zip.** Es enthält Bootloader und Anwendung mit dieser Korrektur sowie die bisherigen V1-Display- und Startkorrekturen. BLE-/Manifestversion bleiben 0.2.0. Die neuen Dateinamen und Prüfsummen unterscheiden diesen Stand. Die Tests am Rechner sind bestanden; ein vollständiger OTA-Transfer mit automatischem Neustart muss auf deiner Brille noch bestätigt werden. Dieses Paket ersetzt keine CPU2-/FUS-Firmware und verändert keine Option Bytes.

## Einmal per Wemos installieren

1. Entpacke das neue ZIP in einen **neuen Ordner**, zum Beispiel `C:\Users\Toaster\Downloads\Smartglasses-OTA-Flash-Fix`. Behalte die alten Pakete und Sicherungen. Vermische die Skripte und Firmwaredateien verschiedener Pakete nicht.
2. Die Brille muss nach einem vollständigen Stromneustart normal laufen. Versorge den vorbereiteten Wemos per USB, bevor du ihn an die eingeschaltete Brille anschließt. Verbinde GND zuerst, dann die übrigen Leitungen:

   | Wemos S2 Mini | Brille |
   |---|---|
   | GND | Pad 4 – GND |
   | GPIO4 | Pad 6 – CLK |
   | GPIO5 | Pad 7 – DIO |
   | GPIO6 | Pad 8 – RST |

   **Für die Installation gehört die RST-Leitung wieder dazu.** Pad 3 (+3V) bleibt frei; die Brille verwendet ihre eigene Versorgung. Die bestehende CMSIS-DAP-Firmware des Wemos bleibt verwendbar. Schließe Web-Flasher und andere Programme, die auf den Wemos zugreifen.
3. Öffne PowerShell im neu entpackten Ordner und erstelle eine frische Sicherung:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Pruefen
   ```

   Nur bei `PRUEFUNG UND SICHERUNG ERFOLGREICH` fortfahren. Die CPU bleibt danach angehalten. Bei einem USB-/`CMD_INFO`-Fehler wurde noch keine Installation begonnen: Wemos-USB bei abgetrennten Programmierleitungen fünf Sekunden trennen und erneut verbinden. Anschließend die Prüfung wiederholen. Andere Abbrüche anhand des Logs klären.
4. Ohne zwischenzeitlichen Brillen-Neustart im selben Ordner installieren:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Installieren
   ```

   Nur `INSTALLATION UND VERIFIKATION ERFOLGREICH` bestätigt den Abschluss. Das Skript prüft dieselbe MCU, Paket und Sicherung, ersetzt gezielt Bootloader, Metadaten und Anwendung und verifiziert die Installation. Die Schlüsselseite 14 bleibt erhalten. Bei einem Abbruch das Log behalten.
5. Brillenversorgung **vollständig** trennen, Wemos-USB abziehen, alle Programmierleitungen entfernen und mindestens zehn Sekunden warten. Dann nur die Brille wieder versorgen; beim Start keine Pads berühren.
6. In der App muss „Verbunden und bereit“ erscheinen. Prüfe Uhr und Anzeige im alten Linsenbereich. Die App 1.1.1 bleibt kompatibel; empfohlen ist die neue **1.1.2**, die OTA-Abbrüche genauer anzeigt. Die lokal bereitgestellte 1.1.2-APK ist mit demselben Entwicklungsschlüssel wie 1.1.1 signiert.

## OTA erneut prüfen

1. Kopiere `Firmware/Smartglasses-0.2.0-ota-flash-fix-OTA.bin` und die gleichnamige JSON-Datei auf das Handy. Verwende zuerst diese beiden Dateien aus dem neuen Paket. Eine OTA-BIN allein aktualisiert den Bootloader nicht; Schritt 4 oben ist deshalb erforderlich.
2. Wähle beide Dateien in der App und starte das Update. Bestätige `OTA?` an der Brille mit Pad 1 für etwa eine Sekunde und lasse danach alle Pads los. Brille während des Transfers eingeschaltet lassen, Wemos und Programmierleitungen bleiben entfernt.
3. Es muss nun eine tatsächliche Datenübertragung mit steigendem Fortschritt stattfinden. 100 % bedeutet nur, dass die Daten übertragen wurden. Erfolg liegt erst vor, wenn die Abschlussprüfung bestätigt wurde und die App nach dem Neustart wieder „Verbunden und bereit“ meldet.
4. Prüfe Uhr und Textübertragung. Wiederhole das Update einmal mit denselben Dateien und bestätige erneut den automatischen Start ohne Versorgungstrennung.
5. Bei einem Fehler **vor einem Stromneustart** die genaue App-Meldung und „Brillenstatus lesen“ festhalten, insbesondere Fortschritt, Reset und Fehlernummer. App 1.1.2 behält den letzten OTA-Fehler auch nach Wiederverbindung und Diagnoseabfrage. Kein automatischer erneuter Upload wird ausgelöst.

Nach begonnenem Löschen oder Schreiben gibt es bei diesem einzelnen Anwendungsslot keinen automatischen Rücksprung zur alten Anwendung. Ein unterbrochener Transfer benötigt einen vollständigen neuen Upload im Bootloader. Die bisherigen fehlgeschlagenen Versuche hatten laut Auslesung noch nichts gelöscht; daraus lässt sich keine Zusicherung für einen späteren Abbruch ableiten.
