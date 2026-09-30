# Displaykorrektur für deine V1-Brille

Die erste OTA-Firmware zeichnete außerhalb des bisher verwendeten Bereichs. Dieses Update setzt Uhr, Datum und BLE-Warten auf die ursprünglichen Zeichenpositionen zurück. Das erste Lauftextbild beginnt wieder bei (14,61). Alle neuen Dialoge bleiben im alten Inhaltsbereich. Display-Drehung, Spiegelung und OLED-Initialisierung entsprechen weiterhin der funktionierenden alten Firmware.

Das Paket heißt **Smartglasses-0.2.0-display-fix-Wemos-Update.zip**. Der Zusatz `display-fix` und die Prüfsummen unterscheiden es vom bereits installierten Paket. BLE-Protokoll und Manifestversion bleiben 0.2.0, damit deine Android-App 1.1.0 weiter funktioniert.

## Installation mit deinem bereits vorbereiteten Wemos S2 Mini

1. Entpacke das neue ZIP in einen **neuen Ordner**, beispielsweise `C:\Users\Toaster\Downloads\Smartglasses-Display-Fix`. Behalte den bisherigen Programmer-Ordner samt seiner ursprünglichen Sicherung. Verwende für dieses Update die neue `Upgrade.ps1` aus dem neuen Ordner; sie prüft die neue HEX-Datei anhand ihrer eigenen SHA-256.
2. Trenne die Brillenversorgung vollständig und ziehe das USB-Kabel vom Wemos ab, bevor du die vier Leitungen wieder anschließt. Das Ausschalten des Displays per Touch trennt die Brillenversorgung nicht.

   | Wemos S2 Mini | Brillenanschluss |
   |---|---|
   | GND | Pad 4 – GND |
   | GPIO4 | Pad 6 – CLK |
   | GPIO5 | Pad 7 – DIO |
   | GPIO6 | Pad 8 – RST |

   Pad 3 (+3V) bleibt für diesen Anschluss frei. Die Brille erhält ihre eigene Versorgung; der Wemos wird über USB versorgt. Die vorhandene CMSIS-DAP-Programmer-Firmware des Wemos kann weiterverwendet werden.
3. Versorge die Brille normal, warte einige Sekunden bis die Firmware gestartet ist und verbinde dann den Wemos mit USB. Schließe den Web-Flasher sowie andere Programme, die den Wemos verwenden. Ein Verbindungsaufbau unter Reset ist für die Sicherung ungeeignet, weil die laufende Bluetooth-Versionstabelle benötigt wird.
4. Öffne PowerShell im neu entpackten Ordner und führe aus:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Pruefen
   ```

   Erforderlicher Abschluss: `PRUEFUNG UND SICHERUNG ERFOLGREICH`. Dabei wird der **jetzt installierte** Stand einschließlich seiner aktuellen Schlüsselseite in einem neuen Backup-Unterordner gesichert. Die CPU bleibt angehalten. Bei einer Fehlermeldung erst die Verbindung korrigieren; anschließend eine neue Prüfung ausführen.
5. Sobald die Prüfung erfolgreich war, ohne Brillen-Neustart im selben Ordner ausführen:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Upgrade.ps1 -Aktion Installieren
   ```

   Erforderlicher Abschluss: `INSTALLATION UND VERIFIKATION ERFOLGREICH`. Der Ablauf ersetzt Bootloader und Anwendung, prüft die geschriebenen Daten und erhält die aktuelle Schlüsselseite 14. CPU2/FUS und die Option Bytes werden nicht programmiert. Die CPU bleibt absichtlich angehalten.
6. **Jetzt die Brillenversorgung vollständig trennen**, Wemos-USB abziehen und die Programmierleitungen entfernen. Mindestens zehn Sekunden warten, dann die Brille wieder normal versorgen. Nur das Entfernen der Programmierleitungen startet die angehaltene CPU nicht neu.

## Anzeige anschließend prüfen

- Ohne Handy-Verbindung sollte `BLE` über `Wait` an den alten Positionen innerhalb der Linse erscheinen. Bei ausgeschaltetem OLED Pad 1 ungefähr eine Sekunde berühren.
- Mit der vorhandenen Android-App verbinden. Uhr und Datum sollten die gleichen Positionen und Zeichenabstände wie vor dem Firmwarewechsel haben.
- Falls ein neuer Zahlenvergleich erscheint: zuerst die obere, dann die untere Zeile lesen. `123` über `456` bedeutet `123456`. Mit dem Handy vergleichen; Pad 1 eine Sekunde bestätigt, Pad 3 lehnt ab.
- Eine kurze und eine längere Nachricht senden und die Grenzen des Lauftexts prüfen.
- Bei der OTA-Anfrage steht `OTA?` über `1+3-`: Pad 1 stimmt zu, Pad 3 lehnt ab. Der Bootloader zeigt `OTA` über `Pair` beziehungsweise `Link`.

Ein BLE-OTA mit den beigefügten BIN-/JSON-Dateien ersetzt nur die Anwendung. Um auch den Bootloader und dessen Pairing-Anzeige zu korrigieren, ist der oben beschriebene SWD-Ablauf vorgesehen. Die Pixelgrenzen wurden mit den echten Fonts und dem Displaytreiber am Rechner geprüft; die Sichtbarkeit durch deine konkrete Linse muss nach Installation an der Brille kontrolliert werden.
