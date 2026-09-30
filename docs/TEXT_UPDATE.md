# Sonderzeichen und ausgerichteter Nachrichtentext per OTA

Diese Anwendung ergänzt € und druckbare Latin-1-Glyphen, unter anderem Umlaute, ß, é/è/ç/ñ, £, ¥ und °. Gängige Unicode-Anführungszeichen, Striche, Leerzeichen und zerlegte Akzente werden lesbar dargestellt. Nicht enthaltene Zeichen, etwa Emoji, erscheinen als Kästchen. Der 7×10-Font bleibt gleich groß; Lauftext beginnt jetzt wie die Uhr bei X=6. Sechs statt fünf Zeichen passen in dieselbe Zeile bei Y=61 und bleiben innerhalb der bisherigen Bildgrenzen. Uhr und Datum ändern sich nicht.

Der Besitzer hat nach Installation der Löschkorrektur `f55c081` einen erfolgreichen OTA-Durchlauf bestätigt. Mit diesem Bootloader genügt **dieses Anwendungsupdate über OTA**. Wemos und Programmierleitungen bleiben entfernt. Die vorhandene Android-App 1.1.2 sendet bereits UTF-8 und braucht für diesen Fix kein Update.

1. Entpacke `Smartglasses-0.2.0-text-fix-OTA.zip` und kopiere beide Dateien in einen neuen Handyordner:
   - `Smartglasses-0.2.0-text-fix-OTA.bin`
   - `Smartglasses-0.2.0-text-fix-OTA.json`
2. Verbinde App 1.1.2 mit der normal gestarteten Brille. Wähle BIN und JSON neu aus. Kontrolliere die Byteanzahl anhand von `size` in der JSON-Datei. BLE- und Manifestversion bleiben zur App-Kompatibilität bei 0.2.0; der Dateiname und die Prüfsummen unterscheiden das neue Paket.
3. Starte OTA und bestätige `OTA?` mit Pad 1 für ungefähr eine Sekunde. Danach loslassen und Brille eingeschaltet lassen.
4. Warte auf Abschlussprüfung und Wiederverbindung mit „Verbunden und bereit“. Prüfe zunächst, dass Uhr und Datum an der bisherigen Position stehen.
5. Sende beispielsweise `Preis: 12,50 €; Grüße aus Österreich – café 23°C`. Kontrolliere € und Umlaute, den gemeinsamen linken Beginn mit der Uhr, sechs sichtbare Zeichen und alle Scrollpositionen durch die Linse.

Bei einem Fehler die vollständige App-Meldung und „Brillenstatus lesen“ vor einem Stromneustart notieren. Ein tatsächlich unterbrochener Transfer benötigt einen vollständigen neuen Upload. Das bewährte Wemos-Löschkorrekturpaket und die Sicherungen für eine Wiederherstellung aufbewahren.

Fontdaten sind aus dem vorhandenen ASCII-Font und eigenen Pixelzeichnungen erzeugt. `python tools/font7x10.py --check` prüft die reproduzierbaren Daten. Rechnerprüfungen erfassen UTF-8-Fragmentgrenzen, Akzentkomposition, Begrenzung, alle 97 Font-Erweiterungen und echte SPI-Framebuffer. Die Sichtbarkeit dieser neuen Glyphen auf der konkreten Brille ist noch durch den Besitzer zu bestätigen.
