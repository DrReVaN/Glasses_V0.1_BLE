# BLE-Protokoll und Bedienung

Alle UUIDs bleiben für Info- und Empfangsdienste kompatibel. Schreibzugriffe verwenden jetzt **ATT Write Request mit Antwort** und verlangen 16-Byte-Verschlüsselung, authentifiziertes Secure Connections Pairing und Bonding. Die alte App mit unbeschränkten Write-Without-Response-Versuchen entsprechend aktualisieren.

| Funktion | UUID | Daten |
|---|---|---|
| Geräteinfo-Service | 00000010-cc7a-482a-984a-7f2ed5b3e58f | Read-Dienst |
| Firmware | 00000011-8e22-4541-9d4c-21edae82ed19 | 4 Bytes: 0, 2, 0, Modus (0 App / 1 Bootloader) |
| Gerätename | 00000012-8e22-4541-9d4c-21edae82ed19 | ASCII |
| Diagnose | 00000013-8e22-4541-9d4c-21edae82ed19 | 5 uint32 little-endian: RCC-Resetflags, Fault-Code, verworfene RX, verlorene Queue-Nachrichten, gekürzte Texte; geschütztes Lesen |
| Empfangs-Service | 00000020-cc7a-482a-984a-7f2ed5b3e58f | Schreibdienst |
| Uhr | 00000021-8e22-4541-9d4c-21edae82ed19 | ASCII HHmmddMMyyyy, alternativ altes HHmmddMM |
| Nachricht | 00000022-8e22-4541-9d4c-21edae82ed19 | Fragmentindex, Fragmentanzahl, 1–18 UTF-8-Bytes |
| Boot-Anfrage | 00000023-8e22-4541-9d4c-21edae82ed19 | ASCII OTA1; verlangt zusätzliche Bestätigung an der Brille |
| OTA-Service | 0000fe20-cc7a-482a-984a-7f2ed5b3e58f | gleiche Attribute in beiden Images; Upload nur im Bootloader erlaubt |
| OTA Begin | 0000fe21-8e22-4541-9d4c-21edae82ed19 | SGU1 + uint32 Größe + uint32 CRC32, little-endian |
| OTA Data | 0000fe22-8e22-4541-9d4c-21edae82ed19 | uint32 Byte-Offset + 1–16 Image-Bytes |
| OTA End | 0000fe23-8e22-4541-9d4c-21edae82ed19 | ASCII END1 |

## Nachrichten

Index beginnt bei 0, Anzahl liegt bei 1–14 und bleibt in allen Fragmenten gleich. Alle Fragmente außer dem letzten enthalten genau 18 Nutzbytes. Der Empfangspuffer ist 252 Bytes groß. Null-Padding ist ausschließlich am Ende des letzten Fragments zulässig. Leere Nachrichten, ungültige UTF-8-Sequenzen, fehlende Fragmente, Duplikate und wechselnde Anzahl werden verworfen. Fragment 0 beginnt eine neue Übertragung. Nach drei Sekunden ohne Folgedaten oder Disconnect wird der unvollständige Empfang zurückgesetzt.

Erst nach vollständigem Empfang wird der Text für die Anzeige freigegeben. Eine Queue hält vier fertige Nachrichten. Bei voller Queue wird die älteste wartende Nachricht entfernt; die aktuell angezeigte bleibt stabil. Der Diagnosezähler protokolliert dies.

Das OLED besitzt ASCII-Fonts. ä/ö/ü/Ä/Ö/Ü/ß werden als ae/oe/ue/Ae/Oe/Ue/ss angezeigt. Andere Unicode-Zeichen erhalten jeweils ein `?`, Steuerzeichen ein Leerzeichen. Der fertige Displaytext wird auf 127 Zeichen begrenzt und immer nullterminiert; Kürzungen werden gezählt.

## Uhr

Die neue App sendet Jahr und Datum beim Verbindungsaufbau und anschließend jede Minute. Die Brille führt die Uhr unabhängig davon mit HAL_GetTick weiter. Diese Zeitbasis läuft im verwendeten CPU1-Sleep mit SysTick weiter; Stop/Standby sind deshalb deaktiviert. Nach vollständigem Spannungsverlust ist eine neue Synchronisierung nötig. Es erfolgt keine automatische Sommerzeit-/Zeitzonenänderung ohne Handy.

Beim alten Acht-Ziffern-Format fehlt das Jahr. Dafür verwendet die Firmware einen unbekannten Kalenderjahrwert; Februar kann 29 Tage haben. Für korrekte Schaltjahre das neue Zwölf-Ziffern-Format verwenden.

## Bedienung und Fehler

Die sechs Ziffern des Zahlenvergleichs erscheinen wegen des begrenzten optischen Bereichs auf zwei Zeilen: zuerst die drei Ziffern oben, dann die drei unten lesen. `123` über `456` entspricht dem Handy-Code `123456`. Bei der OTA-Bestätigung steht unter `OTA?` die Kurzform `1+3-` für Pad 1 zustimmen und Pad 3 ablehnen. Die Displaykorrektur ändert weder BLE-Protokoll noch Paketversion 0.2.0; die bestehende Android-App 1.1.0 bleibt kompatibel.

- Pad 1 eine Sekunde: Uhr/Home; im Zahlenvergleich oder bei OTA-Anfrage zustimmen.
- Pad 2 ungefähr 2,3 Sekunden: Anzeige und Vibration aus/ein. BLE bleibt aktiv, CPU1 schläft zwischen kurzen Arbeitsschritten. OFF wird nur einmal beim Erreichen der Haltefrist ausgelöst; Loslassen ist vor der nächsten Aktion erforderlich.
- Pad 3 eine Sekunde: laufenden Text von vorn lesen; Pairing/OTA ablehnen.
- Pads 1 und 3 gleichzeitig drei Sekunden: Pairing-Fenster für 60 Sekunden öffnen. Ein Fenster ist ebenfalls für die ersten 60 Sekunden nach Start geöffnet. Jeder neue Zahlenvergleich muss trotzdem physisch bestätigt werden.
- Das OLED schaltet nach 15 Sekunden ohne Anzeigeaktivität ab. Neue fertige Nachrichten oder Touch wecken es. Während des Zahlenvergleichs bleibt es an.

CAP1203 wird alle 50 ms einmal abgefragt, nach Fehlern mit einer Sekunde Pause neu initialisiert. ALERT# nutzt bei direkter Verbindung eine fallende Flanke. SPI/I2C-Zugriffe haben endliche Fristen; Displayfehler lösen eine begrenzte Neuinitialisierung aus. Die 1-ms-Reset-Pulsdauer des OLED und Boot-/Schlüsselbereitstellung sind die einzigen absichtlichen kurzen Initialisierungswartezeiten.

ATT-Fehler: 0x03 außerhalb des erlaubten Betriebsmodus, 0x09 bei belegtem OTA-Empfänger, 0x0D bei ungültigem Paket und 0x0E bei Update-/Flash-/Prüfsummenfehlern.
