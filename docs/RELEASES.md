# Firmwareversionen, GitHub-Releases und Rückwechsel

## Installation auf der vorhandenen Brille

1. Zuerst Android-App **1.2.0** über die bisherige App installieren. Die bereitgestellte lokale APK verwendet dieselbe Entwicklungssignatur und behält Einstellungen/Benachrichtigungsfreigaben. GitHub-CI-Debug-APKs sind separat signiert; sie sind keine Austauschpakete für diese lokale APK-Reihe.
2. App öffnen und mit der Brille verbinden. Sie lädt automatisch die Versionsliste aus [GitHub-Releases](https://github.com/DrReVaN/Glasses_V0.1_BLE/releases). „Jetzt nach Updates suchen“ erzwingt die Prüfung; automatische Prüfungen sind auf einmal in sechs Stunden begrenzt.
3. Bei **0.3.0 verfügbar** „Neue Version auswählen“ und „Herunterladen und prüfen“ wählen. Das lädt nur das Paket; auf der Brille wird nichts gelöscht oder installiert.
4. Erst „Update starten“ und die Bestätigung in der App starten OTA. Wie bisher `OTA?` an der Brille mit Pad 1 ungefähr eine Sekunde bestätigen. Brille eingeschaltet lassen; kein Wemos erforderlich, wenn der bereits funktionierende Löschkorrektur-Bootloader installiert ist.
5. Abschlussprüfung und Wiederverbindung abwarten. Ab 0.3.0 vergleicht die App Version, Imagegröße und CRC der gestarteten Anwendung mit dem ausgewählten Paket. Danach muss „Installierte Firmware: 0.3.0“ erscheinen. Uhr/Datum bleiben unverändert; Nachrichtentext X=6, sechs Zellen, eigene €-/Latin-1-Glyphen.

„Später / diese Version zurückstellen“ verschiebt das Angebot für diese konkrete Version. Die Versionsauswahl bleibt erreichbar; die manuelle Prüfung hebt die Zurückstellung auf. Eine spätere höhere Version wird erneut angeboten. Eine automatische Installation gibt es nicht.

## Auf eine ältere Version zurückgehen

1. „Verfügbare Versionen / Zurücksetzen“ öffnen und die gewünschte ältere Version wählen.
2. Paket herunterladen und prüfen. Bereits geprüfte Downloads und die letzte Versionsliste bleiben im privaten App-Speicher erhalten und lassen sich auch ohne Internet verwenden.
3. „Update starten“ zeigt ausdrücklich den Rückwechsel zur ausgewählten Versionsnummer. Bestätigen und den vollständigen OTA-Upload durchführen.

**0.2.0** archiviert exakt den zuvor OTA-erfolgreichen Löschkorrektur-Stand `f55c081` (35676 Bytes, SHA-256 `0a83823a101f77a41048a1f020ce46e6cd29c149104a56febe2e66f6ce9bfb79`). Dieser Stand hatte noch keine eindeutige Geräte-Buildkennung und noch nicht den folgenden Text-Fix. Ältere anders benannte 0.2.0-Pakete werden nicht als gleichwertige Release-Assets vermischt. Nach einem Rückwechsel meldet die App diese Einschränkung. Version 0.3.0 bleibt anschließend erneut auswählbar.

Der Rückwechsel ist ein regulärer OTA-Upload. Die Brille besitzt nur einen Anwendungsslot; nach begonnenem Löschen kann sie eine abgebrochene Installation nicht automatisch zur vorherigen Anwendung zurückrollen. Ein vollständiger neuer Upload im Bootloader oder die vorhandene Wemos-Wiederherstellung ist dann nötig. Bootloader, CPU2/FUS, Optionen und Geräteschlüssel werden durch die Release-BIN nicht ersetzt.

## Versionsdefinition und Veröffentlichung

`Core/Inc/glasses_version.h` ist die einzige Quelle der numerischen Releaseversion (Major.Minor.Patch, Komponenten 0–65535). Neue Firmware trägt bei Imageoffset `0x140` einen zwölf Byte langen `SGV1`-Block: drei uint16 little-endian, OTA-Protokollformat 1 und Imageprofil. Das Manifest muss dazu passen. Prüfsummen allein könnten eine falsch bezeichnete Versionsnummer nicht erkennen.

Die vorhandene Firmware-Lesecharakteristik hat weiterhin dieselbe UUID und denselben Handle. Ihre ersten vier Bytes bleiben das Discovery-Protokoll 0,2,0 und App-/Bootmodus. Neu sind sechs Versionsbytes, Format 1, ein reserviertes Nullbyte und je uint32 Imagegröße/CRC; zusammen zwanzig Bytes. Es wird keine zusätzliche GATT-Charakteristik eingefügt, damit vorhandene Bindungen und Handles erhalten bleiben. App 1.2.0 akzeptiert vier- und zwanzig-Byte-Antworten; App 1.1.2 muss vor dem Umstieg ersetzt werden.

Die CI baut und prüft die Images auf dem Dev-Branch. Erst nach erfolgreichen Tests veröffentlicht sie vollständige GitHub-Release-Paare `Smartglasses-X.Y.Z-OTA.bin/.json`, ZIP, Anleitung und SHA256SUMS. Tags/Releases werden niemals auf andere Builds verschoben oder mit anderen Assets überschrieben. Änderungen für eine neue Veröffentlichung benötigen eine erhöhte Versionsnummer. Auf dem Dev-Branch sind Releases als Entwicklungsversionen markiert; die App zeigt das an. `main` bleibt unverändert.

Die App liest die öffentliche [GitHub Releases API](https://docs.github.com/en/rest/releases/releases), sortiert Versionskomponenten numerisch und akzeptiert nur vollständige Paketpaare mit passenden Release-/Dateinamen aus diesem Repository. HTTPS, begrenzte Downloadgrößen, GitHub-Asset-Digests und SHA-256/CRC/Startvektor-/Ziel-/Versionsprüfungen sichern die Paketkonsistenz. Persönliche Benachrichtigungen werden nicht an GitHub gesendet. Die Prüfsummen sind kein unabhängiges Signatursystem: Die Veröffentlichung vertraut dem Repository und GitHub-HTTPS.
