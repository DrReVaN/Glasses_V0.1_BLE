# Bestückte Hardware und Firmwareprofil

## Bestätigte Brille

Die vom Besitzer bereitgestellten Fotos zeigen die Aufschrift `V1_SmartGlassesControlBoard`. Der Besitzer bestätigt **STM32WB35**; die ablesbare MCU-Beschriftung passt zu **STM32WB35CE** und damit zum vorhandenen CubeMX-/Keil-Projekt. Das Firmwareprofil bleibt STM32WB35CE mit 512 KiB Flash. Vor Erstinstallation vollständige Gerätekennung und tatsächliche Flashgröße per SWD auslesen.

Der Besitzer bestätigt, dass die ursprüngliche Firmware auf dieser Brille einschließlich Display funktionierte. Der genaue Displaytyp und die sichtbare Pixelauflösung sind unbekannt. Die Flex-Leiterbahn-Beschriftung auf den Fotos reicht nicht zur eindeutigen Identifikation eines Datenblatts aus.

## Abweichung zur veröffentlichten Konstruktion

Der [Hackster-Beitrag](https://www.hackster.io/team-smart-glasses/diy-smart-glasses-20a2bf) unterscheidet V1 und V2. Das dort verlinkte [Controller-V2-Archiv](https://hacksterio.s3.amazonaws.com/uploads/attachments/1397677/v2_smartglassescontrollerboard_MLbrKrnEFj.zip) enthält Eagle-Schaltplan, Board und Stückliste mit `STM32WB55CGU6` und dem Display-Footprint `OLED-0.66-64X48`. Diese Angaben beweisen nicht die Bestückung der fotografierten V1-Platine. MCU-Profil und CPU2-Stack deshalb nicht anhand dieser V2-Stückliste wechseln.

Im V2-Schaltplan stimmen die folgenden Signalzuordnungen mit dem ursprünglichen und aktuellen Firmware-Projekt überein. Der Vergleich ist keine Messung oder vollständige Prüfung der V1-Leiterplatte:

| Signal | Firmware / V2-Netz |
|---|---|
| CAP1203 SCL / SDA | PB8 / PB9 |
| CAP1203 ALERT# | PA1 |
| OLED SCK / MOSI | PA5 / PA7 |
| OLED CS / D/C / RESET | PA4 / PA3 / PA2 |
| OLED-Wandlerfreigabe | PB5 |
| Vibrations-PWM | PA10, TIM1_CH3 |
| Batterieteiler-Freigabe | PA12 |

Die V2-Netzliste verbindet `OLED_PWR` mit dem Shutdown-Eingang des AP3012-Wandlers; die OLED-Logik liegt an der dauerhaften 3-V-Versorgung. Daraus darf für V1 ohne Schaltplan/Messung keine vollständige Trennung der Displayversorgung abgeleitet werden.

## Kompatibilität mit dem funktionierenden OLED-Code

Verglichen wurde der aktive Treiber `MDK-ARM/ssd1306.c` aus dem ursprünglichen Firmware-main `4b314ee3e5207037aa04d2b64c7a0a70b461ff6c` mit dem Dev-Stand. Erhalten bleiben:

- SPI1 und die bisherigen GPIO-Zuordnungen, SPI-Modus und Taktteiler.
- Die vollständige OLED-Initialisierungs-Befehlsfolge, Adressierung, Multiplex-/COM-Konfiguration und deaktivierte interne Charge-Pump-Befehle.
- Vertikale/horizontale Spiegelung und die gedrehte Pixelabbildung.
- Der bisherige **128×64-RAM-Puffer**. Diese Softwaredefinition ist kein Nachweis der sichtbaren Panelauflösung.
- Kontrast 50 nach Initialisierung und 100 ms Anlaufzeit nach Freigabe des OLED-Wandlers; die Anlaufzeit wird nun im Vordergrund ohne lange blockierende Pause verwaltet.
- Lauftextfenster mit fünf Zeichen in `Font_7x10` bei y=61. Die neuen Pairing-Ziffern und Bestätigungshinweise verwenden die kleinere `Font_6x8`, damit sie weniger horizontale Fläche belegen.

Gezielt geändert wurden Busfehlerbehandlung, endliche Übertragungsfristen, ein definierter 1-ms-RESET-Puls und Grenzen für Pixel/Fonts. Diese Änderungen sowie Pairing-Anzeige, ausgeschalteter Zustand und erneuter Displaystart müssen an der Brille geprüft werden.

## Verbleibende Prüfung

Den laufenden V1-Stand vor Erstinstallation sichern. MCU-Kennung, 512-KiB-Flash und CPU2-/SFSA-Werte auslesen. Danach zunächst Uhr, Datum, Lauftext, sechs vollständig lesbare Pairing-Ziffern und die Bestätigungshinweise prüfen. Erst mit funktionierender Anzeige und Touch-Bestätigung das erste BLE-OTA durchführen; Vorgehen in [OTA.md](OTA.md).
