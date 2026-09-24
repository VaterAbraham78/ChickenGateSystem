/******************************************************************/
/***															***/
/***	ChickenGateSystem Rev.E		- im Code nachtragen		***/
/***	Korrekturversion: V34		- im Code nachtragen		***/
/***															***/
/***															***/
/***	Sleepmode fuer Energieeinsparung (noch nicht umgesetzt)	***/
/***	Reset-Taster: Mehrfachfunktion (Debug / Reset / Nextion	***/
/***	Debug-Mode fuer serielle Ausgabe bei CPU-Start waehlbar	***/
/***	EEPROM-Magic-Byte fuer Plausibilitaetspruefung			***/
/***	Pruefung & Korrektur von HMI-Grenzwertparametern		***/
/***	Sensorspeisung KEIN bistabiles Relais mehr				***/
/***	Statusmeldungen via HMI-Anzeigefelder					***/
/***	Alarmzustaende als Bitmaske an HMI senden				***/
/***	zusaetzliche Tastereingabe "Licht Stall" via HMI-Button	***/
/***	"Licht Stall" mit Hardware-PWM umgesetzt				***/
/***	Fehler PWM-Dimmstufe behoben (map()						***/
/***	UART-Kommunikation Nextion-Touchpanel angepasst			***/
/***	Nextion-Kommunikation umgebaut - NICHT abwaertskompatibel**/
/******************************************************************/


/******************************************************************************************************/
/******************************************************************************************************/
/***	BIBLIOTHEKEN	***/

#include <EEPROM.h>							// Einbinden der EEPROM-Bibliothek fuer remanente Speicherung der Parameter
#include <avr/sleep.h>						// Sleepmode-Funktionen (sleep_cpu(), sleep_enable() etc.)
#include <avr/wdt.h>						// Watchdog-Timer-Funktionen fuer periodisches Aufwecken aus dem Sleepmode


/******************************************************************************************************/
/******************************************************************************************************/
/***	DEKLARATIONEN	***/

const byte INADaylight = A0;   				// Analogwert Messung "Tageslicht"			-> Integerwert 0..1024
const byte INAMotfuse = A4;  				// Analogwert Messung "RM Motorsicherung"	-> Integerwert 0..1024
const byte INABatterie = A5;  				// Analogwert Messung "Batteriespannung"	-> Integerwert 0..1024

const byte INInterrupt = 2;					// Sammel-Interrupt fuer Sleepmode (INT0) - NICHT in Array "arrPINIn" eingebunden

const byte INSafety1 = 3;					// Infrarotsensor "Safety-1-Innen"
const byte INSafety2 = 4;   				// Infrarotsensor "Safety-2-Aussen"
const byte INTstTorAuf = 5;  				// Taster "Tor AUF"
const byte INTstTorZu = 6;   				// Taster "Tor ZU"
const byte INTstLicht = 7;					// Taster "Licht Stall"
const byte INTstMultiReset = 8;    			// Taster "Multifunktion-Reset"
const byte OUTMotAuf = 9;					// Motorbefehl "Tor AUF"					(bistabiles Relais mit 2 Spulen)
const byte OUTMotZu = 10;   				// Motorbefehl "Tor ZU"						(bistabiles Relais mit 2 Spulen)
const byte OUTLicht = 11;   				// Beleuchtung "Licht Stall" einschalten
const byte OUTAlarm = 12;     				// Signal-LED "Alarm"
const byte OUTPowOn = 13;  					// "Torsensoren/partielle Motorspeisung" einschalten
const byte OUTNextionPower = A2;			// Einschalten Nextion-HMI-Speisung - A2 als Digital-Output verwendet

const byte arrPINIn[] = {INSafety1, INSafety2, INTstTorAuf, INTstTorZu, INTstLicht, INTstMultiReset};// Array "arrPINIn" definieren und initialisieren
const byte anzahlPINIn = sizeof(arrPINIn);														// Arraygroesse "arrPINIn" bestimmen (zwingend eine Konstante)
const byte arrPINOut[] = {OUTMotAuf, OUTMotZu, OUTAlarm, OUTPowOn};								// Array "arrPINOut" definieren und initialisieren (OUTLicht als HW-PWM, daher NICHT im Array)
const byte anzahlPINOut = sizeof(arrPINOut);													// Arraygroesse "arrPINOut" bestimmen (zwingend eine Konstante)

struct strINPUT	{																				// Struktur-Architektur fuer Eingaenge definieren
	bool Safety1;
	bool Safety2;
	bool TstTorAuf;
	bool TstTorZu;
	bool TstLicht;
	bool TstMultiReset;
}	inputs = {false, false, false, false, false, false};										// 		...Struktur-Variable "inputs" erstellen und initialisieren
struct strTSTFUNC	{																			// Struktur-Architektur fuer Taster "Multifunktion-Reset" definieren
	bool Reset;																					// 		...Tasterfunktion "Alarm-Reset" aus "inputs.TstMultiReset" erkannt
	bool Nextion;																				// 		...Tasterfunktion "Nextion-HMI" aus "inputs.TstMultiReset" erkannt
}	TstFunc = {false, false};																	// 		...Struktur-Variable "TstFunc" erstellen und initialisieren
struct strOUTPUT	{																			// Struktur-Architektur fuer Ausgaenge definieren
	bool MotAuf;
	bool MotZu;
	bool Licht;
	bool Alarm;
	bool PowOn;
}	outputs = {false, false, false, false, false};												// 		...Struktur-Variable "outputs" erstellen und initialisieren
struct strBLINK	{																				// Sruktur-Architektur fuer verschiedene Blinkzeiten definieren
	unsigned long MainOn;
	unsigned long MainOff;
	unsigned long BattOn;
	unsigned long BattFirstOff;
	unsigned long BattSecondOff;
} blinktime = {800, 400, 200, 2500, 400};														// 		...Struktur-Variable fuer "blinktime" erstellen und initialisieren

const unsigned long prellTime = 20;   		// Entprellzeit fuer die Eingangssignale   			[in Millisekunden]
const unsigned long photoTime = 60000; 		// Hysteresezeit der Tag/Nacht-Umschaltung   		[in Millisekunden]
const unsigned long motfuseAlaTime = 2000;	// Alarmverzoegerung "RM Motorsicherung" hat ausgeloest		[in Millisekunden]
const unsigned long battAlaTime = 5000;		// Alarmverzoegerung "Batterieladung" zu tief		[in Millisekunden]
const unsigned long relaisTime = 500;		// Relais-Ansteuerzeit für die bistabilen Relais	[in Millisekunden]
const unsigned long driveTime = 28000;		// maximale Fahrzeit vom Tor bis Endlage erreicht sein muss
const unsigned long waitTime = 30000;		// zusaetzliche Wartezeit zur maximalen Fahrzeit vom Tor wenn ein "SafetyUp" ausgeloest wurde
const unsigned long debugBootTime = 3000;	// Notwendige Haltezeit des Reset-Tasters bei Controllerstart	[in Millisekunden]

const int DEF_GWVALNACHT = 100;				// Default Grenzwert Nacht-Status	[Integerwert]
const int DEF_GWVALTAG = 300;				// Default Grenzwert Tag-Status		[Integerwert]
const int MIN_GWBEREICH = 0;				// allgemein min. Analogwert		[Integerwert]
const int MAX_GWBEREICH = 1024;				// allgemein max. Analogwert		[Integerwert]
const int MIN_DIMMLEVEL = 1;				// min. PWM-Dimmstufe		[in %]
const int MAX_DIMMLEVEL = 100;				// max. PWM-Dimmstufe		[in %]
const int DEF_DIMMLEVEL = 50;				// Default PWM-Dimmstufe	[in %]
const int MIN_LIGHTTIME = 2;				// min. Einschaltdauer		[in Sekunden]
const int MAX_LIGHTTIME = 1800;				// max. Einschaltdauer		[in Sekunden]
const int DEF_LIGHTTIME = 10;				// Default Einschaltdauer	[in Sekunden]
											
int gwValueNacht = DEF_GWVALNACHT;			// HMI-Grenzwertvorgabe fuer Helligkeit "NACHT"
int gwValueTag = DEF_GWVALTAG;  			// HMI-Grenzwertvorgabe fuer Helligkeit "TAG"
int gwHystValue = 50;						// Hysteresebreite fuer Tag/Nacht-Umschaltung
int lightvalue = 0;							// aktueller Lichtwert "Tageslicht"

bool stateTag = true;    					// Status "Tag" beim Start auf "TRUE" initialisieren damit Tor geoeffnet wird
bool debugMode = false;						// Debug-Modus bei Controllerstart auswerten		[TRUE=SerialMonitor / FALSE=Nextion-HMI]

int dimmlevel = DEF_DIMMLEVEL;				// PWM-Dimmstufe "Licht Stall"						[0..100%]
int lighttime = DEF_LIGHTTIME;				// maximale Einschaltzeit "Licht Stall"				[in Sekunden]

const int averageCnt = 10;					// Anzahl Messzyklen fuer Mittelwertbildung
const unsigned long samplingTime = 500;		// Abtastrate der Analogmessungen (Tageslicht/Motorsicherung/Batterie)	[in Millisekunden]
const unsigned long holdTimeNextion = 3000;	// Min.Haltezeit Multifunktion-Reset-Taster zur Laufzeit, um Nextion einzuschalten	[in Millisekunden]
int motfuseRaw = 0;							// aktueller Rohwert "RM Motorsicherung"
int gwMotfuseRaw = 546;						// Grenzwert "RM Motorsicherung"					[546=8.00V = Sicherung ausgeloest]
int batterieRaw = 0;						// aktueller Rohwert "Batteriespannung"
int gwBatterieRaw = 810;					// Grenzwert "Batteriespannung tief"				[810 = 11.85V = 30%]
int batterieProzent = 0;					// Batterieladung in Prozent						[SOC, aus "batterieRaw" abgeleitet]

int cntSafetyFail = 0;						// laufender Zaehler "Fahrfehler Tor"
int gwSafetyFail = 3;						// Grenzwert Anzahl erlaubter "Fahrfehler Tor" bis Alarm ausgeloest wird

bool skAlarm = false;						// Alarmstatus "Schrittketten-Ablaufstoerung"
bool safetyAlarm = false;					// Alarmstatus "Fahrfehler Tor" ausgeloest
bool motfuseAlarm = false;					// Alarmstatus "RM Motorsicherung" (Sicherung ausgeloest)
bool batterieAlarm = false;					// Alarmstatus "Batterieladung tief"

enum SK_TOR {STANDBY, AUTOAUF, AUTOZU, HANDAUF, HANDZU};		// ENum-Definition der SK "Torsteuerung"
	SK_TOR schrittTor = STANDBY;			// Variable "schrittTor" dem ENum zuweisen und Variable initialisieren
	
const byte ADDR_GWTAG_LO = 0;				// EEPROM-Adresse: "GW-TAG" LowByte
const byte ADDR_GWTAG_HI = 1;				// EEPROM-Adresse: "GW-TAG" HighByte
const byte ADDR_GWNACHT_LO = 2;				// EEPROM-Adresse: "GW-NACHT" LowByte
const byte ADDR_GWNACHT_HI = 3;				// EEPROM-Adresse: "GW-NACHT" HighByte
const byte ADDR_LIGHTTIME_LO = 4;			// EEPROM-Adresse: "EINSCHALTDAUER Licht Stall" LowByte
const byte ADDR_LIGHTTIME_HI = 5;			// EEPROM-Adresse: "EINSCHALTDAUER Licht Stall" HighByte
const byte ADDR_DIMMLEVEL = 6;				// EEPROM-Adresse: "DIMMSTUFE Licht Stall" (0..100%, passt in 1 Byte)
const byte ADDR_EEPROM_MAGIC = 7;			// EEPROM-Adresse: Gueltigkeits-Erkennungsbyte
const byte EEPROM_MAGIC_VALUE = 0xA5;		// Erkennungswert der Speicherstruktur
											// Naechste freie Adresse: 8

const byte anzahlZeilenanzeige = 11;		// Anzahl Debug-Zeilen insgesamt (fuer Round-Robin-Funktion)											
unsigned long serMonitorTime = 1000;		// Anzeigefrequenz im seriellen Monitor (Debug-Mode)[in Millisekunden]
unsigned long debugStepTime = serMonitorTime / anzahlZeilenanzeige;		// Zeitabstand je Einzelzeile um Serial-Blockade zu verhindern

unsigned long cycleTime = 0;				// aktuelle Zykluszeit								[in Microsekunden]

// Hilfsvariablen fuer den Sleepmode
unsigned long vulLastNextionActivity = 0;	// Zeitpunkt der letzten Seitenaenderung/Parameteruebermittlung des Nextion-HMI
const unsigned long nextionIdleTime = 60000;// Minimale Inaktivitaetszeit des Nextion-HMI bevor Sleep erlaubt ist	[in Millisekunden]
volatile bool wdtWoke = false;				// TRUE = durch WDT-Timeout (~8s) geweckt, FALSE = durch Sammelinterrupt (vorzeitig) geweckt
volatile bool interruptWoke = false;		// ISR fuer den externen Hardware-Interrupt auf steigende Flanke
bool sleepBlockDaylight = false;			// Platzhalter fuer spaeteren Schritt der Tageslichtmessung mit echter Logik befuellt (5s/photoTime-Fenster)
bool motfuseAlarmPending = false;			// Spiegelt "vxState" aus motfuse() - Alarmverzoegerung laeuft
bool batterieAlarmPending = false;			// Spiegelt "vxState" aus batterie() - Alarmverzoegerung laeuft


/******************************************************************************************************/
/******************************************************************************************************/
/***	NEXTION-TOUCHDISPLAY	***/

		// component-ID (byte, bei Touch-Events zurueckgemeldet) und component-Name (String, fuer "get"/"set"-Textbefehle)... 
		// ...muessen mit dem tatsaechlichen Nextion-Projekt (im Nextion Editor: Attribut "id" bzw. Objektname jeder Komponente) uebereinstimmen
		// WICHTIG: Component-IDs werden vom Nextion-Editor pro Seite ab 0 automatisch vergeben und koennen seitenuebergreifend identisch sein...
		// ...die Auswertung in nexFrameAuswerten() prueft deshalb IMMER Seite UND ID gemeinsam!
		
		// WICHTIG: "editingId" (globale Nextion-Variable) enthaelt die ID des zuletzt per Touch-Press beruehrten Zahlenfeldes
		// ...wird in dessen Postinitialize direkt mit Seite+ID+Wert per 0x73 uebertragen (siehe nexFrameAuswerten()), kein "get" mehr noetig!
		
const byte NEX_PAGE_MAIN = 0;				// Seite "pMain"
const byte NEX_PAGE_DAYLIGHT = 1;			// Seite "pDaylight"
const byte NEX_PAGE_BOXLIGHT = 2;			// Seite "pBoxlight"
const byte NEX_PAGE_SYSTEM = 3;				// Seite "pSystem"
const byte NEX_PAGE_INPUTS = 4;				// Seite "pInputs"


const byte NEX_CID_GWTAG = 4;				// Nextion-ID Eingabefeld "Grenzwert Tag"
const byte NEX_CID_GWNACHT = 5;				// Nextion-ID Eingabefeld "Grenzwert Nacht"
const byte NEX_CID_DIMM = 4;				// Nextion-ID Eingabefeld "Dimmstufe Licht Stall"
const byte NEX_CID_LIGHTTIME = 6;			// Nextion-ID Eingabefeld "max.Einschaltdauer Licht Stall"
const byte NEX_CID_LICHTTOGGLE = 7;			// Nextion-ID Eingabefeld "Button Licht Stall"

const char NEX_NAME_GWTAG[] = "pDaylight.nb104";				// Objektname Eingabefeld "Grenzwert Tag"
const char NEX_NAME_GWNACHT[] = "pDaylight.nb105";				// Objektname Eingabefeld "Grenzwert Nacht"
const char NEX_NAME_DIMM[] = "pBoxlight.nb204";					// Objektname Eingabefeld "Dimmstufe Licht Stall"
const char NEX_NAME_LIGHTTIME[] = "pBoxlight.nb206";			// Objektname Eingabefeld "max.Einschaltdauer Licht Stall"

const char NEX_NAME_ACTREVISION[] = "pMain.tx003";				// Name der Hilfsvariable "Revisionsbezeichnung" (z.B. "F05")
const char NEX_NAME_ACTSTATEALARM[] = "pMain.vaAlarms";			// Name der Hilfsvariable "Signalzustand der Alarmstaten"
const char NEX_NAME_ACTDAYLIGHT[] = "pDaylight.nb103";			// Objektname Anzeigefeld "Rohwert Tageslicht"
const char NEX_NAME_ACTSTATETAG[] = "pDaylight.vaStateTag";		// Name der Hilfsvariable "Tag/Nacht-Status"
const char NEX_NAME_ACTSTATELICHT[] = "pBoxlight.vaLicht";		// Name der Hilfsvariable "Ausgangszustand Licht Stall"
const char NEX_NAME_ACTBATTRAW[] = "pSystem.nb303";				// Objektname Anzeigefeld "Rohwert "Batteriespannung"
const char NEX_NAME_ACTBATTLEVEL[] = "pSystem.nb304";			// Objektname Anzeigefeld "Prozentwert der Batterieladung"
const char NEX_NAME_ACTMOTFUSERAW[] = "pSystem.nb306";			// Objektname Anzeigefeld "Rohwert RM Motorsicherung"
const char NEX_NAME_ACTSTATEMOTFUSE[] = "pSystem.vaMotfuse";	// Name der Hilfsvariable "Alarmzustand Motorsicherung"
const char NEX_NAME_ACTCYCLETIME[] = "pSystem.nb309";			// Objektname Anzeigefeld "Zykluszeit der CPU"
const char NEX_NAME_ACTSTATEINPUT[] = "pInputs.vaInputs";		// Name der Hilfsvariable "Signalzustand der Inputs"

const char REVISION_SCHEMA = 'E';								// Aktuelle Schema-Revision	(Buchstabe, manuell nachfuehren)
const byte REVISION_CODE = 34;									// Aktuelle Code-Revision 	(Zahl 0-99, manuell nachfuehren)	

enum NEX_PARSE_STATE {NEX_WAIT_CMD, NEX_COLLECT_PAYLOAD, NEX_WAIT_TERM, NEX_SKIP_UNKNOWN};	// Zustaende des laengenbasierten Nextion-Parsers
byte currentPage = NEX_PAGE_MAIN;								// aktuell auf dem Nextion angezeigte Seite (per "sendme"/0x66 ermittelt)
byte hmiSendIndex = 0;											// Index innerhalb der aktuellen Seite fuer hmiSend() (seitenuebergreifend zugreifbar, da bei Seitenwechsel in nexFrameAuswerten() zurueckgesetzt)	

bool hmiLichtToggleRequest = false;								// HMI-Taster "Licht Stall" (wirkt wie physischer Taster)
unsigned long hmiSendTime = 1000;								// Sendefrequenz "hmiSend()" in Millisekunden
unsigned long hmiSendStepTime = 150;							// Zeitabstand je Einzelwert um Serial-Blockade zu verhindern
bool hmiReceiveInProgress = false;								// true, waehrend ein Telegramm empfangen wird (aber noch nicht fertig ausgewertet wurde)
unsigned long hmiReceiveStartTime = 0;							// Zeitpunkt, an dem der aktuelle Telegrammempfang begonnen hat
const unsigned long hmiReceiveTimeout = 200;					// Sicherheitsabschaltung falls ein Telegramm nie fertig eintrifft [ms]


/******************************************************************************************************/
/******************************************************************************************************/
/***	FUNKTIONSPROTOTYPEN	***/

		// Wenn andere Entwicklungsumgebung als Arduino IDE verwendet wird. Ohne diese Prototypen keine korrekte Kompilierung.
		// Muss bei Funktionsaenderungen korrekt nachgefuehrt werden !!!

/*** Ausserhalb loop	***/
void isrInterrupt();
void wdtSetup8s();												// Watchdog-Timer auf ~8s Interrupt-Modus konfigurieren (kein Reset)
void sleepNow();												// CPU in Power-down-Sleepmode versetzen
bool sleepAllowed();											// Freigabe Sleepmode - prueft alle Ausschlusskriterien
void checkDebugMode();
void speicherRead();
void speicherWrite();
void checkGwBereich();
void checkMinMax(int &viInput, int viGwLow, int viGwHigh);

/*** Nextion-HMI	***/
byte nexErwarteteLaenge(byte cmd);
void nexFrameAuswerten(byte* frame, byte len);
void nexWertUebernehmen(byte page, byte id, long value);		// Signatur geaendert: page+id Kennung
void nexEnde();
void nexSetValue(const char* compName, long value);
void nexSetText(const char* compName, const char* text);

/*** loop-Ablauf	***/
void entprellen();
void tasterfunktion();
void nextionPower();
void daylight();
void motfuse();
void batterie();
byte battAdcToPercent(int adcWert);
void hmiRead();
void torsteuerung();
void beleuchtung();
void ausgaenge();
void alarmhandling();
byte bitmaskStateInputs();
byte bitmaskStateAlarm();	
void hmiSend();
void displayanzeige();
void cycle();


/******************************************************************************************************/
/******************************************************************************************************/
/***	SETUPCODE	***/

void setup()	{
	checkDebugMode();						// Debug-Mode pruefen
	Serial.begin(9600);						// Serial Port für Anzeige oeffnen
	pinMode(INInterrupt, INPUT);			// Sammel-Interrupt fuer Sleepmode
	pinMode(INSafety1, INPUT);				// Infrarotsensor "Safety-1-Innen"
	pinMode(INSafety2, INPUT);    			// Infrarotsensor "Safety-2-Aussen"
	pinMode(INTstTorAuf, INPUT);			// Taster "Tor AUF"
	pinMode(INTstTorZu, INPUT); 			// Taster "Tor ZU"
	pinMode(INTstLicht, INPUT);				// Taster "Licht Stall"
	pinMode(INTstMultiReset, INPUT);   		// Taster "Multifunktion-Reset"
	pinMode(OUTMotAuf, OUTPUT);   			// Motorbefehl "Tor AUF"					(bistabiles Relais mit 2 Spulen)
	pinMode(OUTMotZu, OUTPUT);    			// Motorbefehl "Tor ZU"						(bistabiles Relais mit 2 Spulen)
	pinMode(OUTLicht, OUTPUT);    			// Licht "Stall" einschalten
	pinMode(OUTAlarm, OUTPUT);    			// Signal-LED "Alarm"
	pinMode(OUTPowOn, OUTPUT);				// Torsensoren/partielle Motorspeisung "einschalten"
	pinMode(OUTNextionPower, OUTPUT);		// Nextion-HMI-Speisung einschalten
		digitalWrite(OUTNextionPower, HIGH);// ...Startzustand EIN, damit bestehendes Verhalten (Nextion immer aktiv) erhalten bleibt
	ACSR |= (1<<ACD);								// Analogkomparator dauerhaft abschalten - wird in diesem Projekt nicht verwendet (AIN0/AIN1)
	DIDR0 |= (1<<ADC0D) | (1<<ADC4D) | (1<<ADC5D);	// Digitale Eingangspuffer der genutzten Analogpins (A0/A4/A5) dauerhaft abschalten
	speicherRead();							// FC "Remanenter Speicher auslesen"
}

/***	HAUPTPROGRAMM	***/

void loop()	{
	entprellen();							// FC "Taster entprellen"
	tasterfunktion();						// FC "Tasterfunktion erkennen"
	nextionPower();							// FC "Nextion-HMI einschalten"
	daylight();								// FC "Messung Tageslicht"
	motfuse();								// FC "Messung RM Motorsicherung"
	batterie();								// FC "Messung Batteriespannung"
	if (debugMode == false)	{
		hmiRead();							// FC "HMI Daten empfangen"
	}
	torsteuerung();							// FC "Torsteuerung"
	beleuchtung();							// FC "Innenbeleuchtung Stall"
	ausgaenge();							// FC "Ausgangsvariablen setzen"
	alarmhandling();						// FC "Alarmhandling" mit Parameteruebergabe der verschiedenen Blinkzeiten
	if (debugMode == false)	{
		hmiSend();							// FC "HMI Daten senden"
	} else {
	displayanzeige();						// FC "Displayanzeige"
	}
	cycle();								// FC "Zykluszeit berechnen"
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	INTERRUPT-ROUTINE	***/

void isrInterrupt()	{
	ADCSRA |= (1<<ADEN);						// ADC wieder einschalten (war waehrend Sleepmode bewusst deaktiviert nicht automatisch abgeschaltet)
	interruptWoke = true;						// Merker-Flag für das Hauptprogramm setzen
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Sleepmode: Watchdog + Ein-/Austritt"	***/

		// WICHTIG: millis()/micros() laufen waehrend SLEEP_MODE_PWR_DOWN NICHT weiter (Timer0 steht still)

ISR(WDT_vect)	{													// Wird bei jedem WDT-Timeout (~8s) automatisch aufgerufen
	wdtWoke = true;													// ...WDT stellt in diesem Fall die Weckquelle dar (im Gegensatz zum Sammelinterrupt)
																	// Wird in einem spaeteren Schritt erweitert (Zaehler zur Verkettung mehrerer 8s-Zyklen fuer laengere Schlafzeiten).
}

void wdtSetup8s()	{
	cli();															// Interrupts kurz sperren (zeitkritische Watchdog-Umkonfiguration)
	wdt_reset();													// Watchdog-Zaehler zuruecksetzen
	MCUSR &= ~(1<<WDRF);											// Watchdog-Reset-Flag loeschen (sonst startet WDT im Reset- statt Interrupt-Modus)
	WDTCSR |= (1<<WDCE) | (1<<WDE);									// Aenderungsmodus freischalten (muss binnen 4 Taktzyklen gefolgt werden)
	WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);						// Watchdog-Interrupt aktivieren und Timeout auf ~8s stellen
	sei();															// Interrupts wieder freigeben
return;
}

void sleepNow()	{													// Zentrale Sleepmode-Ein-/Austrittsfunktion
	ADCSRA &= ~(1<<ADEN);											// ADC VOR dem Schlafenlegen explizit abschalten (siehe isrInterrupt())
	wdtSetup8s();													// Watchdog fuer ~8s-Aufwachzyklus konfigurieren
	wdtWoke = false;												// ...Weckgrund-Flag zuruecksetzen, bevor die CPU schlafen geht	
	interruptWoke = false;											// ...Interrupt-Pin-Flag zuruecksetzen, bevor die CPU schlafen geht	
	attachInterrupt(digitalPinToInterrupt(INInterrupt), isrInterrupt, RISING);		// HW-Interrupt aktivieren
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);							// Stromsparendster Modus (Timer0/millis() steht dabei still!)
	sleep_enable();
	sleep_cpu();													// ...hier schlaeft die CPU, bis WDT-Interrupt ODER Sammelinterrupt sie weckt
	sleep_disable();												// ...ab hier laeuft der Code nach dem Aufwachen normal weiter
	wdt_disable();              									// ...Watchdog deaktivieren damit kein weiterer ausgeführt werden kann
	detachInterrupt(digitalPinToInterrupt(INInterrupt));			// HW-Interrupt deaktivieren
	ADCSRA |= (1<<ADEN);											// ADC wieder einschalten (redundant zu isrInterrupt(), falls durch WDT statt Sammelinterrupt geweckt)
return;
}

bool sleepAllowed()	{												// Sleepmode-Sammelbedingung - alle Punkte muessen erfuellt sein
																	// HINWEIS: speicherRead()/speicherWrite() brauchen keine eigene Pruefung da loop() sie synchron/blockierend aufruft. Dadurch kann der Sleep-Check niemals waehrend eines laufenden EEPROM-Zugriffs erreicht werden.
	if (bitmaskStateInputs() != 0)	{								// Ein digitaler Eingang steht auf HIGH (deckt Entprellung mitten im Wechsel sowie Safety1/2 waehrend Torschliessung implizit mit ab)
		return false;
	}
	if (hmiReceiveInProgress == true)	{							// Ein Nextion-Telegramm wird gerade byteweise empfangen
		return false;
	}
	if ((millis() - vulLastNextionActivity) < nextionIdleTime)	{	// Nextion war innerhalb der letzten 60s aktiv (Seitenwechsel/Parameter)
		return false;
	}
	if (sleepBlockDaylight == true)	{								// Tageslichtmessung laeuft (Details folgen in einem spaeteren Schritt)
		return false;
	}
	if (schrittTor != STANDBY)	{									// Torsteuerung nicht im Ruhezustand (faehrt/wartet/etc.)
		return false;
	}
	if (outputs.MotAuf || outputs.MotZu || outputs.Licht || outputs.Alarm || outputs.PowOn)	{	// Ein Ausgang ist aktiv
		return false;
	}
	if (skAlarm || motfuseAlarm || safetyAlarm || batterieAlarm)	{	// Ein Alarm ist aktiv
		return false;
	}
	if (motfuseAlarmPending || batterieAlarmPending)	{			// Alarm-Verzoegerung laeuft noch (Grenzwert verletzt, Alarm noch nicht ausgeloest)
		return false;
	}
return true;														// Keine der obigen Bedingungen traf zu -> Freigabe Sleepmode
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "DEBUG-MODE: UART-AUSWAHL BEIM BOOTING	***/

		// Taster beim Start NICHT gedrueckt -> sofort Nextion-Modus (kein Zeitverlust).
		// Taster gedrueckt -> sofern "debugBootTime" erreicht wird, wird der Serial-Monitor-Modus fuer Debug-Mode aktiviert.

void checkDebugMode()	{
	unsigned long vulStartZeit = 0;
	pinMode(INTstMultiReset, INPUT);										// Muss bereits vor der Initialisierung als Eingang konfiguriert sein

	if (digitalRead(INTstMultiReset) == LOW)	{							// Taster beim Start nicht gedrueckt dann...
		debugMode = false;													// ...sofort Nextion-Modus, kein Warten noetig
		return;
	}
	debugMode = true;														// Annahme: Debug-Modus aktivieren -> bestaetigt wenn Taster durchgehend gehalten wird
	vulStartZeit = millis();
	while (millis() - vulStartZeit < debugBootTime)	{
		if (digitalRead(INTstMultiReset) == LOW)	{						// Wenn Taster vorzeitig losgelassen wird dann...
			debugMode = false;												// ...dann Nextion-Modus, kein Warten noetig
			return;
		}
	}																		// Wenn Funktion durchlaeuft dann "Debug-Mode bzw. Serial-Monitor aktiv"
	return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Remanenter Speicher auslesen"	***/

void speicherRead()	{
		if (EEPROM.read(ADDR_EEPROM_MAGIC) != EEPROM_MAGIC_VALUE)	{		// Wenn erster Initial-Boot oder korrupte Speicherstruktur dann...
		gwValueTag = DEF_GWVALTAG;											// ...zentrale Defaultwerte uebernehmen
		gwValueNacht = DEF_GWVALNACHT;
		lighttime = DEF_LIGHTTIME;
		dimmlevel = DEF_DIMMLEVEL;
		speicherWrite();													// ...Defaultwerte remanent sichern
		EEPROM.update(ADDR_EEPROM_MAGIC, EEPROM_MAGIC_VALUE);				// ...Magic-Byte setzen -> kuenftige Boots erkennen so gueltigen Speicher
		return;
	}
	byte vbTagLowbyte = EEPROM.read(ADDR_GWTAG_LO);							// LowByte "GW-TAG" lesen
	byte vbTagHighbyte = EEPROM.read(ADDR_GWTAG_HI);						// HighByte "GW-TAG" lesen
	byte vbNachtLowbyte = EEPROM.read(ADDR_GWNACHT_LO);						// LowByte "GW-NACHT" lesen
	byte vbNachtHighbyte = EEPROM.read(ADDR_GWNACHT_HI);					// HighByte "GW-NACHT" lesen
	byte vbLightTimeLowbyte = EEPROM.read(ADDR_LIGHTTIME_LO);				// LowByte "EINSCHALTDAUER" lesen
	byte vbLightTimeHighbyte = EEPROM.read(ADDR_LIGHTTIME_HI);				// HighByte "EINSCHALTDAUER" lesen
	byte vbDimmlevel = EEPROM.read(ADDR_DIMMLEVEL);							// Byte "DIMMSTUFE 0..100%" lesen
		
	gwValueTag = vbTagLowbyte + ((vbTagHighbyte << 8) & 0xFF00);			// Low- und HighByte "TAG" zusammenfuehren
	gwValueNacht = vbNachtLowbyte + ((vbNachtHighbyte << 8) & 0xFF00);		// Low- und HighByte "NACHT" zusammenfuehren
	checkGwBereich();														// Grenzwertvorgabe in FC pruefen
	
	lighttime = vbLightTimeLowbyte + ((vbLightTimeHighbyte << 8) & 0xFF00);	// Low- und HighByte "EINSCHALTDAUER LICHT STALL" zusammenfuehren
	checkMinMax(lighttime, MIN_LIGHTTIME, MAX_LIGHTTIME);					// "EINSCHALTDAUER Licht Stall" -> Min/Max-Begrenzung in Sekunden
	
	dimmlevel = vbDimmlevel;
	checkMinMax(dimmlevel, MIN_DIMMLEVEL, MAX_DIMMLEVEL);					// "DIMMSTUFE Licht Stall" -> Min/Max-Begrenzung in Prozent
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Remanenter Speicher schreiben"	***/

void speicherWrite()	{
	static byte vbTagLowbyte = 0;											// LowByte "GW-TAG" initialisieren
	static byte vbTagHighbyte = 0;											// HighByte "GW-TAG" initialisieren
	static byte vbNachtLowbyte = 0;											// LowByte "GW-NACHT" initialisieren
	static byte vbNachtHighbyte = 0;										// HighByte "GW-NACHT" initialisieren
	static byte vbLightTimeLowbyte = 0;										// LowByte "EINSCHALTDAUERR" initialisieren
	static byte vbLightTimeHighbyte = 0;									// HighByte "EINSCHALTDAUER" initialisieren
	static byte vbDimmlevel = 0;											// Byte "PWM-DIMMSTUFE" initialisieren
	
	vbTagLowbyte = gwValueTag &0xFF;										// LowByte aus "GW-TAG" extrahieren
	vbTagHighbyte = (gwValueTag >> 8) &0xFF;								// HighByte aus "GW-TAG" extrahieren
	vbNachtLowbyte = gwValueNacht &0xFF;									// LowByte aus "GW-NACHT" extrahieren
	vbNachtHighbyte = (gwValueNacht >> 8) &0xFF;							// HighByte aus "GW-NACHT" extrahieren
	vbLightTimeLowbyte = lighttime &0xFF;									// LowByte aus "EINSCHALTDAUER" extrahieren
	vbLightTimeHighbyte = (lighttime >> 8) &0xFF;							// HighByte aus "EINSCHALTDAUER" extrahieren
	vbDimmlevel = (byte)dimmlevel;											// Wertuebergabe von INT zu BYTE
	
	EEPROM.update(ADDR_GWTAG_LO, vbTagLowbyte);								// GW "TAG" LowByte in Speicher schreiben
	EEPROM.update(ADDR_GWTAG_HI, vbTagHighbyte);							// GW "TAG" HighByte in Speicher schreiben
	EEPROM.update(ADDR_GWNACHT_LO, vbNachtLowbyte);							// GW "NACHT" LowByte in Speicher schreiben
	EEPROM.update(ADDR_GWNACHT_HI, vbNachtHighbyte);						// GW "NACHT" HighByte in Speicher schreiben
	EEPROM.update(ADDR_LIGHTTIME_LO, vbLightTimeLowbyte);					// "EINSCHALTDAUER" LowByte in Speicher schreiben
	EEPROM.update(ADDR_LIGHTTIME_HI, vbLightTimeHighbyte);					// "EINSCHALTDAUER" HighByte in Speicher schreiben
	EEPROM.update(ADDR_DIMMLEVEL, vbDimmlevel);								// PWM-Dimmstufe "Licht Stall" in Speicher schreiben
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Helligkeit-GW-Bereiche gegenseitig pruefen"	***/

void checkGwBereich()	{
	int viLowMin = 0;														// Unteres Bereichsende des Minimumwertes
	int viLowMax = 0;														// Oberes Bereichsende des Minimumwertes
	int viHighMin = 0;														// Unteres Bereichsende des Maximalwertes
	int viHighMax = 0;														// Oberes Bereichsende des Maximalwertes
	
	viLowMin = gwValueNacht;												// Bereichsende-Zuweisung des tieferen Nachtwertes
	if (viLowMin < MIN_GWBEREICH)	{										// Check des tieferen "GW-NACHT"
		viLowMin = MIN_GWBEREICH;	}
		
	viHighMax = gwValueTag;													// Bereichsende-Zuweisung des hoeheren Tagwertes
	if (viHighMax > MAX_GWBEREICH)	{										// Check des hoeheren "GW-TAG"
	viHighMax = MAX_GWBEREICH;	}	
	
	viLowMax = viHighMax - gwHystValue;										// Bereichsende definieren und erneut pruefen
	if (viLowMax < MIN_GWBEREICH)	{
		viLowMax = MIN_GWBEREICH;	}
		
	viHighMin = viLowMax + gwHystValue;										// Bereichsende definieren
	if (viLowMin > viLowMax)	{											// ggf. Bereichswerte korrigieren -> bedingt notwendig. Falscher Wert haette keinen Einfluss mehr
		viLowMin = viLowMax;	}
	if (viHighMax < viHighMin)	{											// ggf. Bereichswerte korrigieren -> bedingt notwendig. Falscher Wert haette keinen Einfluss mehr
		viHighMax = viHighMin;	}
	
	checkMinMax(gwValueNacht, viLowMin, viLowMax);							// "GW-NACHT" -> Min/Max-Begrenzung fuer Integer-Analogwert
	checkMinMax(gwValueTag, viHighMin, viHighMax);							// "GW-TAG" -> Min/Max-Begrenzung fuer Integer-Analogwert
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Min/Max-Parameterpruefung"	***/

void checkMinMax(int &viInput, int viGwLow, int viGwHigh)	{				// Haupt-Eingangsparamter als Referenz (CallByReference)
	if (viInput < viGwLow)	{
		viInput = viGwLow;
	}
	if (viInput > viGwHigh)	{
		viInput = viGwHigh;
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Taster entprellen"	***/

void entprellen()	{
	struct strTASTER {														// lokale Strukturvariable definieren	
		unsigned long ulTime = 0;											// Komponente -> Laufzeit des gedrueckten Tasters 	-> default-Initialisierung
		bool xState = false;												// Komponente -> momentaner Status des Tasters 		-> default-Initialisierung
		bool xMainstate = false;											// Komponente -> entprellter Status des Tasters 	-> default-Initialisierung
	};
	static struct strTASTER vaTaster[anzahlPINIn];							// Struktur-Variable (mit n-Eingaengen) erstellen
		
	for (byte i = 0; i < anzahlPINIn; i++)	{								// for-Schlaufe mit n-Durchlaeufen fuer n-Taster zu entprellen
		if (vaTaster[i].xState==false)	{									// Wenn laufender Status "xState" FALSE dann...
			vaTaster[i].ulTime = millis();									// ...permanent die Laufzeit "xy" in Array "vaTaster" merken
		}
		if (digitalRead(arrPINIn[i])==HIGH)	{								// Wenn entsprechender I/O "xy" in Array "arrPINIn" EIN dann...
			vaTaster[i].xState = true;										// ...laufender Status "xState" auf TRUE
			if (millis() - vaTaster[i].ulTime > prellTime)	{				// ...wenn Entprellzeit erreicht dann...
				vaTaster[i].xMainstate = true;								// 		...Hauptstatus "xy" in Array "vaTaster" auf TRUE
			}
		}else{																// sonst...
			vaTaster[i].xMainstate = false;									// ...Hauptstatus "xy" in Array "vaTaster" auf FALSE
			vaTaster[i].xState = false;										// ...Status zuruecksetzen (Laufzeit wird im naechsten Durchlauf neu gesetzt)
		}
	}
		
	inputs.Safety1 = vaTaster[0].xMainstate;								// Ergebnisse aus for-Schleife den spezifischen Komponenten der Struktur "inputs.xy" zuweisen
	inputs.Safety2 = vaTaster[1].xMainstate;
	inputs.TstTorAuf = vaTaster[2].xMainstate;
	inputs.TstTorZu = vaTaster[3].xMainstate;
	inputs.TstLicht = vaTaster[4].xMainstate;
	inputs.TstMultiReset = vaTaster[5].xMainstate;
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Tasterfunktion erkennen"	***/

void tasterfunktion()	{
	static unsigned long vulTime = 0;										// momentane Laufzeit
	static bool vxState = false;											// laufender Status
	static bool vxOldState = false;											// vorheriger Status
	
	if (vxState == false)		{											// Wenn laufender Status "vxState" FALSE dann...													
		vulTime = millis();													// ...permanent die Laufzeit merken
	}
	if (inputs.TstMultiReset == true)	{									// Wenn Taster "Multifunktion-Reset" gedrueckt dann...
		vxState = true;														// ...laufender Status "vxState" auf TRUE											
		vxOldState = true;													// ...vorheriger Status "vxOldState" auf TRUE (alter Schaltzustand merken)
			if (millis() - vulTime >= holdTimeNextion)	{					// ...wenn Erkennungszeit erreicht dann...						
				TstFunc.Nextion = true;										// 		...Tasterfunktion "Nextion-HMI einschalten" erkannt
				vxOldState = false;											// 		...vorheriger Status "vxOldState" zuruecksetzen da kein Impulsvorgang getaetigt
			}	
	}else{																	// sonst... 
		vxState = false;													// ...Laufzeit aktualisieren														
		TstFunc.Nextion = false;											// ...und "Nextion-HMI einschalten" auf FALSE setzen (kurzer Tastdruck = "Alarm-Reset")
	}
	if (vxState == false && vxOldState == true)	{							// Wenn Taster losgelassen wird dann... (negative Flanke)
		TstFunc.Reset = true;												// ...Tasterfunktion "Alarm-Reset" erkannt (positive Flanke setzen)
		vxOldState = false;													// ...und "alter Schaltzustand" wieder zuruecksetzen
	}else{																	// sonst... 
		TstFunc.Reset = false;												// ...Tasterfunktion "Alarm-Reset" zuruecksetzen
	}
return;
}	


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Nextion-HMI einschalten"	***/

void nextionPower()	{

	if (TstFunc.Nextion == true)	{										// Wenn Tasterfunktion "Nextion-HMI einschalten" erkannt dann...
			digitalWrite(OUTNextionPower, HIGH);							// ...Nextion-Speisung einschalten (falls bereits an: keine Wirkung)
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Messung Tageslicht"	***/

void daylight()	{
	static unsigned long vulTimeTag = 0;      								// laufende Hysteresezeit "Tag"
	static unsigned long vulTimeNacht = 0;     						   		// laufende Hysteresezeit "Nacht"
	static unsigned long vulMeasureTime = 0;								// laufende Abtastzeit
	static bool vxStateTag = false;                			    			// laufender Status "Tag"
	static bool vxStateNacht = false;            							// laufender Status "Nacht"
	int vbnewValue = 0;														// aktueller Messwert

	if (millis() - vulMeasureTime >= samplingTime)	{						// Im Takt der Abtastzeit messen
		vulMeasureTime = millis();
		vbnewValue = analogRead(INADaylight);									// Lichtwert aus Photosensor auslesen -> Integerwert 0..1024
		lightvalue = (lightvalue * averageCnt + vbnewValue)/(averageCnt + 1);	// fliessende Mittelwertbildung
	}

// Tagerkennung
	if (vxStateTag == false)  {                								// Wenn laufender Status "Tag" FALSE dann...
		vulTimeTag = millis();             									// ...permanent die Laufzeit merken
	}
	if (lightvalue >= gwValueTag) {               							// Wenn Tageshelligkeit-Grenzwert erreicht dann...
		vxStateTag = true;                 									// ...laufender Status "Tag" auf TRUE
		if (millis() - vulTimeTag > photoTime)  {    						// ...wenn Hysterese-Umschaltzeit erreicht dann...
			stateTag = true;                    							// 		...Hauptstatus "Tag" auf TRUE
		}
    }else{																	// sonst...
		vxStateTag = false;                  								// ...Status zuruecksetzen (Laufzeit wird im naechsten Durchlauf neu gesetzt)
	}
 
// Nachterkennung
	if (vxStateNacht == false)  {              								// Wenn laufender Status "Nacht" FALSE dann...
		vulTimeNacht = millis();             								// ...permanent die Laufzeit merken
	}
	if (lightvalue <= gwValueNacht) {             							// Wenn Nachthelligkeit-Grenzwert erreicht dann...
		vxStateNacht = true;                 								// ...laufender Status "Nacht" auf TRUE
		if (millis() - vulTimeNacht > photoTime)  {  						// ...wenn Hysterese-Umschaltzeit erreicht dann...
			stateTag = false;                   							// 		...Hauptstatus "Tag" auf FALSE
		}
    }else{																	// sonst...                            
		vxStateNacht = false;                								// ...Status zuruecksetzen (Laufzeit wird im naechsten Durchlauf neu gesetzt)
	}
return;	
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Messung RM Motorsicherung"	***/

void motfuse()	{
	static unsigned long vulTime = 0;      									// laufende Alarmverzoegerung "RM Motorsicherung"
	static unsigned long vulMeasureTime = 0;								// laufende Abtastzeit
	static bool vxState = false;                			    			// laufender Status "RM Motorsicherung""
	int vbnewValue = 0;														// aktueller Messwert

	if (millis() - vulMeasureTime >= samplingTime)	{						// Im Takt des Abtastzeit tatsaechlich neu messen
		vulMeasureTime = millis();
		vbnewValue = analogRead(INAMotfuse);									// Sicherungsspannung messen -> Integerwert 0..1024
		motfuseRaw = (motfuseRaw * averageCnt + vbnewValue)/(averageCnt + 1);	// fliessende Mittelwertbildung
	}

// Zustand Motorsicherung ermitteln
	if (vxState == false)  {                								// Wenn laufender Status FALSE dann...
		vulTime = millis();             									// ...permanent die Laufzeit merken
	}
	if ((motfuseRaw <= gwMotfuseRaw) && (outputs.PowOn==true)) {			// Wenn Sicherungsspannung kleiner als GW und Motorspeisung AKTIV dann...
		vxState = true;                 									// ...laufender Status auf TRUE
		if (millis() - vulTime > motfuseAlaTime)  {    						// ...wenn Alarmverzoegerung erreicht dann...
			motfuseAlarm = true;                    						// 		...Alarmstatus "Motorsicherung ausgeloest" auf TRUE
		}
    }else{																	// sonst...
		vxState = false;                  									// ...Status zuruecksetzen (Laufzeit wird im naechsten Durchlauf neu gesetzt)
	}
	motfuseAlarmPending = vxState;											// ...global spiegeln fuer sleepAllowed()
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Messung Batteriespannung"	***/

void batterie()	{
	static unsigned long vulTime = 0;      									// laufende Alarmverzoegerung "Batterieladung"
	static unsigned long vulMeasureTime = 0;								// laufende Abtastzeit
	static bool vxState = false;                			    			// laufender Status "Batterieladung"
  	int vbnewValue = 0;														// aktueller Messwert

	if (millis() - vulMeasureTime >= samplingTime)	{						// Im Takt des Abtastzeit tatsaechlich neu messen
		vulMeasureTime = millis();
		vbnewValue = analogRead(INABatterie);									// Batteriespannung messen -> Integerwert 0..1024
		batterieRaw = (batterieRaw * averageCnt + vbnewValue)/(averageCnt + 1);	// fliessende Mittelwertbildung  
		batterieProzent = battAdcToPercent(batterieRaw);						// Ladezustand in Prozent ableiten
	}

// Alarm Batterieladung ermitteln
	if (vxState == false)  {                								// Wenn laufender Status FALSE dann...
		vulTime = millis();             									// ...permanent die Laufzeit merken
	}
	if (batterieRaw <= gwBatterieRaw) {               						// Wenn Batteriespannung kleiner als Grenzwert dann...
		vxState = true;                 									// ...laufender Status auf TRUE
		if (millis() - vulTime > battAlaTime)  {    						// ...wenn Alarmverzoegerung erreicht dann...
			batterieAlarm = true;                    						// 		...Alarmstatus "Batterieladung tief" setzen
		}
    }else{																	// sonst...
		vxState = false;                  									// ...Status zuruecksetzen (Laufzeit wird im naechsten Durchlauf neu gesetzt)
	}
	batterieAlarmPending = vxState;											// ...global spiegeln fuer sleepAllowed()
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Batterie-Ladezustand aus Batteriespannung ableiten (AGM/Gel)"	***/
		
		// Gebraeuchliche Naeherungswerte fuer AGM/Gel-Batterien
		// Bezogen an bestehenden Referenzpunkt "gwBatterieRaw" (810 ADC = 11.85V = 30%)
		// ADC-Spannung gemaess Spannungsteiler 12..15V = 4..5V

struct BattSocPunkt	{
	int adcWert;															// ADC-Rohwert (0..1023)
	byte prozent;															// zugehoeriger Ladezustand in Prozent
};

const BattSocPunkt battSocTabelle[] = {										// Tabelle ABSTEIGEND sortiert
	{874, 100},		// 12.80V - voll geladen (Ruhespannung)
	{860,  90},		// 12.60V
	{853,  80},		// 12.50V
	{847,  70},		// 12.40V
	{840,  60},		// 12.30V
	{833,  50},		// 12.20V
	{819,  40},		// 12.00V
	{810,  30},		// 11.85V - bestehender Referenzpunkt "gwBatterieRaw"
	{795,  20},		// 11.65V
	{775,  10},		// 11.35V
	{717,   0}		// 10.50V - als leer betrachtet
};
const byte battSocTabelleLen = sizeof(battSocTabelle) / sizeof(battSocTabelle[0]);

byte battAdcToPercent(int adcWert)	{
	byte i;

	if (adcWert >= battSocTabelle[0].adcWert)	{							// Oberhalb des hoechsten Tabellenwertes dann...
		return battSocTabelle[0].prozent;									// ...auf 100% begrenzen
	}
	if (adcWert <= battSocTabelle[battSocTabelleLen - 1].adcWert)	{		// Unterhalb des tiefsten Tabellenwertes dann...
		return battSocTabelle[battSocTabelleLen - 1].prozent;				// ...auf 0% begrenzen
	}

	for (i = 0; i < (battSocTabelleLen - 1); i++)	{						// Passendes Tabellensegment suchen...
		if ((adcWert <= battSocTabelle[i].adcWert) && (adcWert >= battSocTabelle[i + 1].adcWert))	{
			return map(adcWert, battSocTabelle[i + 1].adcWert, battSocTabelle[i].adcWert,			// ...und linear zwischen den beiden
					   battSocTabelle[i + 1].prozent, battSocTabelle[i].prozent);					//    Stuetzpunkten interpolieren
		}
	}
return 0;																	// Sicherheitsnetz, wird bei obiger Abdeckung nie erreicht
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "UART-HMI: Telegrammlaenge bestimmen"	***/

		// Liefert die gesamte Telegrammlaenge (Kommandobyte inklusive, OHNE die 3x 0xFF-Terminierung) fuer die ausgewerteten Telegrammtypen.
		// Rueckgabe 0 = Typ wird nicht ausgewertet -> hmiRead() ueberspringt ein solches Telegramm ueber die klassische 3x-0xFF-Suche.
		
byte nexErwarteteLaenge(byte cmd)	{
	switch (cmd)	{
		case 0x65: return 4;												// Touch-Ereignis: 0x65, page, component, event
		case 0x66: return 2;												// Seitenwechsel ("sendme"): 0x66 + page
		case 0x73: return 7;												// Direktuebertragung: 0x73 + page + id + 4 Byte int32 (LE)
		default:   return 0;												// unbekannter/nicht ausgewerteter Telegrammtyp
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "UART-HMI: Daten empfangen"	***/

		// #Nextion-Telegramme sind immer mit 3x 0xFF terminiert. Drei Telegrammtypen werden ausgewertet:
		//   0x65 <page> <component> <event>		-> Touch-Ereignis (nur noch fuer den Lichttaster ausgewertet)
		//   0x66 <page>							-> Seitenwechsel-Meldung ("sendme", siehe jede Seiten-Preinitialize)
		//   0x73 <page> <id> <b0> <b1> <b2> <b3>	-> Direktuebertragung Seite+ID+Wert (Postinitialize eines Zahlenfeldes)
		// Ablauf: Ein Zahlenfeld schickt nach Eingabe-Bestaetigung seinen neuen Wert direkt per 0x73 - kein "get" mehr noetig
		//  ...Die Nutzlaenge wird anhand des Kommandobyte vorgegeben (nexErwarteteLaenge()),und so wird ein 0xFF-Byte innerhalb der
		//  ...Nutzdaten (z.B. hohe int32-Werte oder negative Zahlen in Two's-Complement) korrekt als Datenbyte uebernommen und nicht als Telegrammende gewertet.
		//  ...Nur nach Erreichen der erwarteten Nutzlaenge wird auf die 3x 0xFF-Terminierung geprueft. Kommt kein 0xFF, wird von einem Protokoll-Desync ausgegangen und das Telegramm verworfen (Resync).

void hmiRead()	{
	static byte vBuf[16];													// Empfangspuffer fuer ein Telegramm (Kommandobyte inklusive)
	static byte vBufLen = 0;												// Anzahl bereits empfangener Telegramm-Bytes
	static byte vErwarteteLaenge = 0;										// Erwartete Gesamtlaenge (inkl. Kommandobyte) des laufenden Telegramms
	static byte vFFCount = 0;												// Zaehler aufeinanderfolgender 0xFF (nur fuer Terminator- bzw. Resync-Erkennung)
	static NEX_PARSE_STATE vState = NEX_WAIT_CMD;							// Parser-Zustand (bleibt ueber Aufrufe hinweg erhalten)

	while (Serial.available() > 0)	{										// Solange Zeichen im UART-Puffer warten...
		byte vB = Serial.read();

		switch (vState)	{
			case NEX_WAIT_CMD:												// Warten auf das naechste Kommandobyte (Telegrammanfang)
				if (vB == 0xFF)	{											// ...ueberzaehlige/verirrte Terminator-Reste einfach ignorieren
					break;
				}
				vBufLen = 0;
				vBuf[vBufLen++] = vB;											// ...Kommandobyte uebernehmen
				vErwarteteLaenge = nexErwarteteLaenge(vB);					// ...erwartete Gesamtlaenge anhand des Kommandobytes bestimmen
				vFFCount = 0;
				if (vErwarteteLaenge > 0)	{								// Bekannter, ausgewerteter Telegrammtyp...
					vState = (vBufLen < vErwarteteLaenge) ? NEX_COLLECT_PAYLOAD : NEX_WAIT_TERM;
				}else{														// ...sonst unbekannter Typ -> nur bis zum Ende ueberspringen
					vState = NEX_SKIP_UNKNOWN;
				}
				hmiReceiveInProgress = true;								// Telegrammempfang begonnen - hmiSend() pausiert jetzt zusaetzlich
				hmiReceiveStartTime = millis();	
			break;

			case NEX_COLLECT_PAYLOAD:										// Nutzdaten eines bekannten Telegrammtyps sammeln
				if (vBufLen < sizeof(vBuf))	{								// Nur puffern wenn noch Platz (Schutz vor Overflow)
					vBuf[vBufLen++] = vB;									// ...JEDES Byte, AUCH 0xFF, zaehlt hier als Nutzdatum (keine Terminator-Wertung!)
				}
				if (vBufLen >= vErwarteteLaenge)	{						// Wenn die erwartete Nutzlaenge erreicht ist dann...
					vState = NEX_WAIT_TERM;									// ...ab jetzt die 3x 0xFF-Terminierung erwarten
				}
			break;

			case NEX_WAIT_TERM:												// Ab hier werden ausschliesslich die 3x 0xFF-Terminatorbytes erwartet
				if (vB == 0xFF)	{
					vFFCount++;
					if (vFFCount >= 3)	{									// Telegramm korrekt terminiert...
						nexFrameAuswerten(vBuf, vBufLen);					// ...auswerten
						vBufLen = 0;										// ...Puffer fuer naechstes Telegramm zuruecksetzen
						vFFCount = 0;
						vState = NEX_WAIT_CMD;
						hmiReceiveInProgress = false;						// Telegramm fertig - hmiSend() darf wieder senden
					}
				}else{														// Kein 0xFF wo Terminator erwartet wird -> Protokoll-Desync...
					vBufLen = 0;											// ...Telegramm verwerfen...
					vFFCount = 0;
					vState = NEX_WAIT_CMD;									// ...und mit dem naechsten Byte neu synchronisieren
					hmiReceiveInProgress = false;
				}
			break;

			case NEX_SKIP_UNKNOWN:											// Unbekannter/nicht ausgewerteter Telegrammtyp -> klassische 3x-0xFF-Suche zum Ueberspringen
				if (vB == 0xFF)	{
					vFFCount++;
					if (vFFCount >= 3)	{									// Telegrammende gefunden -> verwerfen (keine Auswertung noetig)...
						vBufLen = 0;
						vFFCount = 0;
						vState = NEX_WAIT_CMD;								// ...und mit dem naechsten Byte neu synchronisieren
						hmiReceiveInProgress = false;
					}
				}else{
					vFFCount = 0;
				}
			break;
		}
	}
	
	if ((hmiReceiveInProgress == true) && (millis() - hmiReceiveStartTime > hmiReceiveTimeout))	{
		vBufLen = 0;														// Sicherheitsabschaltung: Telegramm kam nie vollstaendig an
		vFFCount = 0;														// ...Parser zuruecksetzen, damit hmiSend() nicht dauerhaft blockiert
		vState = NEX_WAIT_CMD;
		hmiReceiveInProgress = false;
	}																		// hmiPendingRequest-Timeout entfaellt - kein "get" mehr, siehe 0x73-Direktuebertragung
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "UART-HMI: Telegramm auswerten"	***/

void nexFrameAuswerten(byte* frame, byte len)	{
	if (len == 0)	{
		return;
	}
	vulLastNextionActivity = millis();										// ...jedes ausgewertete Telegramm zaehlt als Nextion-Aktivitaet fuer sleepAllowed()

	if ((frame[0] == 0x65) && (len >= 4))	{								// Touch-Ereignis: 0x65, page, component, event - nur noch fuer den Lichttaster gebraucht
		byte vPageId = frame[1];
		byte vCompId = frame[2];
		byte vEventType = frame[3];

		if (vEventType == 0x00)	{											// Nur bei "Loslassen" reagieren
			if ((vPageId == NEX_PAGE_BOXLIGHT) && (vCompId == NEX_CID_LICHTTOGGLE))	{
				hmiLichtToggleRequest = true;
			}
		}
	}
	else if ((frame[0] == 0x66) && (len >= 2))	{							// Seitenwechsel-Meldung ("sendme", in jeder Seiten-Preinitialize hinterlegt)
		currentPage = frame[1];												// ...aktuelle Seite merken
		hmiSendIndex = 0;														// ...Rundlauf fuer die neue Seite von vorne beginnen
	}
	else if ((frame[0] == 0x73) && (len >= 7))	{							// Direktuebertragung Seite+ID+Wert (ersetzt 0x65+"get"+0x71 fuer Eingabefelder)
		byte vPageId = frame[1];
		byte vCompId = frame[2];
		long vValue = (long)frame[3] | ((long)frame[4] << 8) | ((long)frame[5] << 16) | ((long)frame[6] << 24);
		nexWertUebernehmen(vPageId, vCompId, vValue);
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "UART-HMI: validierten Wert uebernehmen"	***/

void nexWertUebernehmen(byte page, byte id, long value)	{
	int vNeuerWert = (int)value;

	if ((page == NEX_PAGE_DAYLIGHT) && (id == NEX_CID_GWTAG))	{			// direkte Seite+ID-Zuordnung
		int vVorTag = gwValueTag;											// Werte vor der Aenderung merken (checkGwBereich()
		int vVorNacht = gwValueNacht;										// kann bei Bedarf BEIDE Grenzwerte anpassen
		gwValueTag = vNeuerWert;
		checkGwBereich();													// Tag/Nacht/Hysterese-Verhaeltnis pruefen und ggf. korrigieren
		if ((gwValueTag != vVorTag) || (gwValueNacht != vVorNacht))	{		// Nur bei tatsaechlicher Aenderung...
			speicherWrite();												// ...remanent sichern
		}
	}
	else if ((page == NEX_PAGE_DAYLIGHT) && (id == NEX_CID_GWNACHT))	{
		int vVorTag = gwValueTag;
		int vVorNacht = gwValueNacht;
		gwValueNacht = vNeuerWert;
		checkGwBereich();
		if ((gwValueTag != vVorTag) || (gwValueNacht != vVorNacht))	{
			speicherWrite();
		}
	}
	else if ((page == NEX_PAGE_BOXLIGHT) && (id == NEX_CID_LIGHTTIME))	{
		int vVorher = lighttime;
		lighttime = vNeuerWert;
		checkMinMax(lighttime, MIN_LIGHTTIME, MAX_LIGHTTIME);
		if (lighttime != vVorher)	{
			speicherWrite();
		}
	}
	else if ((page == NEX_PAGE_BOXLIGHT) && (id == NEX_CID_DIMM))	{
		int vVorher = dimmlevel;
		dimmlevel = vNeuerWert;
		checkMinMax(dimmlevel, MIN_DIMMLEVEL, MAX_DIMMLEVEL);
		if (dimmlevel != vVorher)	{
			speicherWrite();
		}
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "UART-HMI: Hilfsfunktionen (Senden)"	***/

void nexEnde()	{															// Standard-Telegrammende (3x 0xFF) senden
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
return;
}

void nexSetValue(const char* compName, long value)	{						// "<component>.val=<value>" absenden
	Serial.print(compName);
	Serial.print(".val=");
	Serial.print(value);
	nexEnde();
return;
}

void nexSetText(const char* compName, const char* text)	{					// "<component>.txt=\"<text>\"" absenden
	Serial.print(compName);
	Serial.print(".txt=\"");
	Serial.print(text);
	Serial.print("\"");
	nexEnde();
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Torsteuerung"	***/

void torsteuerung()	{
	static unsigned long vulTimeA1 = 0;										// momentane Relais-Laufzeit "einschalten" -> Tor oeffnen
	static unsigned	long vulTimeA2 = 0;										// momentane Relais-Laufzeit "ausschalten" -> Tor schliessen
	static bool vxStateA1 = false;											// laufender Status "Relais einschalten"
	static bool vxStateA2	= false;										// laufender Status "Relais ausschalten"
	static bool vxSkAutoOpen = false;           							// Sk-Status "Tor automatisch oeffnen"
	static bool vxSkHandOpen = false;           							// Sk-Status "Tor manuell oeffnen"
	static bool vxSkAutoClose = false;          							// Sk-Status "Tor automatisch schliessen"
	static bool vxSkHandClose = false;          							// Sk-Status "Tor manuell schliessen"
	static bool vxSafetyUp = false;											// Status "Sicherheitsoeffnung durchfuehren"
	static bool vxSafetyWait = false;										// Merker das Status "vxSafetyUp" ausgeloest wurde
	static bool vxTagOld = false;											// Merker "Tageszustand" (ist bereits Tag ??)

	static unsigned long vulGoUpTime = 0;									// momentane Laufzeit "Tor oeffnet sich"
	static unsigned long vulGoDownTime = 0;									// momentane Laufzeit "Tor schliesst sich"
	static unsigned long vulDriveTime = 0;									// ueberschreibare Variable fuer Torlaufzeit wenn "SafetyUp" ausgeloest hat
	static bool vxGoUp = false;												// laufender Status "Tor oeffnet sich"
	static bool vxGoDown = false;											// laufender Status "Tor schliesst sich"
	static bool vxMustUp = false;											// Merker "Tor muss sich oeffnen"
	static bool vxMustDown = false;											// Merker "Tor muss sich schliessen"
	static bool vxIsUp = false;												// Merker "Tor ist offen"
	static bool vxIsDown = false;											// Merker "Tor ist geschlossen"

	
// Fahrtfreigabe ermitteln
	if (((inputs.Safety1 == true) || (inputs.Safety2 == true)) && (vxMustDown == true))	{		// Wenn "Safety-Innen" oder "Safety-Aussen" Hindernis erkannt hat und Merker "Tor muss sich schliessen" dann...
		vxSafetyUp = true;													// ..."SafetyUp" ausloesen (Flanke)
		vxSafetyWait = true;												// ...Merker "SafetyUp wurde ausgeloest" auf TRUE  (permanent)
	}else{																	// sonst...
		vxSafetyUp = false;													// ...KEIN SafetyUp ausloesen
	}
	
// Flankenauswertung Statusaenderung von "TAG zu NACHT"
	if ((vxTagOld == false) && (stateTag == true))	{						// Flankenauswertung vom Nacht/Tag-Wechsel (wird fuer Tor-Handeingriff benoetigt)
		vxSkAutoOpen = false;												// ...Automatisches "Tor oeffnen" wieder freigegeben
	}
	if ((vxTagOld == true) && (stateTag == false)) {   						// Flankenauswertung vom Tag/Nacht-Wechsel (wird fuer Tor-Handeingriff benoetigt)
		vxSkAutoClose = false;             									// ...Automatisches "Tor schliessen" wieder freigegeben
	}
	vxTagOld = stateTag;                 									// Merker "Tageszustand" aktualisieren

// Schritketten-Steuerung
	switch(schrittTor)	{
		case(STANDBY):														// Im Sk-Status "STANDBY" wird Auswahl getroffen in welchen Schritt es geht
			if ((vxSkAutoOpen == false) && (stateTag == true))	{										// Wenn noch nie "AUTOAUF" war und es Tag wird, dann SK "AUTOAUF"
				schrittTor = AUTOAUF;
			}
			else if ((vxSkHandOpen == false) && (inputs.TstTorAuf == true))	{							// Wenn noch nie "HANDAUF" war und Taster "AUF" gedrueckt, dann SK "HANDAUF"
				schrittTor = HANDAUF;
			}
			else if ((vxSkAutoClose == false) && (stateTag == false) && (safetyAlarm == false))	{		// Wenn noch nie "AUTOZU" war und es Nacht wird, dann SK "AUTOZU"
				schrittTor = AUTOZU;
			}
			else if ((vxSkHandClose == false) && (inputs.TstTorZu == true) && (safetyAlarm == false))	{	// Wenn noch nie "HANDZU" war und Taster "ZU" gedrueckt, dann SK "HANDZU"
				schrittTor = HANDZU;
			}
		break;
		case(AUTOAUF):
			vxSkAutoOpen = true;											// Merker "Sk AUTOAUF" wird ausgefuehrt
			vxSkHandOpen = true;											// Merker "Sk HANDAUF" wird ausgefuehrt
			vxSkAutoClose = false;											// Merker "Sk AUTOZU" resetieren
			vxSkHandClose = false;											// Merker "Sk HANDZU" resetieren
			if (vxIsUp == false)	{										// Wenn Tor noch nicht offen...
				vxMustUp = true;											// 	...dann Merker "Tor muss sich oeffnen" setzen
			}
			if (vxIsUp == true)	{											// Wenn Tor offen ist/wird dann...
				if (vxSafetyWait == true)	{								// ...wenn Sk aufgrund "vxSafetyUp" ausgeloest wurde dann...
					schrittTor = AUTOZU;									//		...direkt wieder in Sk "AUTOZU"
					vxMustUp = false;										//		...Tor muss sich nicht mehr oeffnen
					vxSafetyWait = false;									//		..."vxSafetyWait" wieder resetieren
				}else{														// ...sonst...
					schrittTor = STANDBY;									//		...Sk auf STANDBY
					vxMustUp = false;										//		...Tor muss sich nicht mehr oeffnen
				}					
			}
			if (inputs.TstTorZu == true)	{									// Wenn Taster "HandZu" betaetigt wird dann...
				schrittTor = STANDBY;										// ...Sk auf STANDBY
				vxMustUp = false;											// ...Tor muss sich nicht mehr oeffnen
			}
		break;
		case(HANDAUF):
			vxSkAutoOpen = false;											// Merker "Sk AUTOAUF" resetieren
			vxSkHandOpen = true;											// Merker "Sk HANDAUF" wird ausgefuehrt
			vxSkAutoClose = true;											// Merker "Sk AUTOZU" wird ausgefuehrt
			vxSkHandClose = false;											// Merker "Sk HANDZU" resetieren
			if (vxIsUp == false)	{										// Wenn Tor noch nicht offen...
				vxMustUp = true;											// ...dann Merker "Tor muss sich oeffnen" setzen
			}
			if (vxIsUp == true)	{											// Wenn Tor offen ist/wird dann...
				if (vxSafetyWait == true)	{								// ...wenn Sk aufgrund "vxSafetyUp" ausgeloest wurde dann...
					schrittTor = HANDZU;									//		...direkt wieder in Sk "HANDZU"
					vxMustUp = false;										//		...Tor muss sich nicht mehr oeffnen
					vxSafetyWait = false;									//		..."vxSafetyWait" wieder resetieren
				}else{														// ...sonst...
					schrittTor = STANDBY;									//		...Sk auf STANDBY
					vxMustUp = false;										//		...Tor muss sich nicht mehr oeffnen
				}
			}
			if (inputs.TstTorZu == true)	{									// Wenn Taster "HandZu" betaetigt wird dann...
				schrittTor = STANDBY;										// ...Sk auf STANDBY
				vxMustUp = false;											// ...Tor muss sich nicht mehr oeffnen
			}
		break;
		case(AUTOZU):
			vxSkAutoOpen = false;											// Merker "Sk AUTOAUF" resetieren
			vxSkHandOpen = false;											// Merker "Sk HANDAUF" resetieren
			vxSkAutoClose = true;											// Merker "Sk AUTOZU" wird ausgefuehrt
			vxSkHandClose = true;											// Merker "Sk HANDZU" wird ausgefuehrt
			if (vxIsDown == false)	{										// Wenn Tor noch nicht zu...
				vxMustDown = true;											// ...dann Merker "Tor muss sich schliessen" setzen
			}
			if (vxIsDown == true)	{										// Wenn Tor geschlossen ist/wird dann...
				schrittTor = STANDBY;										// ...Sk auf STANDBY
				vxMustDown = false;											// ...Tor muss sich nicht mehr schliessen
			}
			if (inputs.TstTorAuf == true)	{									// Wenn Taster "HandAuf" betaetigt wird dann...
				schrittTor = STANDBY;										// ...Sk auf STANDBY
				vxMustDown = false;											// ...Tor muss sich nicht mehr schliessen
			}
			if (vxSafetyUp == true)	{										// Wenn "SafetyUp" ausgeloest dann..
				schrittTor = AUTOAUF;										// ...direkt in Sk "AUTOAUF" springen im naechsten Zyklus
				vxMustDown = false;											// ...Tor muss sich nicht mehr schliessen
				cntSafetyFail++;											// ...laufender Zaehler "Fahrfehler Tor" inkrementieren
			}
		break;
		case(HANDZU):
			vxSkAutoOpen = true;											// Merker "Sk AUTOAUF" wird ausgefuehrt
			vxSkHandOpen = false;											// Merker "Sk HANDAUF" resetieren
			vxSkAutoClose = false;											// Merker "Sk AUTOZU" resetieren
			vxSkHandClose = true;											// Merker "Sk HANDZU" wird ausgefuehrt
			if (vxIsDown == false)	{										// Wenn Tor noch nicht zu...
				vxMustDown = true;											// ...dann Merker "Tor muss sich schliessen" setzen
			}
			if (vxIsDown == true)	{										// Wenn Tor geschlossen ist/wird dann...
				schrittTor = STANDBY;										// ...Sk auf STANDBY
				vxMustDown = false;											// ...Tor muss sich nicht mehr schliessen
			}
			if (inputs.TstTorAuf == true)	{									// Wenn Taster "HandAuf" betaetigt wird dann...
				schrittTor = STANDBY;										// ...Sk auf STANDBY
				vxMustDown = false;											// ...Tor muss sich nicht mehr schliessen
			}
			if (vxSafetyUp == true)	{										// Wenn "SafetyUp" ausgeloest dann...
				schrittTor = HANDAUF;										// ...direkt in Sk HANDAUF" springen im naechsten Zyklus
				vxMustDown = false;											// ...Tor muss sich nicht mehr schliessen
				cntSafetyFail++;											// ...laufender Zaehler "Fahrfehler Tor" inkrementieren
			}
		break;
		default:															// Default-Anweisung falls Ablaufstoerung eintritt
			schrittTor = STANDBY;											// Sk auf STANDBY
			skAlarm = true;													// Alarmstatus "Schrittketten-Ablaufstoerung" auf TRUE
		break;
	}
	
// Toroeffnung -> bistabiles Relais
	if (vxStateA1 == false)	{												// Wenn laufender Status "Relais einschalten" FALSE dann...
		vulTimeA1 = millis ();												// ...permanent die Laufzeit merken
	}
	if (vxMustUp == true)	{												// Wenn "Tor muss sich oeffnen" TRUE...
		vxStateA1 = true;													// ...laufender Status "Relais einschalten" auf TRUE																				
		if (millis() - vulTimeA1 < relaisTime)	{							// ...Wenn Wartezeit NOCH NICHT erreicht dann...
			outputs.MotAuf = true;											// 		...bistabiles Relais bestromen
		}else{																// ...sonst...
			outputs.MotAuf = false;											// 		...nicht mehr bestromen
		}
	}else{																	// sonst wenn "Tor muss sich schliessen"...
		vxStateA1 = false;													// ...Laufzeit aktualisieren
		outputs.MotAuf = false;												// ...und bistabiles Relais nicht mehr bestromen
	}
	
// Torschliessung -> bistabiles Relais
	if (vxStateA2 == false)	{												// Wenn laufender Status "Relais ausschalten" FALSE dann...
		vulTimeA2 = millis ();												// ...permanent die Laufzeit merken
	}
	if (vxMustDown == true)	{												// Wenn "Tor muss sich schliessen" dann...
		vxStateA2 = true;													// ...laufender Status "Relais ausschalten" auf TRUE
		if (millis() - vulTimeA2 < relaisTime)	{							// ...Wenn Wartezeit NOCH NICHT erreicht dann...
			outputs.MotZu = true;											// 		...bistabiles Relais bestromen
		}else{																// ...sonst...
			outputs.MotZu = false;											// 		...nicht mehr bestromen
		}
	}else{																	// sonst wenn "Tor muss sich oeffnen"...
		vxStateA2 = false;													// ...Laufzeit aktualisieren
		outputs.MotZu = false;												// ...und bistabiles Relais nicht mehr bestromen
	}

// Laufzeit fuer AUF -> Statusmeldung wenn erreicht
	if (vxGoUp == false)	{												// Wenn laufender Status "faehrt aufwaerts" FALSE dann...
		vulGoUpTime = millis();												// ...permanent die Laufzeit merken
	}
	if (vxMustUp == true)	{												// Wenn "Tor muss sich oeffnen" TRUE dann...
		vxGoUp = true;														// ...laufender Status "faehrt aufwaerts" auf TRUE
		vxIsDown = false;													// ...Merker "Tor ist geschlossen" auf FALSE
		if (vxSafetyWait == true)	{										// ...Wenn "SafetyUp" ausgeloest wurde dann...
			vulDriveTime = driveTime + waitTime;							// 		...Wartezeit "Tor ist offen" verlaengern -> erneuter Versuch "Tor schliessen" wird durchgefuehrt
		}else{																// ...sonst...
			vulDriveTime = driveTime;										// 		...Standart-Fahrzeit verwenden
		}
		if (millis() - vulGoUpTime >= vulDriveTime)	{						// ...Wenn maximale Laufzeit erreicht dann...
			vxIsUp = true;													// 		...Status "Tor offen" auf TRUE
		}
	}else{																	// sonst...
		vxGoUp = false;														// ...laufender Status "faehrt aufwaerts" auf FALSE
	}
	
// Laufzeit fuer ZU -> Statusmeldung wenn erreicht
	if (vxGoDown == false)	{												// Wenn laufender Status "faehrt abwaerts" FALSE dann...												
		vulGoDownTime = millis();											// ...permanent die Laufzeit merken
	}
	if (vxMustDown == true)	{												// Wenn "Tor muss sich schliessen" TRUE dann...
		vxGoDown = true;													// ...laufender Status "faehrt abwaerts" auf TRUE
		vxIsUp = false;														// ...Merker "Tor ist offen" auf FALSE
		if (millis() - vulGoDownTime >= driveTime)	{						// ...Wenn maximale Laufzeit erreicht dann...
			vxIsDown = true;												// 		...Status "Tor zu" auf TRUE
		}
	}else{																	// sonst...
		vxGoDown = false;													// ...laufender Status "faehrt abwaerts" auf FALSE
	}
	
// Allgemeiner Code	
	outputs.PowOn = vxMustDown;												// globale Variablenuebergabe fuer "Torsensoren/partielle Motorspeisung"
	
// Alarmausloesung durch Fahrfehler
	if (cntSafetyFail >= gwSafetyFail)	{									// Wenn Zaehler "Fahrfehler Tor" groesserGleich Grenzwert dann...
		safetyAlarm = true;													// ...Alarmstatus "Fahrfehler Tor" ausloesen
		vxSafetyWait = false;												// ...Merker "vxSafetyUp" ruecksetzen
	}
	if (vxIsDown == true)	{												// Wenn Tor sich komplett schliesst dann...
		cntSafetyFail = 0;													// ...Zaehler "Fahrfehler Tor" wieder resetieren
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Innenbeleuchtung Stall"	***/

		// Licht per Hardware-PWM im Verhaeltnis "dimmlevel" (0..100%) getaktet (Timer2).
		
void beleuchtung()	{
	static unsigned long vulTime = 0;      									// momentane Laufzeit
	static bool vxState = false;      		          			    		// laufender Status
	static bool vxOldState = false;											// vorheriger Status

	if (vxState == false)  {                								// Wenn laufender Status FALSE dann...
		vulTime = millis();             									// ...permanent die Laufzeit merken
	}
	if (((inputs.TstLicht == true) && (vxOldState == false)) || (hmiLichtToggleRequest == true))	{	// Wenn Taster mit positiver Flanke gedrueckt ODER HMI-Taster dann...
		vxState = !vxState;													// ... wenn Zustand vorher AUS war, dann invertieren
	}
	hmiLichtToggleRequest = false;											// HMI-Anforderung nach Verarbeitung immer zuruecksetzen		
	if ((millis() - vulTime > ((unsigned long)lighttime*1000uL)))	{		// Wenn maximale Einschaltdauer erreicht dann...	
		vxState = false;													// ...laufender Status "vxState" auf FALSE
	}
	vxOldState = inputs.TstLicht;											// letzten Schaltzustand merken
	outputs.Licht = vxState;												// Dimmstufe (PWM-Tastgrad) wird in ausgaenge() per Hardware gesetzt
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Ausgangsvariablen"	***/

void ausgaenge()	{
	bool vaOutputs[anzahlPINOut] = {outputs.MotAuf, outputs.MotZu, outputs.Alarm, outputs.PowOn};		// Struct-Variable in Array uebergeben

	for (byte i = 0; i < anzahlPINOut; i++)	{								// for-Schlaufe mit n-Durchlaeufen fuer n-Ausgaenge
		digitalWrite(arrPINOut[i], vaOutputs[i] ? HIGH : LOW);				// Array-Wert dem jeweiligen Hardware-Ausgang zuweisen
	}
	if (outputs.Licht == true)	{											// Licht "Stall": Hardware-PWM
		analogWrite(OUTLicht, map(dimmlevel, 0, 100, 0, 255));				// Dimmstufe 0..100% auf PWM-Tastgrad 0..255 abbilden
	}else{
		analogWrite(OUTLicht, 0);
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Alarmhandling"	***/

void alarmhandling()	{
	unsigned long vulBlinkOn = 0;											// LED-Einschaltzeit
	unsigned long vulBlinkOff1 = 0;											// LED-Ausschaltzeit bei einfachem Blinktakt
	unsigned long vulBlinkOff2 = 0;											// LED-Ausschaltzeit bei doppeltem Blinktakt
	unsigned long vulBlinkTimeShort = 0;									// Berechung Rest aus "Dividation & Rest" fuer einfachen Blinktakt
	unsigned long vulBlinkTimeHalf = 0;										// Hilfsvariable fuer einfachere Lesbarkeit bei doppeltem Blinktakt
	unsigned long vulBlinkTimeLong = 0;										// Berechung Rest aus "Dividation & Rest" fuer doppelten Blinktakt
		
// Alarm-LED blinken lassen
	if ((skAlarm == true) || (motfuseAlarm == true))	{					// Wenn Alarmstatus "Schrittkettenablauf" oder "RM Motorsicherung" auf TRUE dann...
		outputs.Alarm = true;												// ...Alarm-LED permanent EIN
	}
	else if (safetyAlarm == true) {											// Sonst wenn Alarmstatus "Fahrfehler Tor" TRUE dann...
		vulBlinkOn = blinktime.MainOn;										// ...Variablen fuer Blinktakt "einfach" beschreiben
		vulBlinkOff1 = blinktime.MainOff;									// ..."dito"
		vulBlinkTimeShort = millis() % (vulBlinkOn + vulBlinkOff1);			// Berechung Rest aus "Dividation & Rest" fuer einfachen Blinktakt
		if (vulBlinkTimeShort < vulBlinkOff1) {               				// ...Wenn "Rest" kleiner als "vulBlinkOff1" dann...
			outputs.Alarm = false;											// 		...LED ausschalten
		}else{                          									// ...sonst...
		  outputs.Alarm = true;												// 		...LED einschalten
		}
	}else if (batterieAlarm == true) {										// Sonst wenn Alarmstatus "Batterieladung tief" dann...
		vulBlinkOn = blinktime.BattOn;										// ...Variablen fuer Blinktakt "doppelt" beschreiben
		vulBlinkOff1 = blinktime.BattFirstOff;								// ..."dito"
		vulBlinkOff2 = blinktime.BattSecondOff;								// ..."dito"
		vulBlinkTimeHalf = (vulBlinkOn + vulBlinkOff1 + vulBlinkOff2);		// Hilfsvariable fuer einfachere Lesbarkeit bei doppeltem Blinktakt
		vulBlinkTimeLong = millis() % (vulBlinkOn+vulBlinkOn+vulBlinkOff1+vulBlinkOff2);														// Berechung Rest aus "Dividation & Rest" fuer doppelten Blinktakt
		if ((vulBlinkTimeLong < vulBlinkOff1) || ((vulBlinkTimeLong > (vulBlinkOn+vulBlinkOff1)) && (vulBlinkTimeLong < vulBlinkTimeHalf)))	{ 	// ...wenn "Rest" kleiner als "vulBlinkOff1" ODER "Rest groesser als erste Blinksequenz" und "Rest kleiner vulBlinkTimeHalf" dann...
			outputs.Alarm = false;											// 		...LED ausschalten
		}else{                          									// ...sonst...
		  outputs.Alarm = true;												//		...LED einschalten
		}
	}else{                            										// sonst...
		outputs.Alarm = false;												// ...LED ausschalten
	}
	
// Alarme ruecksetzen
	if (TstFunc.Reset)  {													// Wenn Tasterfunktion "Reset" (Ursprung positive Flanke) TRUE dann...
		skAlarm = false;													// ...Alarmstatus "Schrittketten-Ablaufstoerung" resetieren
		motfuseAlarm = false;												// ...Alarmstatus "Motorsicherung ausgeloest" resetieren
		safetyAlarm = false;												// ...Alarmstatus "Fahrfehler Tor" resetieren
		cntSafetyFail = 0;													// ...Zaehler "Fahrfehler Tor" resetieren
		batterieAlarm = false;												// ...Alarmstatus "Batterieladung tief" resetieren								
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Bitmaske Eingangssignale"	***/

byte bitmaskStateInputs()	{												// Alle Inputsignale in einer Bitmaske zusammenfassen		
	
	return (inputs.Safety1 << 0) | (inputs.Safety2 << 1) | (inputs.TstTorAuf << 2)
		 | (inputs.TstTorZu << 3) | (inputs.TstLicht << 4) | (inputs.TstMultiReset << 5);
}

byte bitmaskStateAlarm()	{												// Alle Alarmzustaende in einer Bitmaske zusammenfassen
	
	return (skAlarm << 0) | (motfuseAlarm << 1) | (safetyAlarm << 2) | (batterieAlarm << 3);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "HMI Daten senden"	***/

		// case-Struktur damit der Buffer der seriellen Schnittstelle (max. 64Byte) in einem einzelnen Durchlauf die loop() nicht blockiert.
		// Zusaetzlich seitenbezogen aufgeteilt (aeusserer switch(currentPage)): es werden nur die Werte der aktuell auf dem Nextion angezeigten Seite gesendet.
		
void hmiSend()	{															// auf seitenbezogenen Versand umgebaut
	static unsigned long vulTime = 0;
	byte vAnzahlSeite = 1;													// Anzahl Werte auf der aktuellen Seite (fuer Rundlauf-Ende)

	if ((millis() - vulTime >= hmiSendStepTime) && (hmiReceiveInProgress == false))	{	// Periodische Aktualisierung - PAUSIERT waehrend ein Telegramm eintrifft
		vulTime = millis();

		switch (currentPage)	{											// nur Werte der aktuell angezeigten Seite
			case NEX_PAGE_MAIN:	{
				vAnzahlSeite = 2;
				switch (hmiSendIndex)	{
					case 0:	 nexSetValue(NEX_NAME_ACTSTATEALARM, bitmaskStateAlarm()); break;	// Signalzustand der Alarmstaten (Bitmaske)
					case 1:	{												// Revisionsbezeichnung, z.B. "F05"
						char vRevisionText[4];								// 1 Buchstabe + 2 Ziffern + Nullterminierung
						vRevisionText[0] = REVISION_SCHEMA;
						snprintf(&vRevisionText[1], sizeof(vRevisionText) - 1, "%02u", REVISION_CODE);
						nexSetText(NEX_NAME_ACTREVISION, vRevisionText);
					} break;
				}
			} break;
			case NEX_PAGE_DAYLIGHT:	{
				vAnzahlSeite = 4;
				switch (hmiSendIndex)	{
					case 0: nexSetValue(NEX_NAME_ACTDAYLIGHT, lightvalue); break;		// Rohwert Tageslicht
					case 1: nexSetValue(NEX_NAME_ACTSTATETAG, stateTag); break;			// Tag/Nacht-Status
					case 2: nexSetValue(NEX_NAME_GWTAG, gwValueTag); break;
					case 3: nexSetValue(NEX_NAME_GWNACHT, gwValueNacht); break;

				}
			} break;
			case NEX_PAGE_BOXLIGHT:	{
				vAnzahlSeite = 3;
				switch (hmiSendIndex)	{
					case 0: nexSetValue(NEX_NAME_DIMM, dimmlevel); break;
					case 1: nexSetValue(NEX_NAME_LIGHTTIME, lighttime); break;
					case 2: nexSetValue(NEX_NAME_ACTSTATELICHT, outputs.Licht); break;	// Ausgangszustand Licht Stall
				}
			} break;
			case NEX_PAGE_SYSTEM:	{
				vAnzahlSeite = 5;
				switch (hmiSendIndex)	{
					case 0: nexSetValue(NEX_NAME_ACTBATTRAW, batterieRaw); break;		// Rohwert der Batteriespannung
					case 1: nexSetValue(NEX_NAME_ACTBATTLEVEL, batterieProzent); break;	// Prozentwert der Batterieladung
					case 2: nexSetValue(NEX_NAME_ACTMOTFUSERAW, motfuseRaw); break;		// Rohwert RM Motorsicherung
					case 3:	nexSetValue(NEX_NAME_ACTSTATEMOTFUSE, motfuseAlarm); break;	// Alarmzustand Motorsicherung
					case 4: nexSetValue(NEX_NAME_ACTCYCLETIME, (long)cycleTime); break;	// Zykluszeit der CPU
				}
			} break;
			case NEX_PAGE_INPUTS:	{
				vAnzahlSeite = 1;
				nexSetValue(NEX_NAME_ACTSTATEINPUT, bitmaskStateInputs());				// Schalterzustand der Inputs (Bitmaske)
			} break;
		}

		hmiSendIndex++;
		if (hmiSendIndex >= vAnzahlSeite)	{
			hmiSendIndex = 0;															// nach dem letzten Wert dieser Seite wieder von vorne
		}
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Displayanzeie"	***/

		// case-Struktur gewaehlt, damit der Buffer der seriellen Schnittstelle (max. 64Byte) in einem einzelnen Durchlauf die loop() nicht blockiert
		// Somit ist sichergestellt das jeder case in einem einzigen Durchlauf verarbeitet werden kann.
		
void displayanzeige()	{
	static unsigned long vulTime = 0;										// laufende Diplayzeit initialisieren
	static byte vDebugSendIndex = 0;										// Index fuer Reihenfolge der naechsten Zeilen-Uebermittlung		
	byte vbStateInputs = 0;													// Ergebnisse FC "Bitmaske Inputs"
	byte vbStateAlarms = 0;													// Ergebnis FC "Bitmaske Alarmzustaende"
			
	if (millis() - vulTime >= debugStepTime) {								// Wenn "laufende Displayzeit" groesser Anzeigefrequenz dann...
		vulTime = millis();													// ...laufende Displayzeit aktualisieren
		
		switch (vDebugSendIndex)	{
			case 0:	{
				Serial.print("Helligkeit:    ");							// ...Anzeige Rohwert aktueller Lichtwert "Tageslicht"
				Serial.print(lightvalue);
				Serial.print("     ");
				Serial.print("Tagesstatus: ");								// ...Anzeige Status "Tag"
				Serial.print(stateTag ? "Tag" : "Nacht");
				Serial.println();
			} break;
			case 1:	{
				Serial.print("Grenzwert Tag: ");							// ...Anzeige GW Tag
				Serial.print(gwValueTag);
				Serial.print("     ");
				Serial.print("Grenzwert Nacht: ");							// ...Anzeige GW Nacht
				Serial.print(gwValueNacht);
				Serial.println();
			} break;

			case 2:	{
				Serial.print("Licht-Stall Einschaltdauer: ");				// ...Anzeige max.Einschaltzeit "Licht Stall"
				Serial.print(lighttime);
				Serial.print(" Sek.     ");
				Serial.print("Dimmstufe: ");								// ...Anzeige PWM-Dimmstufe "Licht Stall"
				Serial.print(dimmlevel);
				Serial.print(" %");
				Serial.println();
			} break;
			case 3:	{
				Serial.print("Spg.Motorsicherung: ");						// ...Anzeige Rohwert Messung "RM Motorsicherung"
				Serial.print(motfuseRaw);
				Serial.println();
				} break;
			case 4:	{
				Serial.print("Batterieladung:     ");						// ...Anzeige Rohwert Messung Batteriespannung
				Serial.print(batterieRaw);
				Serial.print(" (");
				Serial.print(batterieProzent);								// ...Anzeige Prozentwert Batterieladung
				Serial.println(" %)");
			} break;
			case 5:	{
				vbStateInputs = bitmaskStateInputs();						// ...Anzeige der entprellten Eingangssignale als reine Bitmaske					
				Serial.print("Schalterzustand (Bitmaske wie HMI): ");
				for (byte i=0; i < anzahlPINIn; i++)	{
					Serial.print(bitRead(vbStateInputs, i));	}
				Serial.println();
			} break;
			case 6:	{
				Serial.print("  [Safety1=");								// ...Anzeige der entprellten Eingangssignale als Text
				Serial.print(inputs.Safety1);
				Serial.print("  Safety2=");
				Serial.print(inputs.Safety2);
				Serial.print("  TorAuf=");
				Serial.print(inputs.TstTorAuf);
				Serial.print("  TorZu=");
				Serial.print(inputs.TstTorZu);
				Serial.print("  Licht=");
				Serial.print(inputs.TstLicht);
				Serial.print("  Reset=");
				Serial.print(inputs.TstMultiReset);
				Serial.println("]");
			} break;
			case 7:	{
				vbStateAlarms = bitmaskStateAlarm();							// ...Anzeige der Alarmstaten als reine Bitmaske
				Serial.print("Alarmzustand (Bitmaske wie HMI): ");				// 4 Alarmquellen (siehe bitmaskStateAlarm())
				for (byte i=0; i < 4; i++)	{
					Serial.print(bitRead(vbStateAlarms, i));	}
				Serial.println();
			} break;
			case 8:	{															// ...Anzeige der Alarmstaten als Text
				Serial.print("  [skAlarm=");
				Serial.print(skAlarm);
				Serial.print("  motfuseAlarm=");
				Serial.print(motfuseAlarm);
				Serial.print("  safetyAlarm=");
				Serial.print(safetyAlarm);
				Serial.print("  batterieAlarm=");
				Serial.print(batterieAlarm);
				Serial.println("]");
			} break;
			case 9:	{
				Serial.print("Ausgaenge:");									// ...Anzeige der aktuellen Ausgangssignale
				Serial.print("  [MotAuf=");
				Serial.print(outputs.MotAuf);
				Serial.print("  MotZu=");
				Serial.print(outputs.MotZu);
				Serial.print("  Licht=");
				Serial.print(outputs.Licht);
				Serial.print("  Alarm=");
				Serial.print(outputs.Alarm);
				Serial.print("  PowOn=");
				Serial.print(outputs.PowOn);
				Serial.println("]");
			} break;
			case 10:	{
				Serial.print("Zykluszeit: ");                       			// ...Anzeige der aktuellen Zykluszeit
				Serial.print(cycleTime);
				Serial.println(" Microsekunden");
				Serial.println("");
			} break;
		}
		
		vDebugSendIndex++;
		if (vDebugSendIndex >= anzahlZeilenanzeige)	{
			vDebugSendIndex = 0;											// nach der letzten Zeile wieder von vorne
		}
	}
	
	if (sleepAllowed() == true)	{											// Testausgabe fuer Sleepmode
		Serial.print("--> CPU-Laufzeit Sleepmode vor dem Schlafen...");
		Serial.println(millis());
		Serial.println("");
		Serial.flush();														// WICHTIG: Sendepuffer VOR dem Schlafen vollstaendig leeren, sonst bleiben Bytes haengen
		sleepNow();
		if (wdtWoke == true)	{											// Geweckt durch WDT-Timeout -> vollstaendiger ~8s-Zyklus abgelaufen
			Serial.println("<-- aufgewacht durch WDT (~8 Sekunden Sleep abgeschlossen)");
			Serial.print("<-- CPU-Laufzeit Sleepmode nach dem Schlafen...");
			Serial.println(millis());
			Serial.println("");
		}else if (interruptWoke)	{																// Geweckt durch Sammelinterrupt -> vorzeitig, weniger als 8s WDT-Timeout
			Serial.println("<-- aufgewacht durch Sammelinterrupt (vorzeitig, < 8 Sekunden)");
			Serial.print("<-- CPU-Laufzeit Sleepmode nach dem Schlafen...");
			Serial.println(millis());
			Serial.println("");
		}
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/***	FC "Zykluszeit berechnen"	***/

void cycle()	{
	static unsigned long vulTime = 0;              							// laufende Zyklus-Zwischenzeit
	static unsigned long vulCycleCount = 0;									// Zykluszaehler
  
	vulCycleCount++;                                         				// Zykluszaehler inkrementieren
  
	if (vulCycleCount >= 100uL)  {											// Wenn Zykluszaehler groesser 100
		cycleTime = ((micros()-vulTime)/vulCycleCount);   					// ...Zykluszeit berechnen 
		vulCycleCount = 0;                                    				// ...Zykluszaehler resetieren
		vulTime = micros();                          						// ...laufende Zyklus-Zwischenzeit aktualisieren
	}
return;
}


/******************************************************************************************************/
/******************************************************************************************************/