/**************************************************/
/***	ChickenGateSystem Rev.E					***/
/***	Step-4									***/
/***	Interrupt vorbereitet (OHNE Sleepmode)	***/
/***	Sensorspeisung KEIN bistabiles Relais	***/
/***	Reset-Taster: KEINE Multifunktion mehr	***/
/***	Helligkeit-Lernen entfernt (via HMI)	***/
/***	Software-PWM Innenbeleuchtung Stall		***/
/***	UART-Kommunikation Nextion-Touchpanel	***/
/**************************************************/

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	BIBLIOTHEKEN	***/

#include <EEPROM.h>							// Einbinden der EEPROM-Bibliothek fuer remanente Speicherung des Helligkeit-GW

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	DEKLARATIONEN	***/

const byte INADaylight = A0;   				// Analogwert Messung "Tageslicht"			-> Integerwert 0..1024
const byte INAMotfuse = A4;  				// Analogwert Messung "RM Motorsicherung"	-> Integerwert 0..1024
const byte INABattstate = A5;  				// Analogwert Messung "Batterieladung"		-> Integerwert 0..1024

const byte INInterrupt = 2;					// Sammel-Interrupt fuer Sleepmode (INT0) - NICHT in Array "arrPINIn" eingebunden

const byte INSafety1 = 3;					// Infrarotsensor "Safety-1-Innen"
const byte INSafety2 = 4;   				// Infrarotsensor "Safety-2-Aussen"
const byte INTstTorAuf = 5;  				// Taster "Tor AUF"
const byte INTstTorZu = 6;   				// Taster "Tor ZU"
const byte INTstLicht = 7;					// Taster "Licht Stall"
const byte INTstReset = 8;    				// Taster "Reset"
const byte OUTMotAuf = 9;					// Motorbefehl "Tor AUF"					(bistabiles Relais mit 2 Spulen)
const byte OUTMotZu = 10;   				// Motorbefehl "Tor ZU"						(bistabiles Relais mit 2 Spulen)
const byte OUTPowOn = 11;  					// Torsensoren & partielle Motorspeisung "einschalten"
const byte OUTLicht = 12;   				// Licht "Stall" einschalten
const byte OUTAlarm = 13;     				// Signal-LED "Alarm"

const byte arrPINIn[] = {INSafety1, INSafety2, INTstTorAuf, INTstTorZu, INTstLicht, INTstReset};		// Array "arrPINIn" definieren und initialisieren
const byte anzahlPINIn = sizeof(arrPINIn);																// Arraygroesse "arrPINIn" bestimmen (zwingend eine Konstante)
const byte arrPINOut[] = {OUTMotAuf, OUTMotZu, OUTPowOn, OUTLicht, OUTAlarm};							// Array "arrPINOut" definieren und initialisieren
const byte anzahlPINOut = sizeof(arrPINOut);															// Arraygroesse "arrPINOut" bestimmen (zwingend eine Konstante)

struct strINPUT	{																						// Struktur-Architektur fuer Eingaenge definieren
	bool Safety1;
	bool Safety2;
	bool TstTorAuf;
	bool TstTorZu;
	bool TstLicht;
	bool TstReset;
}	inputs = {false, false, false, false, false, false};												// 		...Struktur-Variable "inputs" erstellen und initialisieren
struct strOUTPUT	{																					// Struktur-Architektur fuer Ausgaenge definieren
	bool MotAuf;
	bool MotZu;
	bool PowOn;
	bool Licht;
	bool Alarm;
}	outputs = {false, false, false, false, false};														// 		...Struktur-Variable "outputs" erstellen und initialisieren	
struct strBLINK	{																						// Sruktur-Architektur fuer verschiedene Blinkzeiten definieren
	unsigned long MainOn;
	unsigned long MainOff;
	unsigned long BattOn;
	unsigned long BattFirstOff;
	unsigned long BattSecondOff;
} blinktime = {800, 400, 200, 2500, 400};																// 		...Struktur-Variable fuer "blinktime" erstellen und initialisieren

unsigned long prellTime = 10;   			// Entprellzeit fuer die Eingangssignale   			[in Millisekunden]
unsigned long photoTime = 60000; 		  	// Hysteresezeit der Tag/Nacht-Umschaltung   		[in Millisekunden]
unsigned long motfuseAlaTime = 3000;		// Alarmverzoegerung "RM Motorsicherung" hat ausgeloest
unsigned long battAlaTime = 5000;			// Alarmverzoegerung "Batterieladung" zu tief
unsigned long relaisTime = 500;				// Relais-Ansteuerzeit für die bistabilen Relais	[in Millisekunden]
unsigned long driveTime = 28000;			// maximale Fahrzeit vom Tor bis Endlage erreicht sein muss
unsigned long waitTime = 30000;				// zusaetzliche Wartezeit zur maximalen Fahrzeit vom Tor wenn ein "SafetyUp" ausgeloest wurde

unsigned long displayTime = 1000;			// Display-Anzeigefrequenz							[in Millisekunden]
unsigned long cycleTime = 0;				// aktuelle Zykluszeit								[in Microsekunden]

int gwValueNacht = 0;						// HMI-Grenzwertvorgabe fuer Helligkeit "NACHT"
int gwValueTag = 0;  						// HMI-Grenzwertvorgabe fuer Helligkeit "TAG"
int gwHystValue = 50;						// Hysteresebreite fuer Vorgabewert "TAG vs. NACHT"
int lightvalue = 0;							// aktueller Lichtwert "Tageslicht"
bool stateTag = true;    					// Status "Tag" beim Start auf "TRUE" initialisieren damit Tor geoeffnet wird

unsigned long pwmPeriod = 10000;			// Software-PWM-Periodendauer in Mikrosekunden (100 Hz)
int dimmlevel = 100;						// PWM-Dimmstufe "Licht Stall"						[0..100%]
int maxLightTime = 60;						// maximale Einschaltzeit "Licht Stall"				[in Sekunden]
bool lichtEin = false;						// Statusmerker "Licht Stall" fuer PWM-Dimming

int motfuseVolt = 0;						// aktuelle Spannung "RM Motorsicherung"
int gwMotfuseVolt = 546;					// Grenzwert "RM Motorsicherung"					[546=8.00V=Sicherung ausgeloest]
int batterieVolt = 0;						// aktuelle Batteriespannung
int gwBatterieVolt = 810;					// Grenzwert "Batteriespannung tief"				[810=11.85V=30%]

int cntSafetyFail = 0;						// laufender Zaehler "Fahrfehler Tor"
int gwSafetyFail = 3;						// Grenzwert Anzahl erlaubter "Fahrfehler Tor" bis Alarm ausgeloest wird

bool safetyAlarm = false;					// Alarmstatus "Fahrfehler Tor" ausgeloest
bool motfuseAlarm = false;					// Alarmstatus "RM Motorsicherung" (Sicherung ausgeloest)
bool batterieAlarm = false;					// Alarmstatus "Batteriespannung tief"
bool skAlarm = false;						// Alarmstatus "Schrittketten-Ablaufstoerung"

enum SK_TOR {STANDBY, AUTOAUF, AUTOZU, HANDAUF, HANDZU};		// ENum-Definition der SK "Torsteuerung"
	SK_TOR schrittTor = STANDBY;			// Variable "schrittTor" dem ENum zuweisen und Variable initialisieren

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	NEXTION-TOUCHDISPLAY	***/

		// component-ID (byte, bei Touch-Events zurueckgemeldet) und component-Name (String, fuer "get"/"set"-Textbefehle) sind hier PLATZHALTER. 
		// Sie muessen mit dem tatsaechlichen Nextion-Projekt (im Nextion Editor: Attribut "id" bzw. Objektname jeder Komponente) uebereinstimmen
		// Jede Komponente muss im Editor "Sends Component ID" bei Touch Release aktiviert haben, damit ein 0x65-Ereignis eintrifft.

const byte NEX_CID_GWTAG = 1;				// Component-ID Eingabefeld "Grenzwert Tag"
const byte NEX_CID_GWNACHT = 2;				// Component-ID Eingabefeld "Grenzwert Nacht"
const byte NEX_CID_DIMM = 3;				// Component-ID Eingabefeld "Dimmstufe Licht Stall"
const byte NEX_CID_MAXLIGHTTIME = 4;		// Component-ID Eingabefeld "max.Einschaltdauer Licht Stall"

const char NEX_NAME_GWTAG[] = "n0";			// Objektname im Nextion-Projekt - Platzhalter
const char NEX_NAME_GWNACHT[] = "n1";		// Objektname im Nextion-Projekt - Platzhalter
const char NEX_NAME_DIMM[] = "n2";			// Objektname im Nextion-Projekt - Platzhalter
const char NEX_NAME_MAXLIGHTTIME[] = "n3";	// Objektname im Nextion-Projekt - Platzhalter
const char NEX_NAME_ACTDAYLIGHT[] = "n4";	// Objektname im Nextion-Projekt - Platzhalter (nur Anzeige)
const char NEX_NAME_ACTMOTFUSE[] = "n5";	// Objektname im Nextion-Projekt - Platzhalter (nur Anzeige)
const char NEX_NAME_ACTBATTSTATE[] = "n6";	// Objektname im Nextion-Projekt - Platzhalter (nur Anzeige)
const char NEX_NAME_ACTSWITCHSTATE[] = "n7";// Objektname im Nextion-Projekt - Platzhalter (nur Anzeige)

enum HMI_REQUEST {HMI_NONE, HMI_REQ_GWTAG, HMI_REQ_GWNACHT, HMI_REQ_DIMM, HMI_REQ_MAXLIGHTTIME};
HMI_REQUEST hmiPendingRequest = HMI_NONE;	// aktuell offene "get"-Anfrage ans HMI
unsigned long hmiRequestTime = 0;			// Zeitpunkt der letzten "get"-Anfrage (fuer Timeout)
unsigned long hmiRequestTimeout = 1000;		// Timeout in ms, falls HMI nicht antwortet
unsigned long hmiSendTime = 2000;			// Sendefrequenz "hmiSend()" in Millisekunden

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FUNKTIONSPROTOTYPEN	***/

		// Wenn andere Entwicklungsumgebung als Arduino IDE verwendet wird. Ohne diese Prototypen keine korrekte Kompilierung.
		// Muss bei Funktionsaenderungen korrekt nachgefuehrt werden !!!

/*** Ausserhalb loop	***/
void isrInterrupt();
void speicherRead();
void speicherWrite();
void checkGwBereich();
void checkMinMax(int &viInput, int viGwLow, int viGwHigh);

/*** Nextion-HMI	***/
void nexFrameAuswerten(byte* frame, byte len);
void nexWertUebernehmen(long value);
void nexEnde();
void nexGetValue(const char* compName);
void nexSetValue(const char* compName, long value);

/*** loop-Ablauf	***/
void entprellen();
void daylight();
void motfuse();
void batterie();
void hmiRead();
void torsteuerung();
void beleuchtung();
void ausgaenge();
void alarmhandling();
void hmiSend();
void displayanzeige();
void cycle();

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	INTERRUPT-ROUTINE	***/

		// Schritt 1: keine Funktion noetig, da noch kein sleep_cpu() aufgerufen wird.
		// Ab Schritt 2 (Sleepmode): bleibt trotzdem leer - sie dient nur dem Aufwecken der CPU,
		// die eigentliche Auswertung erfolgt wie bisher ueber entprellen()/arrPINIn im Hauptprogramm.
		
void isrInterrupt()	{
	return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	SETUPCODE	***/

void setup()	{
	Serial.begin(9600);						// Serial Port für Anzeige oeffnen
	pinMode(INInterrupt, INPUT);			// Sammel-Interrupt fuer Sleepmode
		attachInterrupt(digitalPinToInterrupt(INInterrupt), isrInterrupt, RISING);		// Interrupt aus Sleepmode
	pinMode(INSafety1, INPUT);				// Infrarotsensor "Safety-1-Innen"
	pinMode(INSafety2, INPUT);    			// Infrarotsensor "Safety-2-Aussen"
	pinMode(INTstTorAuf, INPUT);			// Taster "Tor AUF"
	pinMode(INTstTorZu, INPUT); 			// Taster "Tor ZU"
	pinMode(INTstLicht, INPUT);				// Taster "Licht Stall"
	pinMode(INTstReset, INPUT);    			// Taster "Reset"
	pinMode(OUTMotAuf, OUTPUT);   			// Motorbefehl "Tor AUF"					(bistabiles Relais mit 2 Spulen)
	pinMode(OUTMotZu, OUTPUT);    			// Motorbefehl "Tor ZU"						(bistabiles Relais mit 2 Spulen)
	pinMode(OUTPowOn, OUTPUT);				// Torsensoren/partielle Motorspeisung "einschalten"
	pinMode(OUTLicht, OUTPUT);    			// Licht "Stall" einschalten
	pinMode(OUTAlarm, OUTPUT);    			// Signal-LED "Alarm"
	speicherRead();							// FC "Remanenter Speicher auslesen"
}

/***	HAUPTPROGRAMM	***/

void loop()	{
	entprellen();							// FC "Taster entprellen"
	daylight();								// FC "Messung Tageslicht"
	motfuse();								// FC "Messung RM Motorsicherung"
	batterie();								// FC "Messung Batterieladung"
	hmiRead();								// FC "HMI Daten empfangen"
	torsteuerung();							// FC "Torsteuerung"
	beleuchtung();							// FC "Innenbeleuchtung Stall"
	ausgaenge();							// FC "Ausgangsvariablen setzen"
	alarmhandling();						// FC "Alarmhandling" mit Parameteruebergabe der verschiedenen Blinkzeiten
	hmiSend();								// FC "HMI Daten senden"
	displayanzeige();						// FC "Displayanzeige"
	cycle();								// FC "Zykluszeit berechnen"
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Remanenter Speicher auslesen"	***/

void speicherRead()	{
	byte vbTagLowbyte = EEPROM.read(0);										// LowByte "GW-TAG" lesen
	byte vbTagHighbyte = EEPROM.read(1);									// HighByte "GW-TAG" lesen
	byte vbNachtLowbyte = EEPROM.read(2);									// LowByte "GW-NACHT" lesen
	byte vbNachtHighbyte = EEPROM.read(3);									// HighByte "GW-NACHT" lesen
	byte vbLightTimeLowbyte = EEPROM.read(4);								// LowByte "EINSCHALTDAUER" lesen
	byte vbLightTimeHighbyte = EEPROM.read(5);								// HighByte "EINSCHALTDAUER" lesen
	byte vbDimmlevel = EEPROM.read(6);										// Byte "DIMMSTUFE 0..100%" lesen
		
	gwValueTag = vbTagLowbyte + ((vbTagHighbyte << 8) & 0xFF00);			// Low- und HighByte "TAG" zusammenfuehren
	gwValueNacht = vbNachtLowbyte + ((vbNachtHighbyte << 8) & 0xFF00);		// Low- und HighByte "NACHT" zusammenfuehren
	checkGwBereich();														// Grenzwertvorgabe in FC pruefen
	
	maxLightTime = vbLightTimeLowbyte + ((vbLightTimeHighbyte << 8) & 0xFF00);	// Low- und HighByte "EINSCHALTDAUER LICHT STALL" zusammenfuehren
		
	checkMinMax(maxLightTime, 1, 1800);										// "EINSCHALTDAUER Licht Stall" -> Min/Max-Begrenzung in Sekunden
	checkMinMax(dimmlevel, 0, 100);											// "DIMMSTUFE Licht Stall" -> Min/Max-Begrenzung in Prozent
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Remanenter Specher schreiben"	***/

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
	vbLightTimeLowbyte = maxLightTime &0xFF;								// LowByte aus "EINSCHALTDAUER" extrahieren
	vbLightTimeHighbyte = (maxLightTime >> 8) &0xFF;						// HighByte aus "EINSCHALTDAUER" extrahieren
	vbDimmlevel = dimmlevel;												// Wertuebergabe von INT zu BYTE
	
	EEPROM.update(0, vbTagLowbyte);											// GW "TAG" LowByte in Speicher schreiben
	EEPROM.update(1, vbTagHighbyte);										// GW "TAG" HighByte in Speicher schreiben
	EEPROM.update(2, vbNachtLowbyte);										// GW "NACHT" LowByte in Speicher schreiben
	EEPROM.update(3, vbNachtHighbyte);										// GW "NACHT" HighByte in Speicher schreiben
	EEPROM.update(4, vbLightTimeLowbyte);									// GW "EINSCHALTDAUER" LowByte in Speicher schreiben
	EEPROM.update(5, vbLightTimeHighbyte);									// GW "EINSCHALTDAUER" HighByte in Speicher schreiben
	EEPROM.update(6, vbDimmlevel);											// PWM-Dimmstufe "Licht Stall" in Speicher schreiben
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Helligkeit-GW-Bereiche gegenseitig pruefen"	***/

void checkGwBereich()	{
	int viLowMin = 0;														// Unteres Bereichsende des Minimumwertes
	int viLowMax = 0;														// Oberes Bereichsende des Minimumwertes
	int viHighMin = 0;														// Unteres Bereichsende des Maximalwertes
	int viHighMax = 0;														// Oberes Bereichsende des Maximalwertes
	
	viLowMin = gwValueNacht;												// Bereichsende-Zuweisung des tieferen Nachtwertes
	if (viLowMin < 0)	{													// Check des tieferen "GW-NACHT"
		viLowMin = 0;	}
		
	viHighMax = gwValueTag;													// Bereichsende-Zuweisung des hoeheren Tagwertes
	if (viHighMax > 1024)	{												// Check des hoeheren "GW-TAG"
	viHighMax = 1024;	}	
	
	viLowMax = viHighMax - gwHystValue;										// Bereichsende definieren und erneut pruefen
	if (viLowMax < 0)	{
		viLowMax = 0;	}
		
	viHighMin = viLowMax + gwHystValue;										// Bereichsende definieren
	if (viLowMin > viLowMax)	{											// ggf. Bereichswerte korrigieren -> bedingt notwendig. Falscher Wert haette keinen Einfluss mehr
		viLowMin = viLowMax;	}
	if (viHighMax < viHighMin)	{											// ggf. Bereichswerte korrigieren -> bedingt notwendig. Falscher Wert haette keinen Einfluss mehr
		viHighMax = viHighMin;	}
	
	checkMinMax(gwValueNacht, viLowMin, viLowMax);							// "GW-NACHT" -> Min/Max-Begrenzung fuer Integer-Analogwert
	checkMinMax(gwValueTag, viHighMin, viHighMax);							// "GW-TAG" -> Min/Max-Begrenzung fuer Integer-Analogwert
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
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

/*******************************************************************************************************/
/*******************************************************************************************************/
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
			vaTaster[i].xState = false;										// ...und Laufzeit aktualisieren
		}
	}
		
	inputs.Safety1 = vaTaster[0].xMainstate;									// Ergebnisse aus for-Schleife den spezifischen Komponenten der Struktur "inputs.xy" zuweisen
	inputs.Safety2 = vaTaster[1].xMainstate;
	inputs.TstTorAuf = vaTaster[2].xMainstate;
	inputs.TstTorZu = vaTaster[3].xMainstate;
	inputs.TstLicht = vaTaster[4].xMainstate;
	inputs.TstReset = vaTaster[5].xMainstate;
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Messung Tageslicht"	***/

void daylight()	{
	static unsigned long vulTimeTag = 0;      								// laufende Hysteresezeit "Tag"
	static unsigned long vulTimeNacht = 0;     						   		// laufende Hysteresezeit "Nacht"
	static bool vxStateTag = false;                			    			// laufender Status "Tag"
	static bool vxStateNacht = false;            							// laufender Status "Nacht"
  
	lightvalue = analogRead(INADaylight);									// Lichtwert aus Photosensor auslesen -> Integerwert 0..1024

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
		vxStateTag = false;                  								// ...Laufzeit aktualisieren
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
		vxStateNacht = false;                								// ...Laufzeit aktualisieren
	}
return;	
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Messung RM Motorsicherung"	***/

void motfuse()	{
	static unsigned long vulTime = 0;      									// laufende Alarmverzoegerung "RM Motorsicherung""
	static bool vxState = false;                			    			// laufender Status "RM Motorsicherung""
	int vimotfuseVolt = 0;													// aktuelle Spannung "RM Motorsicherung"
  
	vimotfuseVolt = analogRead(INAMotfuse);									// Sicherungsspannung messen -> Integerwert 0..1024
	motfuseVolt = vimotfuseVolt;											// -> Ubertrag zu globaler Variable fuer Displayanzeie

// Zustand Motorsicherung ermitteln
	if (vxState == false)  {                								// Wenn laufender Status FALSE dann...
		vulTime = millis();             									// ...permanent die Laufzeit merken
	}
	if (vimotfuseVolt <= gwMotfuseVolt) {               					// Wenn Sicherungsspannung kleiner als Grenzwert dann...
		vxState = true;                 									// ...laufender Status auf TRUE
		if (millis() - vulTime > motfuseAlaTime)  {    						// ...wenn Alarmverzoegerung erreicht dann...
			motfuseAlarm = true;                    						// 		...Alarmstatus "Motorsicherung ausgeloest" auf TRUE
		}
    }else{																	// sonst...
		vxState = false;                  									// ...Laufzeit aktualisieren
	}
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Messung Batterieladung"	***/

void batterie()	{
	static unsigned long vulTime = 0;      									// laufende Alarmverzoegerung "Batteriespannung"
	static bool vxState = false;                			    			// laufender Status "Batteriespannung"
	int viBatterieVolt = 0;													// aktuelle Batteriespannung
  
	viBatterieVolt = analogRead(INABattstate);								// Batteriespannung messen -> Integerwert 0..1024
	batterieVolt = viBatterieVolt;											// -> Ubertrag zu globaler Variable fuer Displayanzeie

// Batterie-Ladezustand ermitteln
	if (vxState == false)  {                								// Wenn laufender Status FALSE dann...
		vulTime = millis();             									// ...permanent die Laufzeit merken
	}
	if (viBatterieVolt <= gwBatterieVolt) {               					// Wenn Batteriespannung kleiner als Grenzwert dann...
		vxState = true;                 									// ...laufender Status auf TRUE
		if (millis() - vulTime > battAlaTime)  {    						// ...wenn Alarmverzoegerung erreicht dann...
			batterieAlarm = true;                    						// 		...Alarmstatus "Batterieladung tief" setzen
		}
    }else{																	// sonst...
		vxState = false;                  									// ...Laufzeit aktualisieren
	}
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "UART-HMI: Daten empfangen"	***/

		// #Nextion-Telegramme sind IMMER mit 3x 0xFF terminiert. Zwei Telegrammtypen werden ausgewertet:
		//   0x65 <page> <component> <event>		-> Touch-Ereignis (event: 0=Release, 1=Press)
		//   0x71 <b0> <b1> <b2> <b3>				-> Rueckgabewert einer "get"-Anfrage (int32 little-endian)
		// Ablauf: Beim Loslassen einer bekannten Eingabekomponente wird per "get compname.val" aktiv nach dem
		// aktuellen Wert gefragt; die Antwort (0x71) wird dann dem zuvor gemerkten Parameter zugeordnet.

void hmiRead()	{
	static byte buf[16];													// Empfangspuffer fuer ein Telegramm (ohne Terminator)
	static byte bufLen = 0;
	static byte ffCount = 0;												// Zaehler aufeinanderfolgender 0xFF (Telegramm-Ende)

	while (Serial.available() > 0)	{										// Solange Zeichen im UART-Puffer warten...
		byte b = Serial.read();

		if (b == 0xFF)	{
			ffCount++;
		}else{
			if (bufLen < sizeof(buf))	{									// Nur puffern wenn noch Platz (Schutz vor Overflow)
				buf[bufLen++] = b;
			}
			ffCount = 0;
		}

		if (ffCount >= 3)	{												// Wenn Telegramm-Ende (3x 0xFF) erkannt dann...
			nexFrameAuswerten(buf, bufLen);									// ...Telegramm auswerten
			bufLen = 0;														// ...Puffer fuer naechstes Telegramm zuruecksetzen
			ffCount = 0;
		}
	}

	if ((hmiPendingRequest != HMI_NONE) && (millis() - hmiRequestTime > hmiRequestTimeout))	{	// Wenn offene Anfrage zu lange unbeantwortet dann...
		hmiPendingRequest = HMI_NONE;										// ...Anfrage verwerfen (kein Absturz/Haengenbleiben)
	}
return;
}

/*******************************************************************************************************/
/***	FC "UART-HMI: Telegramm auswerten"	***/

void nexFrameAuswerten(byte* frame, byte len)	{
	if (len == 0)	{
		return;
	}

	if ((frame[0] == 0x65) && (len >= 4))	{								// Touch-Ereignis: 0x65, page, component, event
		byte compId = frame[2];
		byte eventType = frame[3];

		if (eventType == 0x00)	{											// Nur bei "Loslassen" reagieren (Wert steht dann fest)
			HMI_REQUEST req = HMI_NONE;
			const char* name = nullptr;

			if (compId == NEX_CID_GWTAG)			{ req = HMI_REQ_GWTAG;			name = NEX_NAME_GWTAG;	}
			else if (compId == NEX_CID_GWNACHT)		{ req = HMI_REQ_GWNACHT;		name = NEX_NAME_GWNACHT;	}
			else if (compId == NEX_CID_DIMM)		{ req = HMI_REQ_DIMM;			name = NEX_NAME_DIMM;	}
			else if (compId == NEX_CID_MAXLIGHTTIME){ req = HMI_REQ_MAXLIGHTTIME;	name = NEX_NAME_MAXLIGHTTIME;	}

			if (req != HMI_NONE)	{										// Wenn eine bekannte Eingabekomponente betroffen ist dann...
				nexGetValue(name);											// ...aktuellen Wert aktiv anfragen
				hmiPendingRequest = req;									// ...und merken, worauf sich die Antwort  bezieht
				hmiRequestTime = millis();
			}
		}
	}
	else if ((frame[0] == 0x71) && (len >= 5))	{							// Rueckgabewert einer "get"-Anfrage: 0x71 + 4 Byte int32 LE
		long value = (long)frame[1] | ((long)frame[2] << 8) | ((long)frame[3] << 16) | ((long)frame[4] << 24);
		nexWertUebernehmen(value);
	}
return;
}

/*******************************************************************************************************/
/***	FC "UART-HMI: validierten Wert uebernehmen"	***/

void nexWertUebernehmen(long value)	{
	int neuerWert = (int)value;

	switch (hmiPendingRequest)	{
		case HMI_REQ_GWTAG:	{
			int vorTag = gwValueTag;											// Werte vor der Aenderung merken (checkGwBereich()
			int vorNacht = gwValueNacht;										// kann bei Bedarf BEIDE Grenzwerte anpassen)
			gwValueTag = neuerWert;
			checkGwBereich();													// Tag/Nacht/Hysterese-Verhaeltnis pruefen und ggf. korrigieren
			if ((gwValueTag != vorTag) || (gwValueNacht != vorNacht))	{		// Nur bei tatsaechlicher Aenderung...
				speicherWrite();												// ...remanent sichern
			}
		} break;
		case HMI_REQ_GWNACHT:	{
			int vorTag = gwValueTag;
			int vorNacht = gwValueNacht;
			gwValueNacht = neuerWert;
			checkGwBereich();
			if ((gwValueTag != vorTag) || (gwValueNacht != vorNacht))	{
				speicherWrite();
			}
		} break;
		case HMI_REQ_DIMM:	{
			int vorher = dimmlevel;
			dimmlevel = neuerWert;
			checkMinMax(dimmlevel, 0, 100);
			if (dimmlevel != vorher)	{
				speicherWrite();
			}
		} break;
		case HMI_REQ_MAXLIGHTTIME:	{
			int vorher = maxLightTime;
			maxLightTime = neuerWert;
			checkMinMax(maxLightTime, 1, 1800);
			if (maxLightTime != vorher)	{
				speicherWrite();
			}
		} break;
		default:
		break;
	}
	hmiPendingRequest = HMI_NONE;												// Anfrage abgeschlossen
return;
}

/*******************************************************************************************************/
/***	FC "UART-HMI: Hilfsfunktionen (Senden)"	***/

void nexEnde()	{															// Standard-Telegrammende (3x 0xFF) senden
	Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
return;
}

void nexGetValue(const char* compName)	{									// "get <component>.val" absenden
	Serial.print("get ");
	Serial.print(compName);
	Serial.print(".val");
	nexEnde();
return;
}

void nexSetValue(const char* compName, long value)	{						// "<component>.val=<value>" absenden
	Serial.print(compName);
	Serial.print(".val=");
	Serial.print(value);
	nexEnde();
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
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

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Innenbeleuchtung Stall"	***/

		// Licht per Software-PWM (kooperativ, ueber micros()) im Verhaeltnis "dimmlevel" (0..100%) getaktet, da Ausgabepin kein HW-PWM unterstutzt.
		// WICHTIG: Diese Software-PWM funktioniert nur, solange loop() regelmaessig durchlaeuft - sie darf nicht im Sleepmode aktiv sein.
		
void beleuchtung()	{
	
	static unsigned long vulTime = 0;      									// momentane Laufzeit
	static bool vxState = false;      		          			    		// laufender Status
	static bool vxOldState = false;											// vorheriger Status
	static unsigned long pwmCycleStart = 0;									// Start der aktuellen PWM-Periode
	unsigned long pwmOnTime;												// Einschaltdauer innerhalb der Periode, gemaess Parameter

	if (vxState == false)  {                								// Wenn laufender Status FALSE dann...
		vulTime = millis();             									// ...permanent die Laufzeit merken
	}
	if ((inputs.TstLicht == true) && (vxOldState == false))	{				// Wenn Taster mit positiver Flanke gedrueckt dann...
		if (vxState == false)	{											// ... wenn Zustand vorher AUS war, dann
			vxState = true;													// ...laufender Status "vxState" auf TRUE
		} else	{															// ... wenn Zustand vorher EIN war, dann
			vxState = false;												// ...laufender Status "vxState" auf FALSE
		}
	}
	if ((millis() - vulTime > ((unsigned long)maxLightTime*1000uL)))	{	// Wenn maximale Einschaltdauer erreicht dann...	
		vxState = false;													// ...laufender Status "vxState" auf FALSE
	}
	vxOldState = inputs.TstLicht;											// letzten Schaltzustand merken
	lichtEin = vxState;														// Zustand an PWM-Ablauf uebergeben

	if (lichtEin == true)	{
			pwmOnTime = (unsigned long)dimmlevel * pwmPeriod / 100;			// Einschaltzeit aus Dimmstufe berechnen (0..pwmPeriod)
		if (micros() - pwmCycleStart >= pwmPeriod)	{						// Wenn aktuelle Periode abgelaufen dann...
			pwmCycleStart = micros();										// ...neue Periode starten
		}
		if ((micros() - pwmCycleStart) < pwmOnTime)	{						// Wenn innerhalb der Einschaltzeit dann...
			outputs.Licht = true;											// ...Ausgang EIN
		}else{																// sonst (innerhalb der Periode, aber nach Einschaltzeit)...
			outputs.Licht = false;											// ...Ausgang AUS
		}
	} else {
		outputs.Licht = false;
	}
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Ausgangsvariablen"	***/

void ausgaenge()	{
		
	if (outputs.MotAuf == true)												// Variable "MotAuf" dem Hardware-Ausgang zuweisen
		digitalWrite(arrPINOut[0], HIGH);
		else
			digitalWrite(arrPINOut[0], LOW);
		
	if (outputs.MotZu == true)												// Variable "MotZu" dem Hardware-Ausgang zuweisen
		digitalWrite(arrPINOut[1], HIGH);
		else
			digitalWrite(arrPINOut[1], LOW);
		
	if (outputs.PowOn == true)												// Variable "PowOn" dem Hardware-Ausgang zuweisen
		digitalWrite(arrPINOut[2], HIGH);
		else
			digitalWrite(arrPINOut[2], LOW);

	if (outputs.Licht == true)												// Variable "Licht" dem Hardware-Ausgang zuweisen
		digitalWrite(arrPINOut[3], HIGH);
		else
			digitalWrite(arrPINOut[3], LOW);
		
	if (outputs.Alarm == true)												// Variable "Alarm" dem Hardware-Ausgang zuweisen
		digitalWrite(arrPINOut[4], HIGH);
		else
			digitalWrite(arrPINOut[4], LOW);
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Alarmhandling"	***/

void alarmhandling()	{
	unsigned long vulBlinkOn = 0;											// LED-Einschaltzeit
	unsigned long vulBlinkOff1 = 0;											// LED-Ausschaltzeit bei einfachem Blinktakt
	unsigned long vulBlinkOff2 = 0;											// LED-Ausschaltzeit bei doppeltem Blinktakt
	unsigned long vulBlinkTimeShort = 0;									// Berechung Rest aus "Dividation & Rest" fuer einfachen Blinktakt
	unsigned long vulBlinkTimeHalf = 0;										// Hilfsvariable fuer einfachere Lesbarkeit bei doppeltem Blinktakt
	unsigned long vulBlinkTimeLong = 0;										// Berechung Rest aus "Dividation & Rest" fuer doppelten Blinktakt
	
// Blinktakt generieren
	if (safetyAlarm == true)  {       						      			// Wenn Alarmstatus "Fahrfehler Tor" TRUE dann...
		vulBlinkOn = blinktime.MainOn;										// ...Variablen fuer Blinktakt "einfach" beschreiben
		vulBlinkOff1 = blinktime.MainOff;									// ..."dito"
	}else if (batterieAlarm == true)	{       	           		     	// sonst wenn Alarmstatus "Batterie-Ladezustand tief" dann...
		vulBlinkOn = blinktime.BattOn;										// ...Variablen fuer Blinktakt "doppelt" beschreiben
		vulBlinkOff1 = blinktime.BattFirstOff;								// ..."dito"
		vulBlinkOff2 = blinktime.BattSecondOff;								// ..."dito"
	}
	vulBlinkTimeShort = millis() % (vulBlinkOn + vulBlinkOff1);						// Berechung Rest aus "Dividation & Rest" fuer einfachen Blinktakt
	vulBlinkTimeHalf = (vulBlinkOn + vulBlinkOff1 + vulBlinkOff2);					// Hilfsvariable fuer einfachere Lesbarkeit bei doppeltem Blinktakt
	vulBlinkTimeLong = millis() % (vulBlinkOn+vulBlinkOn+vulBlinkOff1+vulBlinkOff2);// Berechung Rest aus "Dividation & Rest" fuer doppelten Blinktakt
	
// Alarm-LED blinken lassen
	if ((skAlarm == true) || (motfuseAlarm == true))	{					// Wenn Alarmstatus "Schrittkettenablauf" oder "RM Motorsicherung" auf TRUE dann...
		outputs.Alarm = true;												// ...Alarm-LED permanent EIN
	}
	else if (safetyAlarm == true) {											// Ansonsten wenn Alarm "Fahrfehler Tor" TRUE dann...
		if (vulBlinkTimeShort < vulBlinkOff1) {               				// ...Wenn "Rest" kleiner als "vulBlinkOff1" dann...
			outputs.Alarm = false;											// 		...LED ausschalten
		}else{                          									// ...sonst...
		  outputs.Alarm = true;												// 		...LED einschalten
		}
	}else if (batterieAlarm == true) {										// Ansonsten wenn Alarm "Batterie-Ladezustand tief" TRUE dann...
		if ((vulBlinkTimeLong < vulBlinkOff1) || ((vulBlinkTimeLong > (vulBlinkOn+vulBlinkOff1)) && (vulBlinkTimeLong < vulBlinkTimeHalf)))	{ 	// ...Wenn "Rest" kleiner als "vulBlinkOff1" ODER "Rest groesser als erste Blinksequenz" und "Rest kleiner vulBlinkTimeHalf" dann...
			outputs.Alarm = false;											// 		...LED ausschalten
		}else{                          									// ...sonst...
		  outputs.Alarm = true;												//		...LED einschalten
		}
	}else{                            										// sonst...
		outputs.Alarm = false;												// ...LED ausschalten
	}
	
// Alarme ruecksetzen
	if (inputs.TstReset == true)  {                   						// Wenn Taster "Reset" TRUE dann...
		skAlarm = false;													// ...Alarmstatus "Schrittketten-Ablaufstoerung" resetieren
		safetyAlarm = false;												// ...Alarmstatus "Fahrfehler Tor" resetieren
		cntSafetyFail = 0;													// ....Zaehler "Fahrfehler Tor" resetieren
		motfuseAlarm = false;												// ...Alarmstatus "Motorsicherung ausgeloest" resetieren
		batterieAlarm = false;												// ...Alarmstatus "Batterieladung tief" resetieren								
	}
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "HMI Daten senden"	***/

void hmiSend()	{
	static unsigned long vulTime = 0;
	byte switchState = 0;													// Bitmaske der entprellten Eingaenge

	if (millis() - vulTime >= hmiSendTime)	{								// Periodische Aktualisierung
		vulTime = millis();

		switchState = (inputs.Safety1 << 0) | (inputs.Safety2 << 1) | (inputs.TstTorAuf << 2)
					| (inputs.TstTorZu << 3) | (inputs.TstLicht << 4) | (inputs.TstReset << 5);
					
		nexSetValue(NEX_NAME_GWTAG, gwValueTag);
		nexSetValue(NEX_NAME_GWNACHT, gwValueNacht);
		nexSetValue(NEX_NAME_DIMM, dimmlevel);
		nexSetValue(NEX_NAME_MAXLIGHTTIME, maxLightTime);
		nexSetValue(NEX_NAME_ACTDAYLIGHT, lightvalue);
		nexSetValue(NEX_NAME_ACTMOTFUSE, motfuseVolt);
		nexSetValue(NEX_NAME_ACTBATTSTATE, batterieVolt);
		nexSetValue(NEX_NAME_ACTSWITCHSTATE, switchState);
	}
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/***	FC "Displayanzeie"	***/

void displayanzeige()	{
	static unsigned long vulTime = 0;										// laufende Diplayzeit initialisieren
			
	if (millis() - vulTime >= displayTime) {								// Wenn "laufende Displayzeit" groesser Anzeigefrequenz dann...
		vulTime = millis();													// ...laufende Displayzeit aktualisieren
/*
		Serial.print("Erster Eingang: ");									// ...Anzeige Zustand Taster-1
		Serial.println(inputs.Safety1);
		Serial.print("Zweiter Eingang: ");									// ...Anzeige Zustand Taster-2
		Serial.println(inputs.Safety2);
		Serial.print("Dritter Eingang: ");									// ...Anzeige Zustand Taster-3
		Serial.println(inputs.TstTorAuf);
		Serial.print("Vierter Eingang: ");									// ...Anzeige Zustand Taster-4
		Serial.println(inputs.TstTorZu);
		Serial.print("Fuenfter Eingang: ");									// ...Anzeige Zustand Taster-5
		Serial.println(inputs.TstLicht);
		Serial.print("Sechster Eingang: ");									// ...Anzeige Zustand Taster-6
		Serial.println(inputs.TstReset);
		Serial.println();
*/
		Serial.print("Helligkeit: ");										// ...Anzeige aktueller Lichtwert
		Serial.print(lightvalue);
		Serial.print("   ");
		Serial.print("Grenzwert Tag: ");									// ...Anzeige GW Tag
		Serial.print(gwValueTag);
		Serial.print("   ");
		Serial.print("Grenzwert Nacht: ");									// ...Anzeige GW Nacht
		Serial.print(gwValueNacht);
		Serial.print("   ");
		Serial.print("Tagesstatus: ");										// ...Anzeige Tagesstatus
		Serial.print(stateTag);
		Serial.print("   ");
		Serial.println();
		
		Serial.print("Spg.Motorsicherung: ");								// ...Anzeige aktuelle Sicherungsspannung "RM Motorsicherung"
		Serial.print(motfuseVolt);
		Serial.println("   ");
		
		Serial.print("Batterieladung: ");									// ...Anzeige aktuelle Batteriespannung
		Serial.print(batterieVolt);
		Serial.println("   ");
			
		Serial.print("Zykluszeit: ");                       				// ...Anzeige der aktuellen Zykluszeit
		Serial.print(cycleTime);
		Serial.println(" Microsekunden");
		Serial.println("");
	}
return;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
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

/*******************************************************************************************************/