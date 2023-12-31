#include <INA226_WE.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_SHT4x.h"
#include "Adafruit_FONA.h"
#include <OneWire.h>
#include <Adafruit_BME280.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <PwFusion_MAX31865.h> 

//#define DEBUG ON

// Kalibracje dla r�nych sensor�w
float kalibracjaSHT45 = -0.1;
float kalibracjaPT100 = 0.37;
float kalibracjaDS18_1 = 0;
float kalibracjaDS18_2 = 0.1;

#define TS_API_KEY "S2SC0GMEVLC1Q1KQ"
#define TS_SUB_API_KEY "BCB7R7R2BPUTIQF4"	 

//Serial for Debugging
#define RX_PIN 5 
#define TX_PIN 3
#define BAUD 115200

//Setup I2C BUS
TwoWire I2CSHT = TwoWire(0);
#define I2C_SDA 1
#define I2C_SCL 2

//GSM Settings
#define APN_NAME  "internet"
#define APN_USER_NAME   ""
#define APN_PASSWORD    ""
#define FONA_RST		39
#define FONA_TX			38
#define FONA_RX			37
#define SIM_PWR_FET     16
#define SENSOR_PWR_FET  10

//How often device will attempt to send the data 
//Data upload takes around 30s 
#define CHECK_INTERVAL_MIN 5

//Number of samples for SHT 
#define SHT_READ_COUNT 1 

float temp[SHT_READ_COUNT];
float humidity[SHT_READ_COUNT];

uint16_t length = 0;

//PT100 
MAX31865 maxReader;
#define RTD_TYPE_PT100_R430    430
#define CS_A    17
#define DRDY_A  34
#define SDI     33
#define SDO     21
#define CLK     18
float PT100temp = -255;

//DS18B20
const int oneWireBus = 14;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
float DSTempC[2] = { -255, -255 };
float DSTempF[2] = { -255, -255 };

//BME280
Adafruit_BME280 bme280; // I2C
#define SEALEVELPRESSURE_HPA (1005.2)
float BMEtempReadC = -255;
float BMEtempReadF = -255;
float BMEhumRead = -255;
float BMEpressureReadHPA = -255;
float BMEAltitudeM = -255;

//SHT
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
float SHTtempRead = -255;
float SHTtempReadF = -255;
float SHThumRead = -255;

//INA226 
#define I2C_ADDRESS 0x40 
INA226_WE ina226(&I2CSHT, I2C_ADDRESS);
float busVoltage;
float extVoltage;
float extCurrent;
char battV[5];

//SIM800
#define SIMBAUD 115200
Adafruit_FONA SIM800 = Adafruit_FONA(FONA_RST);
bool useSIM800 = true;
char imei[30];
char simCCID[25];

char simV[5];
float simVoltage;
unsigned int voltage = 0;
uint16_t simBat;
int net_status = 0;
int8_t rdbm = 0;				

bool hasSHT = false;
bool hasDS = false;
bool hasMAX = false;
bool hasBME = false;


bool mainSendSuccess = false;
bool subSendSuccess = false;							 
							

enum EState {
	BootOK,
	CollecData,
	ConnectCarrier,
	Sleep,
	SendMainData,
	SendSubData,
	NoSim,
	GSMFail,
};

EState _state;

void setup() {

	pinMode(SIM_PWR_FET, OUTPUT);
	digitalWrite(SIM_PWR_FET, LOW);
	delay(200);
	digitalWrite(SIM_PWR_FET, HIGH);

	pinMode(SENSOR_PWR_FET, OUTPUT);
	digitalWrite(SENSOR_PWR_FET, HIGH);
	delay(100);
	digitalWrite(SENSOR_PWR_FET, HIGH);

	Serial.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
	Serial1.begin(SIMBAUD, SERIAL_8N1, FONA_RX, FONA_TX);

	Serial.println(F("..........................."));
	Serial.println(F(".. Weather Station v1.0b .."));
	Serial.println(F("..........................."));

	//INA226
	I2CSHT.begin(I2C_SDA, I2C_SCL, 100000);

	while (!ina226.init()) {
		Serial.println(F("ERROR:Couldn't find INA226"));
		BlinkLedNTimes(10, 0.2f);
	}

	ina226.setMeasureMode(CONTINUOUS);
	ina226.setAverage(AVERAGE_4);
	ina226.setResistorRange(0.1, 1.3);
	ina226.waitUntilConversionCompleted();

	delay(20);
	Serial.println(F("INA226 Ready!"));

	ina226.readAndClearFlags();
	busVoltage = ina226.getBusVoltage_V();
	extVoltage = ina226.getShuntVoltage_V();
	extCurrent = ina226.getCurrent_mA();
	ina226.powerDown();

	Serial.print(F("Batt Voltage [V]:"));
	Serial.println(busVoltage);
	Serial.print(F("Ext Shunt Voltage [mV]:"));
	Serial.println(extVoltage);
	Serial.print(F("EXT Current [mA]:"));
	Serial.println(extCurrent);
	//END INA226

	//SHT45
	if (!sht4.begin(&I2CSHT)) {
		Serial.println(F("SHT4x Not found!"));
	}
	else {
		hasSHT = true;
		Serial.println(F("Found SHT4x sensor"));
		Serial.print(F("Serial number 0x"));
		Serial.println(sht4.readSerial(), HEX);

		sht4.setPrecision(SHT4X_HIGH_PRECISION);
		sht4.setHeater(SHT4X_NO_HEATER);
		Serial.println(F("SHT4x Ready!"));
	}
	//END SHT45

	//BME280
	if (!bme280.begin(0x76, &I2CSHT)) {
		Serial.println(F("BME280 Not found !"));
	}
	else {
		hasBME = true;
	}
	//END BME280

	//PT100
	Serial.println(F("MAX31865+PT100 Sensor Start"));
	pinMode(CS_A, OUTPUT);

	SPIClass SPI;
	SPI.begin(CLK, SDO, SDI, CS_A);
	maxReader.begin(CS_A, RTD_3_WIRE, RTD_TYPE_PT100_R430, SPI);
	maxReader.setLowFaultTemperature(-70);
	maxReader.setHighFaultTemperature(70);
	int ptTryCNT = 0;
	while (ptTryCNT <= 3)
	{
		maxReader.sample();
		float r = maxReader.getResistance();
		Serial.print(r);
		Serial.print(F(" Ohms,   "));
		PT100temp = GetPlatinumRTD2(r, 100) - kalibracjaPT100;
													
		PrintRTDStatus(maxReader.getStatus());
		if (PT100temp > -60 && PT100temp < 70)
		{
			hasMAX = true;
			Serial.println(F("MAX31865 Found!"));
			break;
		}

		delay(150);
		ptTryCNT++;
	}
	if (!hasMAX) Serial.println(F("MAX31865 Not Found !"));

	//END PT100

	//SIM800
	if (useSIM800) {
		Serial.println(F("Starting SIM800"));
		if (!SIM800.begin(Serial1)) {
			Serial.println(F("Couldn't find SIM800L"));
			_state = NoSim;
			return;
		}

		while (!SIM800.sendCheckReply(F("AT"), F("OK"), 1000))
			delay(40);
		while (!SIM800.sendCheckReply(F("AT+CSCLK=0"), F("OK"), 1000))
			delay(100);

		//SIM800.sendCheckReply(F("AT+CFUN=0"), F("OK"), 1000);
		SIM800.getIMEI(imei);
		SIM800.getBattVoltage(&simBat);
		Serial.print(F("SIM800 IMEI:"));	Serial.println(imei);
		Serial.print(F("SIM800 Voltage:"));	Serial.println((float)simBat / 1000);

		//SIM800.getSIMCCID(simCCID);
		//Serial.print(F("SIM800 SimCCID:"));	Serial.println(simCCID);

		SIM800.sendCheckReply(F("AT+CMEE=0"), F("OK"), 500);

		if (SIM800.sendCheckReply(F("AT+CBAND?"), F("+CBAND: EGSM_MODE,ALL_BAND"), 1000U)) { Serial.println(F("Band's OK: EGSM_MODE,ALL_BAND")); }
		else { Serial.println(F("Invalid Band... setting up: EGSM_MODE,ALL_BAND")); SIM800.sendCheckReply(F("AT+CBAND=\"EGSM_MODE, ALL_BAND\""), F("OK")); }

		SIM800.setGPRSNetworkSettings(F(APN_NAME), F(APN_USER_NAME), F(APN_PASSWORD));

		if (SIM800.sendCheckReply(F("AT+CPIN?"), F("+CPIN: READY")))
		{
			Serial.println(F("SIM800 Ready!"));
			_state = BootOK;
		}
		else
		{
			_state = NoSim;
		}
	}
	else
	{
		//setup WIFI communication
	}
	Serial.println(F("Boot Completed."));

}

void loop() {

	//while (useSIM800 && Serial1.available())
	//    Serial.write(Serial1.read());

	switch (_state)
	{
	case BootOK:
	{
		Serial.println(F("State:BootOK"));
		Serial.println(F("Read in 1s"));
		BlinkLedNTimes(3, 0.2);
		_state = CollecData;

	}break;
	case CollecData:
	{
		Serial.println(F("State:CollectData"));
		GetPT100Readings();
		GetSHTReadings();
		GetBMEReadings();
		GetDSReadings();

		dtostrf(busVoltage, 4, 3, battV);

		simVoltage = (float)simBat / 1000;
		dtostrf(simVoltage, 4, 3, simV);

		_state = ConnectCarrier;
	}break;
	case NoSim:
	{
		Serial.println(F("State:NoSIM"));
		Serial.println(F("No Sim Card Inserted. Reset in 1min"));
		BlinkLedNTimes(4, 1);
		delay(60000);
		ESP.restart();

	}break;
	case GSMFail:
	{
		Serial.println(F("GSM Failure"));
		BlinkLedNTimes(10, 0.5);
		delay(60000);
		ESP.restart();

	}break;
	case ConnectCarrier:
	{
		Serial.println(F("State:ConnectCarrier"));
		uint8_t gsmInitCnt = 0;
		while (!InitGSM()) //try attaching and if it fail we will restart the whole thing.
		{
			if (gsmInitCnt == 5) {
				Serial.println(F("GSM Init Limit Reached ... Restarting ESP"));
				ESP.restart();
			}
			gsmInitCnt++;
			delay(200);
		}
		uint8_t rssi = SIM800.getRSSI();

		int8_t rdbm = 0;
		if (rssi == 0) rdbm = -115;
		if (rssi == 1) rdbm = -111;
		if (rssi == 31) rdbm = -52;
		if ((rssi >= 2) && (rssi <= 30)) {
			rdbm = map(rssi, 2, 30, -110, -54);
		}

		Serial.print(F("RSSI signal:")); Serial.println(rssi);
		if (rssi > 4) {
			_state = SendMainData;
		}
		else {
			Serial.println(F("Bad Signal!, Trying again in 5s"));
			delay(5000);
			_state = ConnectCarrier;
		}
	}break;
	case SendMainData:
	{
		Serial.println(F("State:SendMainData"));						 
   
		//Optional for fallback readings.
		float humidity = SHThumRead;
		float extTemp = SHTtempRead;
		float intTemp = 0;

				 				  
		char creq[230];
		if (ValidateData(battV, SHTtempRead, SHThumRead, simVoltage))
			sprintf(creq, "api.thingspeak.com/update?api_key=%s&field1=%f&field2=%f&field3=%f&field4=%f&field5=%f&field6=%f&field7=%s&field8=%d", TS_API_KEY, extTemp, PT100temp, DSTempC[0], humidity, DSTempC[1], BMEpressureReadHPA, battV, rdbm);
			
		Serial.println(F("Sending Data..."));
		if (!SendData(creq))
		{
			Serial.println(F("Sending failure, restarting station"));
			
		}
		else {
			Serial.println(F("Sending Success..."));
			mainSendSuccess = true;
		}
		battV[0] = '\n';
		creq[0] = '\n';
		if (mainSendSuccess) {
			_state = SendSubData;
		}
		else
		{
			_state = GSMFail;
		}
	}break;
	case SendSubData:
	{
		char creq[230];
		sprintf(creq, "api.thingspeak.com/update?api_key=%s&field1=%f&field2=%f&field3=%f&field4=%f&field5=%f&field6=%f", TS_SUB_API_KEY, BMEhumRead, BMEtempReadC, BMEAltitudeM, SHTtempReadF, extCurrent, simBat);
		Serial.println(F("Sending Second Data..."));
		if (!SendData(creq))
		{
			Serial.println(F("Second Sending failure, restarting station"));
		}
		else {
			Serial.println(F("Sending Success..."));
			subSendSuccess = true;
				 
		}
				 

		simV[0] = '\n';
		creq[0] = '\n';

		//if main send was ok then we dont need to care for the second packed, just go sleep.
		if (mainSendSuccess) {
			_state = Sleep;
		}
		if (mainSendSuccess)
		{
			_state = GSMFail;
		}

				 
	}break;
	case Sleep: {

		Serial.println(F("State:Sleep"));
		Serial.println(F("SIM POWER OFF..."));
		digitalWrite(SIM_PWR_FET, LOW);
		digitalWrite(SENSOR_PWR_FET, LOW);

		Serial.println(F("ESP go sleep..."));
		Serial.println(F("........................"));

		ESP.deepSleep(CHECK_INTERVAL_MIN * 60000000);

	}break;
	}
}

void BlinkLedNTimes(uint8_t n, float intervalS)
{
	short cstate = digitalRead(LED_BUILTIN);
	for (uint8_t i = 0; i < n; i++)
	{
		digitalWrite(LED_BUILTIN, LOW);
		delay(1000 * intervalS);
		digitalWrite(LED_BUILTIN, HIGH);
		delay(1000 * intervalS);
	}
	digitalWrite(LED_BUILTIN, cstate);
}

bool InitGSM()
{
	Serial.println(F("GSM Initialising..."));
	int GSMCNT = 1;
	while (net_status != 1 && net_status != 5) { //1 home network or 5 roaming network 
		Serial.print(F("GSM Attempt:")); Serial.println(GSMCNT);
		net_status = SIM800.getNetworkStatus();
		if (net_status == 1 || net_status == 5)
		{
			Serial.println(F("GSM Attached."));
			SIM800.enableGPRS(true);
			delay(1000);
			GSMCNT = 1;
			break;
		}
		if (GSMCNT == 10)
		{
			Serial.println(F("GSM failed 10 times... restarting SIM800..."));
			if (!ToggleGSModule(false))
			{
				Serial.println(F("Airplane mode failed."));
				return false;
			}
		}
		if (GSMCNT == 20)
		{
			Serial.println(F("GSM not ready after 20"));
			return false;
		}
		GSMCNT++;
		delay(3500);
	}

	int gprsCNT = 1;
	int gprsState = SIM800.GPRSstate();
	while (gprsState != 1 && !SIM800.enableGPRS(true)) {

		Serial.print(F("GPRS Attempt:")); Serial.println(gprsCNT);
		Serial.print("GPRS:"); Serial.println(SIM800.GPRSstate());
		if (gprsCNT == 15)
		{
			Serial.println(F("Unable to get GPRS connection trying again to reattach GSM"));
			return false;
		}
		gprsCNT++;
		delay(1000);
	}
	Serial.println(F("GPRS Attached."));

	return true;
}

void GetPT100Readings()
{
	if (!hasMAX) return;
	Serial.print(F("PT100:Temp: "));
	Serial.print(PT100temp);
	Serial.println(F(" deg C"));
}

void PrintRTDStatus(uint8_t status)
{
	// status will be 0 if no faults are active
	if (status == 0)
	{
		Serial.print(F("OK"));
	}
	else
	{
		// status is a bitmask, so multiple faults may be active at the same time

		// The RTD temperature is above the threshold set by setHighFaultTemperature()
		if (status & RTD_FAULT_TEMP_HIGH)
		{
			Serial.print(F("RTD High Threshold Met, "));
		}

		// The RTD temperature is below the threshold set by setHLowFaultTemperature()
		if (status & RTD_FAULT_TEMP_LOW)
		{
			Serial.print(F("RTD Low Threshold Met, "));
		}

		// The RefIn- is > 0.85 x Vbias
		if (status & RTD_FAULT_REFIN_HIGH)
		{
			Serial.print(F("REFin- > 0.85 x Vbias, "));
		}

		// The RefIn- or RtdIn- pin is < 0.85 x Vbia
		if (status & (RTD_FAULT_REFIN_LOW_OPEN | RTD_FAULT_RTDIN_LOW_OPEN))
		{
			Serial.print(F("FORCE- open, "));
		}

		// The measured voltage at the RTD sense pins is too high or two low
		if (status & RTD_FAULT_VOLTAGE_OOR)
		{
			Serial.print(F("Voltage out of range fault, "));
		}
	}

	Serial.println();
}

float GetPlatinumRTD2(float R, float R0) {
	if (R == 0 || R0 == 0) return -256;
	float A = 3.9083E-3;
	float B = -5.775E-7;
	float T;

	R = R / R0;

	// Calculate temperature for positive resistance
	T = 0.0 - A;
	T += sqrt((A * A) - 4.0 * B * (1.0 - R));
	T /= (2.0 * B);

	// Check if the temperature is within the valid range
	if (T > -200 && T < 200) {
		return T;
									   
	}
	else {
						
		// Calculate temperature for negative resistance
		//T = 0.0 - A;
		//T -= sqrt((A * A) - 4.0 * B * (1.0 - R));
		//T /= (2.0 * B);
    // Check if the temperature is within the valid range
		//if (T >= -200 && T <= 200) {
			//return T;
		//}
		//else {
			// Return an error value if the temperature is out of range

		return -256; // You can choose any suitable error value
		   }														 
																 
  }


void GetDSReadings()
{
	sensors.requestTemperatures();
	DSTempC[0] = sensors.getTempCByIndex(0) + kalibracjaDS18_1;
	DSTempF[0] = sensors.getTempFByIndex(0) + kalibracjaDS18_1;
	Serial.print(F("DS[0]:Temp C:")); Serial.println(DSTempC[0]);
	Serial.print(F("DS[0]:Temp F:")); Serial.println(DSTempF[0]);

	DSTempC[1] = sensors.getTempCByIndex(1) + kalibracjaDS18_2;
	DSTempF[1] = sensors.getTempFByIndex(1) + kalibracjaDS18_2;
	Serial.print(F("DS[1]:Temp C:")); Serial.println(DSTempC[1]);
	Serial.print(F("DS[1]:Temp F:")); Serial.println(DSTempF[1]);

}

void GetBMEReadings()
{
	if (!hasBME) return;
	BMEtempReadC = bme280.readTemperature();
	BMEpressureReadHPA = bme280.readPressure() / 100.0F + 28.7;
	BMEtempReadF = (1.8 * BMEtempReadC + 32);
	BMEhumRead = bme280.readHumidity();
	BMEAltitudeM = bme280.readAltitude(SEALEVELPRESSURE_HPA);

	Serial.print(F("BME:Temp C:")); Serial.println(BMEtempReadC);
	Serial.print(F("BME:Pressure:")); Serial.println(BMEpressureReadHPA);
	Serial.print(F("BME:Temp F:")); Serial.println(BMEtempReadF);
	Serial.print(F("BME:Hum %:")); Serial.println(BMEhumRead);
	Serial.print(F("BME:Altitude:")); Serial.println(BMEAltitudeM);
}

void GetSHTReadings()
{
	if (!hasSHT) return;
	sensors_event_t sHum, sTemp;

	sht4.getEvent(&sHum, &sTemp);// populate temp and humidity objects with fresh data
	if (SHT_READ_COUNT > 1) {
		for (int j = 0; j < SHT_READ_COUNT; j++)
		{
			temp[j] = sTemp.temperature; // distance (mm) = time (total sound travelling distance, microseconds) * speed (mm/s)		
			humidity[j] = sHum.relative_humidity;
		}

		float totalTemp = 0;
		float totalHum = 0;
		for (int i = 0; i < SHT_READ_COUNT; i++)
		{
			totalTemp = totalTemp + temp[i];
			totalHum = totalHum + humidity[i];
		}

		SHTtempRead = (totalTemp / SHT_READ_COUNT) + kalibracjaSHT45;
    SHTtempReadF = (SHTtempRead * 9.0 / 5.0) + 32.0;
		SHThumRead = totalHum / SHT_READ_COUNT;
	}
	else
	{
		SHTtempRead = sTemp.temperature + kalibracjaSHT45;
    SHTtempReadF = (SHTtempRead * 9.0 / 5.0) + 32.0;
		SHThumRead = sHum.relative_humidity;
	}
	Serial.print(F("SHT:Temp:")); Serial.println(SHTtempRead);
	Serial.print(F("SHT:Hum:")); Serial.println(SHThumRead);

}

/// <summary>
/// Restarts or shut down GSM module RF
/// </summary>
/// <param name="stayOff">if true modem will not switch RF back on.</param>
bool ToggleGSModule(bool stayOff)
{
	if (AirplaneModeOn() && !stayOff)
	{
		if (AirplaneModeOff()) return true;
	}
	else
	{
		return true;
	}
}

bool AirplaneModeOn()
{
	uint8_t tryCnt = 1;

	while (!SIM800.sendCheckReply(F("AT+CFUN=0"), F("OK"), 1500))
	{
		Serial.print(F("Airplane mode ON Attempt:")); Serial.println(tryCnt);
		if (tryCnt == 10) {
			Serial.print(F("Airplane mode on failed 10 times"));
			return false;
		}

		delay(2000);
		tryCnt++;
	}
	Serial.println(F("Airplane mode ON"));
	return true;
}

bool AirplaneModeOff()
{
	uint8_t tryCnt = 0;
	while (!SIM800.sendCheckReply(F("AT+CFUN=1"), F("+CPIN: READY"), 3500))
	{
		Serial.print(F("Airplane mode OFF Attempt:")); Serial.println(tryCnt);

		if (tryCnt == 10) {
			Serial.println(F("Airplane mode off failed 5 times "));
			return false;
		};
		delay(1500);
		tryCnt++;

	}
	Serial.println(F("Airplane mode OFF waiting 2s for network "));
	delay(2000);
	return true;
}

bool SendData(char* creq)
{
	bool isSendSuccess = false;
	uint8_t rtryCount = 1;
	uint8_t rssi = 0;
	uint16_t statuscode = 0;
	while (!isSendSuccess)
	{
		Serial.print(F("Request:")); Serial.println(creq);
		Serial.print(F("Request Sent Attempt:")); Serial.println(rtryCount);

		if (SIM800.HTTP_GET_start(creq, &statuscode, (uint16_t*)&length));
		{
			isSendSuccess = CeheckHTTPStatus(&statuscode);
			if (!isSendSuccess) {
				Serial.println(F("Invalid Response... Retry in 3s"));
				SIM800.HTTP_GET_end();
					 
				if (rtryCount == 6)
				{
					Serial.print(F("Invalid Response after 6 times...Giving up..."));
					return false;
				}
				delay(3000);
			}
			else
			{
				break;
				 
			}
			   
		}
		
		
		

		rtryCount++;
	}
	Serial.println(F("Request finished."));
	return isSendSuccess;
}

bool ValidateData(char* var1, float var2, float var3, float simVol)
{
	return true;
}

bool CeheckHTTPStatus(uint16_t* statuscode)
{
	Serial.print(F("Http status:"));
	Serial.println(*statuscode);
	if (*statuscode == 200) {
		SIM800.HTTP_GET_end();
		delay(100);
	}
	return *statuscode == 200;
}
