/*********************************************************************
 BleuIO Example part 3.
 Expanding on the previous example of using a Adafruit Feather RP2040 
 Board with a BME680 sensor and a OTP3002 sensor with a BleuIO Dongle. 
 The sensor values is moved from the advertising data to a custom 
 service and is only accessible during a secure connection.

 Copyright (c) 2024 Smart Sensor Devices AB
*********************************************************************/

#include <Wire.h>
#include <SPI.h>
/* Adafruid BME680 */
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme; // I2C

#define SEALEVELPRESSURE_HPA (1013.25)

/* ClosedCube_OPT3002 */
#include "ClosedCube_OPT3002.h"
ClosedCube_OPT3002 opt3002;

// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"
// CDC Host object
Adafruit_USBH_CDC SerialHost;

#define OPT3002_ADDRESS 0x44
#define BME680_ADDRESS 0x76


#define PASSKEY_CMD_FORMAT(x) "AT+SETPASSKEY=" x "\r"

/* Requires 6 digits */
#define SECURE_CONN_PASSKEY "232425"

/* Starting commands for the BleuIO. Run in tuh_cdc_mount_cb() as soon as the BleuIO is mounted.
  Commands will:
  Turn of echo, 
  Remove bonded devices,
  Setup a Custom service with 5 characteristics with read and notify properites, 
  Start Custom Service,
  Set 'BleuIO Arduino Example' as Complete Local Name in the Advertising Response data,
  Set the Input/Output capability of the BleuIO to "Display Only",
  Set the Security Level to 4 (Authenticated LE Secure Connections pairing with encryption),
  Set the Passkey required for pairing and bonding,
  Lastly it will start Advertising.
*/
#define TURN_OFF_ECHO_CMD "ATE0\r"
#define REMOVE_BONDED_CMD "AT+GAPUNPAIR\r"
#define SETUP_SERVICE "AT+CUSTOMSERVICE=0=UUID=ee6ec068-7447-4045-9fd0-593f3ba3c2ee\r"
#define SETUP_CHAR1  "AT+CUSTOMSERVICE=1=UUID=018f55d9-d747-4c4e-a87b-e9b074ffd2b1\rAT+CUSTOMSERVICE=1=PROP=RN\rAT+CUSTOMSERVICE=1=PERM=R\rAT+CUSTOMSERVICE=1=LEN=100\rAT+CUSTOMSERVICE=1=VALUEB=00\rAT+CUSTOMSERVICE=1=DLEN=100\rAT+CUSTOMSERVICE=1=DVALUE=Lux\rAT+CUSTOMSERVICE=1=DPERM=R\r"
#define SETUP_CHAR2  "AT+CUSTOMSERVICE=2=UUID=018f55d9-d747-4c4e-a87b-e9b074ffd2b2\rAT+CUSTOMSERVICE=2=PROP=RN\rAT+CUSTOMSERVICE=2=PERM=R\rAT+CUSTOMSERVICE=2=LEN=100\rAT+CUSTOMSERVICE=2=VALUEB=00\rAT+CUSTOMSERVICE=2=DLEN=100\rAT+CUSTOMSERVICE=2=DVALUE=Pressure\rAT+CUSTOMSERVICE=2=DPERM=R\r"
#define SETUP_CHAR3  "AT+CUSTOMSERVICE=3=UUID=018f55d9-d747-4c4e-a87b-e9b074ffd2b3\rAT+CUSTOMSERVICE=3=PROP=RN\rAT+CUSTOMSERVICE=3=PERM=R\rAT+CUSTOMSERVICE=3=LEN=100\rAT+CUSTOMSERVICE=3=VALUEB=00\rAT+CUSTOMSERVICE=3=DLEN=100\rAT+CUSTOMSERVICE=3=DVALUE=Temperature\rAT+CUSTOMSERVICE=3=DPERM=R\r"
#define SETUP_CHAR4  "AT+CUSTOMSERVICE=4=UUID=018f55d9-d747-4c4e-a87b-e9b074ffd2b4\rAT+CUSTOMSERVICE=4=PROP=RN\rAT+CUSTOMSERVICE=4=PERM=R\rAT+CUSTOMSERVICE=4=LEN=100\rAT+CUSTOMSERVICE=4=VALUEB=00\rAT+CUSTOMSERVICE=4=DLEN=100\rAT+CUSTOMSERVICE=4=DVALUE=Humidity\rAT+CUSTOMSERVICE=4=DPERM=R\r"
#define SETUP_CHAR5  "AT+CUSTOMSERVICE=5=UUID=018f55d9-d747-4c4e-a87b-e9b074ffd2b5\rAT+CUSTOMSERVICE=5=PROP=RN\rAT+CUSTOMSERVICE=5=PERM=R\rAT+CUSTOMSERVICE=5=LEN=100\rAT+CUSTOMSERVICE=5=VALUEB=00\rAT+CUSTOMSERVICE=5=DLEN=100\rAT+CUSTOMSERVICE=5=DVALUE=Gas Resistance\rAT+CUSTOMSERVICE=5=DPERM=R\r"
#define START_CUSTOM_SERVICE_CMD "AT+CUSTOMSERVICESTART\r"
#define SET_ADV_LOCAL_NAME_CMD "AT+ADVRESP=17:09:42:6C:65:75:49:4F:20:41:72:64:75:69:6E:6F:20:45:78:61:6D:70:6C:65\r"
#define SET_IOCAP_CMD  "AT+GAPIOCAP=0\r"
#define SET_SECLVL_CMD  "AT+SECLVL=4\r"
#define SET_PASSKEY_CMD PASSKEY_CMD_FORMAT(SECURE_CONN_PASSKEY)
#define START_ADV_CMD "AT+ADVSTART\r"

/* Command to check the security level, runs in every  READ_UPDATE_FREQUENCY loop*/
#define GET_SECLVL_CMD "AT+SECLVL\r"

/* Macro to format command to set all 5 characteristics values in one line*/
#define SET_CHAR_VALUE_CMD "AT+CUSTOMSERVICE=1=VALUEB=%02X%02X\rAT+CUSTOMSERVICE=2=VALUEB=%02X%02X\rAT+CUSTOMSERVICE=3=VALUEB=%02X%02X\rAT+CUSTOMSERVICE=4=VALUEB=%02X%02X\rAT+CUSTOMSERVICE=5=VALUEB=%02X%02X\r"

// How often we read the sensors and update the charactaristics (in seconds) 
#define READ_UPDATE_FREQUENCY   5

/* Global variables */
int loop_cnt;
char dongle_cmd[240];
int dongle_cmd_len;
uint32_t char_val1 = 0;
uint32_t char_val2 = 0;
uint32_t char_val3 = 0;
uint32_t char_val4 = 0;
uint32_t char_val5 = 0;
bool init_complete;
bool connected;
bool secure_conn;

void forward_serial(void) {
  uint8_t buf[256];

  // SerialHost -> Serial
  if (SerialHost.connected() && SerialHost.available()) {
    size_t count = SerialHost.read(buf, sizeof(buf));
    /* Uncomment to print BleuIO response to serial*/
    // Serial.println("BleuIO response:"); 
    // Serial.write(buf, count);
    // Serial.println("----"); 
    Serial.flush();
    /* Sets flags if dongle is connected or not, and if it's secure connection or not */
    if(strstr((char *)buf, "\r\nCONNECTED.") != NULL)
    {
        connected = true;
    }
    if(strstr((char *)buf, "\r\nDISCONNECTED.") != NULL)
    {
        connected = false;
        secure_conn = false;
    }
    if(connected && strstr((char *)buf, "\r\nSECURITY LEVEL = GAP_SEC_LEVEL_4") != NULL)
    {
        secure_conn = true;
    }  
    if(connected && strstr((char *)buf, "\r\nSECURITY LEVEL = GAP_SEC_LEVEL_1") != NULL)
    {
        secure_conn = false;
    }   

  }
}

void setup() {
  Serial.begin(9600);
  loop_cnt = 0;
  init_complete = false;
  secure_conn = false;
  connected = false;
  while (!Serial);
  Serial.println(F("BME680 & OPT3002 test v1.1b"));

  // init host stack on controller (rhport) 1
  USBHost.begin(1);

  // Initialize SerialHost
  SerialHost.begin(9600);


  if (!bme.begin(BME680_ADDRESS)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Setup OPT3002
  opt3002.begin(OPT3002_ADDRESS);
	Serial.print("OPT3002 Manufacturer ID:");
	Serial.print(opt3002.readManufacturerID());
	Serial.print(" Device ID:");
	Serial.println(opt3002.readDeviceID());

	configureSensor();
	printResult("High-Limit", opt3002.readHighLimit());
	printResult("Low-Limit", opt3002.readLowLimit());
	Serial.println("----");  

  // Delay so we have a chance to read the sensor setup prints
  delay(5000);
}


void loop() {
  // Forward the output from the dongle to the serial
  forward_serial();

  if(loop_cnt >= (READ_UPDATE_FREQUENCY * 1000))
  {
      Serial.println("Reading sensor values!");

      /* BME680 */
      if (! bme.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }

      /* Sending generated command to BleuIO */
      if (SerialHost && SerialHost.connected() && init_complete) 
      {
        SerialHost.write((uint8_t *)GET_SECLVL_CMD, sizeof(GET_SECLVL_CMD));
        SerialHost.flush();
      }

      Serial.print("Temperature = ");
      Serial.print(bme.temperature);
      Serial.println(" *C");

      Serial.print("Pressure = ");
      Serial.print(bme.pressure / 100.0);
      Serial.println(" hPa");

      Serial.print("Humidity = ");
      Serial.print(bme.humidity);
      Serial.println(" %");

      Serial.print("Gas = ");
      Serial.print(bme.gas_resistance);
      Serial.println(" Ohms");
      // Serial.print(bme.gas_resistance / 1000.0);
      // Serial.println(" KOhms");

      Serial.print("Approx. Altitude = ");
      Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");
      /* End BME680 */

      /* OPT3002 */
      OPT3002 result = opt3002.readResult();

      printResult("OPT3002", result);
      Serial.println();
      Serial.flush();
      /* End OPT3002 */

      /* Storing the sensor values to fit in a uint16_t format for the characteristics, no decimals*/
      char_val1 = (uint32_t) (result.lux/1000); /* uW/cm2 */
      char_val2 = (uint32_t) (bme.pressure/100); /* hPa (removing the decimals)*/
      char_val3 = (uint32_t) (bme.temperature); /* Celcius */
      char_val4 = (uint32_t) (bme.humidity); /* %Rh*/
      char_val5 = (uint32_t) bme.gas_resistance / 1000; /* KOhms */


      /* Print out the state of the dongle, if it's connected and if it's a secure connection. */
      Serial.print("Connected: ");
      Serial.println(connected);
      Serial.print("secure_conn: ");
      Serial.println(secure_conn);


      /* Here we update the custom service characteritstics with the proper sensor values, otherwise we set them to 0 */
      if(connected && secure_conn)
      {
          dongle_cmd_len = snprintf(dongle_cmd, 240, SET_CHAR_VALUE_CMD, 
            (uint8_t) (char_val1 >> 8), (uint8_t) (char_val1 & 0xFF), /*big endian*/
            (uint8_t) (char_val2 >> 8), (uint8_t) (char_val2 & 0xFF), /*big endian*/
            (uint8_t) (char_val3 >> 8), (uint8_t) (char_val3 & 0xFF), /*big endian*/
            (uint8_t) (char_val4 >> 8), (uint8_t) (char_val4 & 0xFF), /*big endian*/
            (uint8_t) (char_val5 >> 8), (uint8_t) (char_val5 & 0xFF) /*big endian*/          
          );

      } else 
      {
            dongle_cmd_len = snprintf(dongle_cmd, 240, SET_CHAR_VALUE_CMD, 
            0,0,
            0,0,
            0,0,
            0,0,
            0,0          
          );
      }

      /* Sending generated command to BleuIO */
      if (SerialHost && SerialHost.connected()) 
      {
        SerialHost.write((uint8_t *)dongle_cmd, dongle_cmd_len);
        SerialHost.flush();
      }

      /* Print out the values set in the characeristics (when in secure connetion) to compare*/
      Serial.println();
      Serial.print("Out: ");
      Serial.printf("char_val1=0x%02X%02X(%i), char_val2=0x%02X%02X(%i), char_val3=0x%02X%02X(%i), char_val4=0x%02X%02X(%i), char_val5=0x%02X%02X(%i)", 
            (uint8_t) (char_val1 >> 8), (uint8_t) (char_val1 & 0xFF), char_val1, /*big endian*/
            (uint8_t) (char_val2 >> 8), (uint8_t) (char_val2 & 0xFF), char_val2, /*big endian*/
            (uint8_t) (char_val3 >> 8), (uint8_t) (char_val3 & 0xFF), char_val3, /*big endian*/
            (uint8_t) (char_val4 >> 8), (uint8_t) (char_val4 & 0xFF), char_val4, /*big endian*/
            (uint8_t) (char_val5 >> 8), (uint8_t) (char_val5 & 0xFF), char_val5 /*big endian*/  
          );          
      Serial.println();          

      loop_cnt = 0;
  } // end of if(loop_cnt >= (READ_UPDATE_FREQUENCY * 1000))

  loop_cnt++;
  delay(1);
}

//------------- Core1 -------------//
void setup1() {
  // configure pio-usb: defined in usbh_helper.h
  rp2040_configure_pio_usb();

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);

  // Initialize SerialHost
  SerialHost.begin(9600);
}

void loop1() {
  USBHost.task();
}

/* ClosedCube_OPT3002 functions */
void printResult(String text, OPT3002 result) {
	if (result.error == NO_ERROR) {
		Serial.print(text);
		Serial.print(": ");
		Serial.print(result.lux);
		Serial.println(" nW/cm2");
	}
	else {
		printError(text, result.error);
	}
}

void printError(String text, OPT3002_ErrorCode error) {
	Serial.print(text);
	Serial.print(": [ERROR] Code #");
	Serial.println(error);
}

void configureSensor() {
	OPT3002_Config newConfig;

	newConfig.RangeNumber = 0b1100;
	newConfig.ConvertionTime = 0b0;
	newConfig.Latch = 0b1;
	newConfig.ModeOfConversionOperation = 0b11;

	OPT3002_ErrorCode errorConfig = opt3002.writeConfig(newConfig);
	if (errorConfig != NO_ERROR)
		printError("OPT3002 configuration", errorConfig);
	else {
		OPT3002_Config sensorConfig = opt3002.readConfig();
		Serial.println("OPT3002 Current Config:");
		Serial.println("------------------------------");

		Serial.print("Conversion ready (R):");
		Serial.println(sensorConfig.ConversionReady, HEX);

		Serial.print("Conversion time (R/W):");
		Serial.println(sensorConfig.ConvertionTime, HEX);

		Serial.print("Fault count field (R/W):");
		Serial.println(sensorConfig.FaultCount, HEX);

		Serial.print("Flag high field (R-only):");
		Serial.println(sensorConfig.FlagHigh, HEX);

		Serial.print("Flag low field (R-only):");
		Serial.println(sensorConfig.FlagLow, HEX);

		Serial.print("Latch field (R/W):");
		Serial.println(sensorConfig.Latch, HEX);

		Serial.print("Mask exponent field (R/W):");
		Serial.println(sensorConfig.MaskExponent, HEX);

		Serial.print("Mode of conversion operation (R/W):");
		Serial.println(sensorConfig.ModeOfConversionOperation, HEX);

		Serial.print("Polarity field (R/W):");
		Serial.println(sensorConfig.Polarity, HEX);

		Serial.print("Overflow flag (R-only):");
		Serial.println(sensorConfig.OverflowFlag, HEX);

		Serial.print("Range number (R/W):");
		Serial.println(sensorConfig.RangeNumber, HEX);

		Serial.println("------------------------------");
	}

}
/* end of ClosedCube_OPT3002 functions */

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
extern "C" {

// Invoked when a device with CDC interface is mounted
// idx is index of cdc interface in the internal pool.
void tuh_cdc_mount_cb(uint8_t idx) {
  // bind SerialHost object to this interface index
  SerialHost.mount(idx);
  Serial.print("SerialHost is connected to a new CDC device. Idx: ");
  Serial.println(idx);

  /* Send start commands to BleuIO when detecting that  */
  if (SerialHost && SerialHost.connected()) 
  {
    SerialHost.write((uint8_t *)TURN_OFF_ECHO_CMD, sizeof(TURN_OFF_ECHO_CMD));
    SerialHost.write((uint8_t *)REMOVE_BONDED_CMD, sizeof(REMOVE_BONDED_CMD));
    SerialHost.write((uint8_t *)SETUP_SERVICE, sizeof(SETUP_SERVICE));
    SerialHost.write((uint8_t *)SETUP_CHAR1, sizeof(SETUP_CHAR1));
    SerialHost.write((uint8_t *)SETUP_CHAR2, sizeof(SETUP_CHAR2));
    SerialHost.write((uint8_t *)SETUP_CHAR3, sizeof(SETUP_CHAR3));
    SerialHost.write((uint8_t *)SETUP_CHAR4, sizeof(SETUP_CHAR4));
    SerialHost.write((uint8_t *)SETUP_CHAR5, sizeof(SETUP_CHAR5));
    SerialHost.write((uint8_t *)START_CUSTOM_SERVICE_CMD, sizeof(START_CUSTOM_SERVICE_CMD));
    SerialHost.write((uint8_t *)SET_ADV_LOCAL_NAME_CMD, sizeof(SET_ADV_LOCAL_NAME_CMD));
    SerialHost.write((uint8_t *)SET_IOCAP_CMD, sizeof(SET_IOCAP_CMD));
    SerialHost.write((uint8_t *)SET_SECLVL_CMD, sizeof(SET_SECLVL_CMD));
    SerialHost.write((uint8_t *)SET_PASSKEY_CMD, sizeof(SET_PASSKEY_CMD));
    SerialHost.write((uint8_t *)START_ADV_CMD, sizeof(START_ADV_CMD));
    SerialHost.flush();
    init_complete = true;
  }
}

// Invoked when a device with CDC interface is unmounted
void tuh_cdc_umount_cb(uint8_t idx) {
  SerialHost.umount(idx);
  Serial.println("SerialHost is disconnected");
  init_complete = false;
  secure_conn = false;
  connected = false;
}

}
