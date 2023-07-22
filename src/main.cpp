#include <mcp_can.h>
#include <SPI.h>

// MCP2515 Pins
#define CAN_CS_PIN 10u
#define CAN_INT_PIN 2u

// PWM Output
#define PWM_PIN 3u

// Amount of Sensors
#define NUM_SENSORS 2u

// CAN-ID und Data Length
#define CAN_ID_0 0x180u
#define CAN_ID_1 (CAN_ID_0 + 1u)
#define CAN_DATA_LEN 8u
#define CAN_MSG_TIMEOUT_MS 250u

// PWM characteristic curve
#define M -5.0f    // Slope
#define B 99.5f    // Offset

// Stoichiometric Air/Fuel Ratio
#define STOICH_AFR 14.7f

// Enable debug outputs on console
#define ENABLE_DEBUG_OUTPUT

typedef struct uego_t
{
    float valueLambda;
    float valueAfr;
    unsigned long lastRx;
} uego;

MCP_CAN CAN0(CAN_CS_PIN);
uego sensor[NUM_SENSORS];

void checkCanReception(void);
void updateOutput(void);

void setup()
{
    Serial.begin(115200u);

    // Init CAN
    if ( CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK )
    {
        Serial.print("MCP2515 Init Okay!!\r\n");
    }
    else
    {
        Serial.print("MCP2515 Init Failed!!\r\n");
        while(1);
    }

    // Setting pin 2 for /INT input
    pinMode(CAN_INT_PIN, INPUT);

    // Set PWM frequency for D3 & D11
    TCCR2B = ( TCCR2B & B11111000 ) | B00000001;  // set timer 2 divisor to 1 for PWM frequency of 31372.55 Hz

    CAN0.init_Mask(0, 0, 0x07F00000);                                // Init first mask...
    CAN0.init_Filt(0, 0, 0x01800000);                                // Init first filter...
    // CAN0.init_Filt(1, 1, 0x00000180);                                // Init second filter...

    CAN0.init_Mask(1, 0, 0x07F00000);                                // Init second mask...
    CAN0.init_Filt(2, 0, 0x01800000);                                // Init third filter...
    // CAN0.init_Filt(3, 1, 0x00000000);                                // Init fourth filter...
    // CAN0.init_Filt(4, 1, 0x00000000);                                // Init fifth filter...
    // CAN0.init_Filt(5, 1, 0x00000000);                                // Init sixth filter...

    CAN0.setMode(MCP_NORMAL);                                // Change to normal mode to allow messages to be transmitted
}

void loop()
{
    checkCanReception();
    updateOutput();
}

void checkCanReception(void)
{
    unsigned long rxId;
    uint8_t len = 0u;
    uint8_t rxBuf[8u];

    // If pin 2 is low, read receive buffer
    if(!digitalRead(CAN_INT_PIN))
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

        if ( ( CAN_ID_0 == rxId ) &&
             ( CAN_DATA_LEN == len ) )
        {
            // Interpret data e.g. Lambda 0.85 <-> 8500u <-> 0x 21 34
            uint16_t lambdaInt = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
            sensor[0u].valueLambda = 0.0001f * lambdaInt;
            sensor[0u].valueAfr = sensor[0u].valueLambda * STOICH_AFR;
            sensor[0u].lastRx = millis();
        }
        else if ( ( CAN_ID_1 == rxId ) &&
                  ( CAN_DATA_LEN == len ) )
        {
            // Interpret data e.g. 0.85 -> 8500 -> 0x 21 34
            uint16_t lambdaInt = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
            sensor[1u].valueLambda = 0.0001f * lambdaInt;
            sensor[1u].valueAfr = sensor[1u].valueLambda * STOICH_AFR;
            sensor[1u].lastRx = millis();
        }

        // Serial.print("ID: ");
        // Serial.print(rxId, HEX);
        // Serial.print(" Data: ");
        // for(int i = 0; i<len; i++)                     // Print each byte of the data
        // {
        //     if(rxBuf[i] < 0x10)                                // If data byte is less than 0x10, add a leading zero
        //     {
        //         Serial.print("0");
        //     }
        //     Serial.print(rxBuf[i], HEX);
        //     Serial.print(" ");
        // }
        // Serial.println();
    }
}

void updateOutput(void)
{
    float outputAfr = 0.0f;
#ifdef ENABLE_DEBUG_OUTPUT
    static boolean debugLastValidState[NUM_SENSORS] = { false }; // invalid
    static float   debugLastAfr[NUM_SENSORS] = { 0.0f };
#endif

    // Check all sensors for invalid value or timeout
    for (uint8_t i=0u; i<NUM_SENSORS; i++)
     {
        if ((sensor[i].valueAfr != 0.0f) && (sensor[i].lastRx > (millis() - CAN_MSG_TIMEOUT_MS)))
        {
            // Sensor is valid, take highest (leanest) value for output
            outputAfr = max(outputAfr, sensor[i].valueAfr); // Take highest (leanest) value for output

#ifdef ENABLE_DEBUG_OUTPUT
            // Output valid state or Afr on change
            if ((false == debugLastValidState[i]) ||
                (sensor[i].valueAfr != debugLastAfr[i]))
            {
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.print(" valid. Afr=");
                Serial.println(sensor[i].valueAfr);
                debugLastValidState[i] = true;
                debugLastAfr[i] = sensor[i].valueAfr;
            }
        }
        else
        {
            // Output valid state on change
            if (true == debugLastValidState[i])
            {
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" invalid");
                debugLastValidState[i] = false;
            }
        }
#else
        }
#endif
    }


    if (outputAfr != 0.0f)
    {
        // At least one sensor is valid, calculate output according to characteristic curve
        uint8_t pwmValue = (uint8_t)(M * outputAfr + B);
        /*
          +----+------+
          | y  |  x   |
          +----+------+
          | 49 | 10.1 |
          | 40 | 11.9 |
          | 30 | 13.9 |
          | 20 | 15.9 |
          | 10 | 17.9 |
          |  0 | 20.0 |
          +----+------+

          Test
          | Lambda | AFR calc | AFR display |
          | 0.75   | 11.03    | 11.1        |
          | 0.80   | 11.76    | 11.9        |
          | 0.85   | 12.50    | 12.5        |
          | 0.90   | 13.23    | 13.3        |
          | 0.95   | 13.97    | 14.1        |
          | 1.00   | 14.70    | 14.7        |
        */
        analogWrite(PWM_PIN, pwmValue);
    }
    else
    {
        // Error, no valid sensor
        analogWrite(PWM_PIN, 0u);
    }

}