/*
 *  HiDTracker_MPU6050 v0.1
 *  
 *  (c) 2019  HickDead, Vis3r
 *  
 */
 
#include <USBComposite.h>

#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x03
#include "MPU6050_6Axis_MotionApps20.h"
#define MPU_AD0 0

// The answer to the ultimate question...
#define REPORT_ID 42

// I2C speed in KHz, 100 - 400
#define WIRE_CLOCK 100  
//#define WIRE_CLOCK 400

// Sensor pins 
#define INT_PIN PA1
#define SCL_PIN PB6     // default
#define SDA_PIN PB7     // default
#define VCC_PIN PB5

// Led stuff
#define LED_PIN PC13
#define LED_ON LOW
#define LED_OFF HIGH


typedef struct 
{
    float x;
    float y;
    float z;
    float w;
} quat_t;

typedef struct 
{
    const uint8_t  ID = REPORT_ID;
    quat_t         Q;
} __packed HIDReport_t;


const uint8_t reportDescriptor[] = 
{
  0x05, 0x03,               // Usage Page (VR Ctrls)
  0x09, 0x06,               // Usage (Head Mounted Display)
  0xA1, 0x01,               // Collection (Application)
  0x85, REPORT_ID,          //   Report ID (REPORT_ID)
  0x09, 0x06,               //   Usage (Head Mounted Display)
  0xA1, 0x00,               //   Collection (Physical)
  0x05, 0x01,               //     Usage Page (Generic Desktop Ctrls)
  0x09, 0x49,               //     Usage (0x49) Qx
  0x09, 0x4A,               //     Usage (0x4A) Qy
  0x09, 0x4B,               //     Usage (0x4B) Qz
  0x09, 0x4C,               //     Usage (0x4C) Qw
  0x75, 0x20,               //     Report Size (32)
  0x95, 0x04,               //     Report Count (4)
  0x81, 0x82,               //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,                     //   End Collection
  0xC0,                     // End Collection
};


USBHID         HID;
HIDReport_t    report, old_report;
HIDReporter    reporter( HID, (uint8_t*)&report, sizeof(report), REPORT_ID);
volatile bool  sensorInterrupt = false;     // has sensor interrupt pin triggered?

MPU6050        sensor( 0x68 + MPU_AD0);     // our sensor du jour
uint8_t        packetSize = 0;              // expected sensor packet size


/*
 *  Handle interrupts from the motion sensor
 */
void sensorReady()
{
    sensorInterrupt = true;
}


/* 
 *  Initialize the i2c bus
 */
void setup_wire()
{

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_SOFTWIRE
  Wire.begin();
  Wire.setClock( WIRE_CLOCK*1000UL);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup( WIRE_CLOCK, true);
#endif // I2CDEV_ARDUINO_WIRE || I2CDEV_BUILTIN_FASTWIRE

}

/*
 *  Initialize sensor, returns true upon success
 */
bool setup_sensor()
{
  bool   rv = false;


  // initialize device
  sensor.initialize();

# ifdef CALIBRATE
  sensor.setXAccelOffset( ACCEL_OFFSET_X);
  sensor.setYAccelOffset( ACCEL_OFFSET_Y);
  sensor.setZAccelOffset( ACCEL_OFFSET_Z);
  sensor.setXGyroOffset( GYRO_OFFSET_X);
  sensor.setYGyroOffset( GYRO_OFFSET_Y);
  sensor.setZGyroOffset( GYRO_OFFSET_Z);
# endif // CALIBRATE

  // load and configure the DMP
  if( ! sensor.dmpInitialize() )
  {
    // turn on the DMP, now that it's ready
    sensor.setDMPEnabled( true);

    // get expected DMP packet size for later comparison
    packetSize= sensor.dmpGetFIFOPacketSize();

    rv= true;
  }
  
  return rv;
}



void setup() 
{

  pinMode( LED_PIN, OUTPUT);
  digitalWrite( LED_PIN, LED_OFF);
# ifdef VCC_PIN
  pinMode( VCC_PIN, OUTPUT);
  digitalWrite( VCC_PIN, HIGH);    // turn on sensor
# endif // VCC_PIN

  USBComposite.setManufacturerString( "Vis3r");
  USBComposite.setProductString( "Vis3r STM32 HMD MPU6050");
  USBComposite.setSerialString( "HMD 0.1");
  HID.begin( reportDescriptor, sizeof(reportDescriptor));

  // First blink; USB ready
  digitalWrite( LED_PIN, LED_ON);
  delay( 500);
  digitalWrite( LED_PIN, LED_OFF);
  delay( 500);

  setup_wire();

  // Initialize sensor
  while( ! setup_sensor() )
  {
    // short blinks means retrying sensor initialization, shouldn't normally happen

#   ifdef VCC_PIN
    digitalWrite( VCC_PIN, LOW);    // turn off sensor
#   endif // VCC_PIN
    digitalWrite( LED_PIN, LED_ON);
    delay( 100);

#   ifdef VCC_PIN
    digitalWrite( VCC_PIN, HIGH);    // turn on sensor
#   endif // VCC_PIN
    digitalWrite( LED_PIN, LED_OFF);
    delay( 100);
  }

# ifdef INT_PIN    // sensor INT pin hooked up?
  // configure interrupt pin as input, prolly default but meh
  pinMode( INT_PIN, INPUT);
  // activate pull-up on interrupt pin
  digitalWrite( INT_PIN, HIGH);
  // enable Arduino interrupt detection
  attachInterrupt( digitalPinToInterrupt(INT_PIN), sensorReady, RISING);
# endif // INT_PIN

  // Second blink means we have passed setup
  // also to slow things down
  digitalWrite( LED_PIN, LED_ON);
  delay( 500);
  digitalWrite( LED_PIN, LED_OFF);
  delay( 500);

}



/*
 *  Spit out quaternion data to the PC
 */
void output_vis3r(Quaternion q)
{

    report.Q.w=q.w;
    report.Q.x=q.x;
    report.Q.y=q.y;
    report.Q.z=q.z;

    if( memcmp(&old_report,&report,sizeof(HIDReport_t)) )
    {
      memcpy( &old_report, &report, sizeof(HIDReport_t));
      reporter.sendReport();
    }

}



void loop() 
{
  uint8_t            sensorIntStatus;   // holds actual interrupt status byte from sensor
  static uint16_t    fifoCount;         // count of all bytes currently in FIFO
  static uint8_t     fifoBuffer[64];    // FIFO storage buffer
  Quaternion         q;                 // [w, x, y, z]         quaternion container


  // wait for MPU interrupt or extra packet(s) available
  if( ! sensorInterrupt && fifoCount < packetSize )
    return;

  // get current FIFO count
  fifoCount= sensor.getFIFOCount();

  if( fifoCount < packetSize )
    return;

  // reset interrupt flag
  sensorInterrupt= false;

  // get INT_STATUS byte
  sensorIntStatus= sensor.getIntStatus();

  // check for overflow (this should never happen unless our code is too inefficient)
  
  if( (sensorIntStatus & 0x10) || fifoCount == 1024 )
  {
    digitalWrite( LED_PIN, LED_ON);
    
    // reset so we can continue cleanly
    sensor.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if( sensorIntStatus & 0x02 )
  {
    digitalWrite( LED_PIN, LED_OFF);

    // process all packets currently in the FIFO
    while( fifoCount >= packetSize )
    {
  
      // read a packet from FIFO
      sensor.getFIFOBytes( fifoBuffer, packetSize);
  
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

    }

    sensor.dmpGetQuaternion( &q, fifoBuffer);


    output_vis3r( q);


  }

  // done or are there new packets in FIFO?
//  if( fifoCount < packetSize )
//    fifoCount= sensor.getFIFOCount();
    


}
