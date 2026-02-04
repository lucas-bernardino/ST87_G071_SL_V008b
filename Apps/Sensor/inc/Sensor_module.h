
typedef struct
{
  float Ax;
  float Ay;
  float Az;
  float temp;
 }
sensor_data_t;

void lis2du12_tap_irq_handler(void);
void Sensor_Controller (void);
void Sensor_Init (void);