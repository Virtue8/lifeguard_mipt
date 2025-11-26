#include "main.h"

bool is_pulsing ()
{
    struct DeviceData* data = (struct DeviceData*) malloc (sizeof(struct DeviceData));

    //add condition for 5-20 sec burst of measurements

    debug.begin(9600);
    debug.println("MAX30105 Basic Readings Example");

    // Initialize sensor
    if (particleSensor.begin() == false)
    {
        debug.println("MAX30105 was not found. Please check wiring/power. ");
        while (1);
    }

    particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive

    while (true)
    {
        /*debug.print(" R[");
        debug.print(particleSensor.getRed());
        debug.print("] IR[");
        debug.print(particleSensor.getIR());
        debug.print("] G[");
        debug.print(particleSensor.getGreen());
        debug.print("]");*/
        debug.print(particleSensor.getIR());
            Serial.println();
        delay(10);
    }

    return true;
}

bool is_moving ()
{
    struct DeviceData* data = (struct DeviceData*) malloc (sizeof(struct DeviceData));

    digitalWrite (LED_PIN, HIGH);
    data->mpu.setSleepEnabled (false);
    delay (5);

    int N = 200;

    struct axelXYZ
    {
        int ax;
        int ay;
        int az;
    };

    struct axelXYZ axel_xyz[N] = {0};

    int min_ax = 32767;
    int max_ax = -32768;
    int min_ay = 32767;
    int max_ay = -32768;
    int min_az = 32767;
    int max_az = -32768;

    for (int i = 0; i < N; i++)
    {
        int16_t ax, ay, az;
        //mpu.getAcceleration(&data[i].ax, &data[i].ay, &data[i].az);
        for (int j = 0; j <= 10; j++)
        {
            data->mpu.getAcceleration (&ax, &ay, &az);
            axel_xyz[i].ax += ax;
            axel_xyz[i].ay += ay;
            axel_xyz[i].az += az;
        }
        axel_xyz[i].ax /= 10;
        axel_xyz[i].ay /= 10;
        axel_xyz[i].az /= 10;
        // Serial.print(data[i].ax);
        // Serial.print(" ");
        // Serial.print(max_ax - min_ax);
        // Serial.print(" ");
        // Serial.print(max_ay - min_ay);
        // Serial.print(" ");
        // Serial.println(max_az - min_az);
        if (axel_xyz[i].ax > max_ax)
            max_ax = axel_xyz[i].ax;
        if (axel_xyz[i].ax < min_ax)
            min_ax = axel_xyz[i].ax;

        if (axel_xyz[i].ay > max_ay)
            max_ay = axel_xyz[i].ay;
        if (axel_xyz[i].ay < min_ay)
            min_ay = axel_xyz[i].ay;

        if (axel_xyz[i].az > max_az)
            max_az = axel_xyz[i].az;
        if (axel_xyz[i].az < min_az)
            min_az = axel_xyz[i].az;
        delay (50);
    }

    data->movement_analysis = false;
    data->mpu.setSleepEnabled (true);
    digitalWrite (LED_PIN, LOW);
    Serial.print (max_ax - min_ax);
    Serial.print (" ");
    Serial.print (max_ay - min_ay);
    Serial.print (" ");
    Serial.println ((max_ax - min_ax >= 4000) || (max_ay - min_ay >= 4000));
    return (max_ax - min_ax >= 4000) || (max_ay - min_ay >= 4000);// || (max_az - min_az >= 2500);
}

void emergency ()
{
    struct DeviceData* data = (struct DeviceData*) malloc (sizeof(struct DeviceData));

    send_tg_message ("NO MOVEMENT");
    while (!data->button_state)
        ;
    data->is_buzzing = false;
    data->movement_analysis = false;
    data->pulse_analysis = false;
}

void make_buzz ()
{
    struct DeviceData* data = (struct DeviceData*) malloc (sizeof(struct DeviceData));

    if (data->is_buzzing)
    {
        digitalWrite (BUZZER_PIN, !digitalRead (BUZZER_PIN));
        digitalWrite (VIBRO_PIN, !digitalRead (VIBRO_PIN));
    }
    else
    {
        digitalWrite (BUZZER_PIN, LOW);
        digitalWrite (VIBRO_PIN, LOW);
    }
}

void smart_delay (int time)
{
    int start = millis ();
    while (millis () - start < time)
        ;
}

void IRAM_ATTR movement_check ()
{
    struct DeviceData* data = (struct DeviceData*) malloc (sizeof(struct DeviceData));
    data->movement_analysis = true;
}

void battery_check ()
{
    struct DeviceData* data = (struct DeviceData*) malloc (sizeof(struct DeviceData));

    const int FULL_CHARGE_V = 4;
    const int LOW_CHARGE_V = 3;
    const int V_DIVIDER = 2;

    float V_sum = 0;

    for (int i = 0; i < 100000; i++)
        V_sum = (float) analogRead (BATTERY_PIN)/4096*3.3*V_DIVIDER;

    Serial.println (V_sum);
    if (V_sum < LOW_CHARGE_V)
        Serial.println ("LOW CHARGE");
}
