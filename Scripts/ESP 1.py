#include <ros.h>
    #include <std_msgs/Int32MultiArray.h>
    #include <std_msgs/Float32MultiArray.h>
    #include <Arduino.h>
    #define ROS_NODE_BUFFER_IN 2048
    #define ROS_NODE_BUFFER_OUT 2048
    ros::NodeHandle_<ArduinoHardware, 25, 25, ROS_NODE_BUFFER_IN, ROS_NODE_BUFFER_OUT> nh;
    /* ---------- Flame Sensor Setup ---------- */
    std_msgs::Int32MultiArray flame_msg;
    ros::Publisher flame_pub("flame_sensor", &flame_msg);
    const int flamePins[5] = {39, 15, 4, 16, 17};
    TaskHandle_t flameSensorTaskHandle;

    void flameSensorTask(void *pvParameters) {
        while (true) {
            for (int i = 0; i < 5; ++i) {
                flame_msg.data[i] = digitalRead(flamePins[i]);
            }
            flame_pub.publish(&flame_msg);
            nh.spinOnce();
            vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 10ms
        }
    }

    /* ---------- Ultrasonic Sensor Setup ---------- */
    #define FRONT_TRIG  0
    #define FRONT_ECHO  5
    #define LEFT_TRIG   2
    #define LEFT_ECHO   18
    #define RIGHT_TRIG  23
    #define RIGHT_ECHO  19

    const float SOUND_SPEED_CM = 0.0343F;
    const float MIN_CM = 2.0F, MAX_CM = 50.0F;
    const uint8_t MED_SAMPLES = 5;
    const uint32_t SHOT_DELAY_US = 8000;
    const float CHANGE_THRESH = 1.0F;

    typedef struct {
        uint8_t trigPin, echoPin;
        float buf[MED_SAMPLES];
        uint8_t idx;
        bool filled;
        float stable;
    } Sensor;

    static Sensor sensors[] = {
        {FRONT_TRIG, FRONT_ECHO, {0}, 0, false, -1.0F},
        {LEFT_TRIG, LEFT_ECHO, {0}, 0, false, -1.0F},
        {RIGHT_TRIG, RIGHT_ECHO, {0}, 0, false, -1.0F}
    };
    const uint8_t N_SENS = sizeof(sensors) / sizeof(sensors[0]);

    float shot(uint8_t trigPin, uint8_t echoPin) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        unsigned long t = pulseIn(echoPin, HIGH, 25000UL);
        if (t == 0) return -1.0F;
        float d = (t * SOUND_SPEED_CM) / 2.0F;
        return (d < MIN_CM || d > MAX_CM) ? -1.0F : d;
    }

    float median5(const float *b) {
        float tmp[MED_SAMPLES];
        memcpy(tmp, b, sizeof(tmp));
        for (uint8_t i = 1; i < MED_SAMPLES; ++i) {
            float key = tmp[i];
            int8_t j = i - 1;
            while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; --j; }
            tmp[j + 1] = key;
        }
        return tmp[MED_SAMPLES / 2];
    }

    QueueHandle_t distQ;
    std_msgs::Float32MultiArray distances_msg;
    ros::Publisher pub("esp1", &distances_msg);

    void ultraTask(void *unused) {
        const TickType_t shotTicks = pdMS_TO_TICKS(SHOT_DELAY_US / 1000);

        for (;;) {
            for (auto &s : sensors)
                s.buf[s.idx] = shot(s.trigPin, s.echoPin);

            for (auto &s : sensors) {
                s.idx = (s.idx + 1) % MED_SAMPLES;
                if (s.idx == 0) s.filled = true;
            }

            if (sensors[0].filled) {
                float out[N_SENS];
                for (uint8_t i = 0; i < N_SENS; ++i) {
                    float cand = median5(sensors[i].buf);
                    if (cand == -1.0F) {
                        sensors[i].stable = -1.0F;
                    } else if (fabsf(cand - sensors[i].stable) > CHANGE_THRESH || sensors[i].stable == -1.0F) {
                        sensors[i].stable = cand;
                    }
                    out[i] = sensors[i].stable;
                }
                xQueueOverwrite(distQ, &out);
            }
            vTaskDelay(shotTicks);
        }
    }

    void controlTask(void *unused) {
        float latest[N_SENS];

        for (;;) {
            if (xQueueReceive(distQ, &latest, portMAX_DELAY) == pdPASS) {
                float front = latest[0];
                float left = latest[1];
                float right = latest[2];

                Serial.println(F("\n=== 🚗 NAVIGATION DECISION ==="));
                Serial.print(F("Front: ")); front == -1 ? Serial.println("No reading") : Serial.println(String(front, 1) + " cm");
                Serial.print(F("Left : ")); left == -1 ? Serial.println("No reading") : Serial.println(String(left, 1) + " cm");
                Serial.print(F("Right: ")); right == -1 ? Serial.println("No reading") : Serial.println(String(right, 1) + " cm");

                distances_msg.data_length = N_SENS;
                distances_msg.data = latest;
                pub.publish(&distances_msg);

                    nh.spinOnce();
                    vTaskDelay(pdMS_TO_TICKS(200));
                
            }
        }
    }

    /* ---------- Setup and Loop ---------- */
    
    void setup() {
        Serial.begin(115200);

        // Flame sensor setup
        nh.initNode();
        nh.advertise(flame_pub);
        flame_msg.data_length = 5;
        flame_msg.data = new int[5];
        for (int i = 0; i < 5; ++i) {
            pinMode(flamePins[i], INPUT);
        }
        

        // Ultrasonic sensor setup
        for (auto &s : sensors) {
            pinMode(s.trigPin, OUTPUT);
            pinMode(s.echoPin, INPUT);
            digitalWrite(s.trigPin, LOW);
        }
        distQ = xQueueCreate(1, sizeof(float) * N_SENS);
        nh.advertise(pub);
        xTaskCreatePinnedToCore(flameSensorTask, "FlameSensorTask", 2048, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(ultraTask, "ultraTask", 2048, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(controlTask, "controlTask", 2048, nullptr, 1, nullptr, 1);
    }

    void loop() {
        // Empty since FreeRTOS tasks handle the functionality
    }