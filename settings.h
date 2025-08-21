/*
  esp32-ai-cam

    by James Zahary    jamzah.plc@gmail.com

    https://github.com/jameszah/ESP32-AI-CAM

    jameszah/ESP32-AI_CAM is licensed under the
    GNU General Public License v3.0

*/

#define IOT_CONFIG_WIFI_SSID            "..."
#define IOT_CONFIG_WIFI_PASSWORD        "..."

/*String containing Hostname, Device Id & Device Key in the format:                         */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"                */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessSignature=<device_sas_token>"    */
//static const char* connectionString = "[device connection string]";
static const char* connectionString = "HostName=myhubname.azure-devices.net;DeviceId=jzLogger;SharedAccessKey=............................................";

const char* host = "myazureregion.api.cognitive.microsoft.com"; 
const char* Ocp_Apim_Subscription_Key = "................................";
const int Port = 443;

#define ms_between_pictures 10000
#define number_of_pictures 2

// Wildlife Camera Configuration
#define WILDLIFE_CAMERA_ENABLED 1
#define WEATHER_FILTERING_ENABLED 1
#define SOLAR_POWER_MONITORING 1
#define PAN_TILT_SERVOS_ENABLED 1
#define LORA_COMMUNICATION_ENABLED 0  // Disabled by default, enable when LoRa module is available

// Camera positioning
#define DEFAULT_PAN_HOME 135    // Center position
#define DEFAULT_TILT_HOME 90    // Level position

// Power management
#define ENABLE_POWER_SAVE_MODE 1
#define SOLAR_PANEL_PRESENT 1
#define BATTERY_CAPACITY_MAH 2000

// Weather filtering sensitivity
#define WEATHER_FILTER_SENSITIVITY 0.8  // 0.5 = less sensitive, 1.0 = more sensitive
#define WIND_DETECTION_THRESHOLD 0.7
#define RAIN_DETECTION_THRESHOLD 0.6

// Camera operation modes
#define PERIODIC_SWEEP_ENABLED 1
#define SWEEP_INTERVAL_HOURS 1
#define IR_AUTO_NIGHT_MODE 1
