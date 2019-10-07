#include "DHT.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "MPU9250.h"


//----------------------------coneccion con el wifi-------------------------------------------
const char* ssid = "WiFi-UAO";
const char* password =  "";


//*****************************************************VARIABLES GOBALES*****************************************************************************************************************************************************************
// El cable de datos se conecta al pin 26 del Arduino
#define ONE_WIRE_BUS 25 // Definir una instancia de OneWire para comunicarse con cualquier dispositivo OneWire
#define DHTTYPE DHT11   // DHT 22  (AM2302), AM2321
OneWire oneWire(ONE_WIRE_BUS); // Pasar la referencia a OneWire del sensor de temperatura Dallas
DallasTemperature sensors(&oneWire);
const int GSR = 34;
int sensorValue = 0;
int gsr_average = 0;
byte dat[5];
//esta es el tamaño de datos que va enviar el json
const size_t capacidad = JSON_OBJECT_SIZE(2) + 21 * JSON_OBJECT_SIZE(3) + 2 * JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(10);
int Conteo_conexiones = 0; //los intentos de conexion a la red wifi
uint8_t DHTPin = 4;//pin del sensor de humedad y temperatura
float ambient_humidity, ambient_temperature = 0;
// Definir modelo y pin del DTH
DHT dht(DHTPin, DHTTYPE);
// Definir cliente NTP para obtener tiempo
WiFiUDP ntpUDP;//Network Time Protocol Protocolo de datagramas de usuario
NTPClient timeClient(ntpUDP);//Network Time Protocol Client
// Variables para guardar fecha y hora
String formato;//formato de datos
String fecha;//
String hora;//
MPU9250 IMU(Wire, 0x68);
int status;
//*****************************************************SET UP*****************************************************************************************************************************************************************
void setup()
{
  Serial.begin(115200);
  Serial.println("Comenzando");
  // Iniciar la biblioteca
  sensors.begin(); // Establece valor por defecto del chip 9 bit. Si tiene problemas pruebe aumentar
  buscar_wifi(); //Usamos el metodo buscar_wifi para buscar el wifi
  timeClient.begin(); timeClient.setTimeOffset(-18000); //Inicialice un NTPClient para obtener tiempo
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

}
//*****************************************************LOOP*****************************************************************************************************************************************************************
void loop()
{
  if (WiFi.status() == WL_CONNECTED) { //comprobamos que seguimos conectados ala red wifi


    // Leemos la temperatura o la humedad toma alrededor de 250 milisegundos
    ambient_humidity = dht.readHumidity(); ambient_temperature = dht.readTemperature();

    // put your main code here, to run repeatedly:
    // read the sensor
    IMU.readSensor();

    //aceleracion
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");

    //Giros
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print("\t");

    //campo magnetico
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);
    Serial.print("\t");

    //sensor de teperatura
    Serial.println(IMU.getTemperature_C(), 6);
    delay(20);
    //resibimos un string que es array de chars que tambien es un objeto json a traves del metodo devolver_json

    String info = devolver_json(
                    sensors.getTempCByIndex(0), //body temperature value
                    gsr_average, //body resistenace value
                    0, //body BPM value
                    0, //body sp02 value
                    ambient_temperature, //ambient temperature value
                    ambient_humidity, //ambient humidity value
                    IMU.getAccelX_mss(),  //device acceleration ax
                    IMU.getAccelY_mss(),  //device acceleration ay
                    IMU.getAccelZ_mss(),  //device acceleration az
                    IMU.getGyroX_rads(),  //device gyro gx
                    IMU.getGyroY_rads(),  //device gyro gy
                    IMU.getGyroZ_rads(),  //device gyro gz
                    IMU.getMagX_uT(), //device magnetometer mx
                    IMU.getMagY_uT(),  //device magnetometer my
                    IMU.getMagZ_uT(),  //device magnetometer mz
                    0,//device pressure value
                    0,//device altitude value
                    IMU.getTemperature_C());//device temperature value
    //le enviamos los datos al servidor
    enviar_dato_cliente(info);
  } else {
    Serial.println("Error in WiFi connection");
    Serial.println("Reeconectando:");
    buscar_wifi();
  }
  sensors.requestTemperatures();  // Enviar el comando para obtener la temperatura
  long sum = 0;
  for (int i = 0; i < 10; i++)    //Average the 10 measurements to remove the glitch
  {
    sensorValue = analogRead(GSR);
    sum += sensorValue;
    delay(5);
  }
  gsr_average = sum / 10;
  int form = ((4095 + 2 * gsr_average) * 10000) / (2043 - gsr_average);
  Serial.println(form);
}


//*****************************************************BUSCAR WIFI*****************************************************************************************************************************************************************
void buscar_wifi() {
  //mandamos el nombre de la red wifi y la contraseña
  WiFi.begin(ssid, password);
  //Cuenta hasta 50 si no se puede conectar lo cancela
  while (WiFi.status() != WL_CONNECTED and Conteo_conexiones < 50  ) {
    Conteo_conexiones++;
    delay(500);
    Serial.print("#");
  }
  //cuando se conecte me dira la aprobacion y la ip
  if (WiFi.status() == WL_CONNECTED) {
    Conteo_conexiones == 0;
    Serial.println("WiFi conectado");
    Serial.println(WiFi.localIP());
  }
}
//*****************************************************JSON*****************************************************************************************************************************************************************
String devolver_json(float v1, int v2, int v3, float v4, float v5, float v6, int v7, int v8, int v9, float v10, float v11,
                     float v12, int v13, int v14, int v15, float v16, float v17, float v18) {

  DynamicJsonDocument doc(capacidad);

  //le mandamos la fecha y la hora del metodo tiempo_actual
  doc["time"] = tiempo_actual();

  JsonObject body = doc.createNestedObject("body");

  JsonObject body_temperature = body.createNestedObject("temperature");
  body_temperature["value"] = v1;
  body_temperature["type"] = "float";
  body_temperature["unit"] = "°C";

  JsonObject body_resistance = body.createNestedObject("resistance");
  body_resistance["value"] = v2;
  body_resistance["type"] = "int";
  body_resistance["unit"] = "Ohm";

  JsonObject body_BPM = body.createNestedObject("BPM");
  body_BPM["value"] = v3;
  body_BPM["type"] = "int";
  body_BPM["unit"] = "Dimensionless";

  JsonObject body_SpO2 = body.createNestedObject("SpO2");
  body_SpO2["value"] = v4;
  body_SpO2["type"] = "float";
  body_SpO2["unit"] = "%";

  JsonObject ambient = doc.createNestedObject("ambient");

  JsonObject ambient_temperature = ambient.createNestedObject("temperature");
  ambient_temperature["value"] = v5;
  ambient_temperature["type"] = "float";
  ambient_temperature["unit"] = "°C";

  JsonObject ambient_humidity = ambient.createNestedObject("humidity");
  ambient_humidity["value"] = v6;
  ambient_humidity["type"] = "float";
  ambient_humidity["unit"] = "%";

  JsonObject device = doc.createNestedObject("device");
  device["UUID"] = "550e8400-e29b-41d4-a716-446655440000";

  JsonObject device_acceleration = device.createNestedObject("acceleration");

  JsonObject device_acceleration_ax = device_acceleration.createNestedObject("ax");
  device_acceleration_ax["value"] = v7;
  device_acceleration_ax["type"] = "int";
  device_acceleration_ax["unit"] = "mg";

  JsonObject device_acceleration_ay = device_acceleration.createNestedObject("ay");
  device_acceleration_ay["value"] = v8;
  device_acceleration_ay["type"] = "int";
  device_acceleration_ay["unit"] = "mg";

  JsonObject device_acceleration_az = device_acceleration.createNestedObject("az");
  device_acceleration_az["value"] = v9;
  device_acceleration_az["type"] = "int";
  device_acceleration_az["unit"] = "mg";

  JsonObject device_gyro = device.createNestedObject("gyro");

  JsonObject device_gyro_gx = device_gyro.createNestedObject("gx");
  device_gyro_gx["value"] = v10;
  device_gyro_gx["type"] = "float";
  device_gyro_gx["unit"] = "deg/s";

  JsonObject device_gyro_gy = device_gyro.createNestedObject("gy");
  device_gyro_gy["value"] = v11;
  device_gyro_gy["type"] = "float";
  device_gyro_gy["unit"] = "deg/s";

  JsonObject device_gyro_gz = device_gyro.createNestedObject("gz");
  device_gyro_gz["value"] = v12;
  device_gyro_gz["type"] = "float";
  device_gyro_gz["unit"] = "deg/s";

  JsonObject device_magnetometer = device.createNestedObject("magnetometer");

  JsonObject device_magnetometer_mx = device_magnetometer.createNestedObject("mx");
  device_magnetometer_mx["value"] = v13;
  device_magnetometer_mx["type"] = "int";
  device_magnetometer_mx["unit"] = "mG";

  JsonObject device_magnetometer_my = device_magnetometer.createNestedObject("my");
  device_magnetometer_my["value"] = v14;
  device_magnetometer_my["type"] = "int";
  device_magnetometer_my["unit"] = "mG";

  JsonObject device_magnetometer_mz = device_magnetometer.createNestedObject("mz");
  device_magnetometer_mz["value"] = v15;
  device_magnetometer_mz["type"] = "int";
  device_magnetometer_mz["unit"] = "mG";

  JsonObject device_pressure = device.createNestedObject("pressure");
  device_pressure["value"] = v16;
  device_pressure["type"] = "float";
  device_pressure["unit"] = "mb";

  JsonObject device_altitude = device.createNestedObject("altitude");
  device_altitude["value"] = v17;
  device_altitude["type"] = "float";
  device_altitude["unit"] = "m";

  JsonObject device_temperature = device.createNestedObject("temperature");
  device_temperature["value"] = v18;
  device_temperature["type"] = "float";
  device_temperature["unit"] = "°C";

  //char info[1200];
  String info;
  serializeJson(doc, info);
  //Serial.println(info);
  return info;
}
//*****************************************************CALCULA TIEMPO*****************************************************************************************************************************************************************
String tiempo_actual() {
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // Necesitamos extraer fecha y hora
  String formato = timeClient.getFormattedDate();
  //aqui obtenemos la fecha
  int splitT = formato.indexOf("T");
  String fecha = formato.substring(0, splitT);
  //aqui obtenemos la hora
  String hora = formato.substring(splitT + 1, formato.length() - 1);
  String fecha_hora = fecha + " " + hora ;
  return fecha_hora;
}//fin de tiempo_actual
//*****************************************************ENVIAR DATOS*****************************************************************************************************************************************************************
void enviar_dato_cliente(String info) {
  HTTPClient http;
  //Especificar destino para solicitud HTTP
  http.begin("http://11.11.27.131:8080/");
  //Especificar encabezado de tipo de contenido
  http.addHeader("Content-Type", "application/json");
  //Enviar la solicitud POST real
  int httpResponseCode = http.POST(info);
  if (httpResponseCode > 0) { //comprobamos la solicitud fue aceptada
    //Obtenga la respuesta a la solicitud
    String response = http.getString();
    Serial.println(httpResponseCode);   //Imprimir código de retorno si es posivito llego bien
    Serial.println(response);           //Imprimir solicitud de respuesta es lo que devuelvo para verificar que si llego como era
  } else {
    Serial.print("Error al enviar POST: "); Serial.println(httpResponseCode);// si el -1 no conecto al servidor
  } http.end(); //liberamos los recursos
}
