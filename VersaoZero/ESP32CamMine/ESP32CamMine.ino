/* ******ESP32 CAM SIGEAUTO*******
   *****ANDRESSA ROCHA VINHAL*****
   ***Atualizado em 05/07/2023****
*/

#include "esp_camera.h"
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include "BluetoothSerial.h"
#include "Arduino.h"
#include "esp32-hal-ledc.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "FS.h"
#include <SD_MMC.h>
#include <EEPROM.h>
#include "config.h" 

//chars para lidar com o http e 8266
const char* ssid = "CFC CAMERA";
const char* password = "esp8266pass";
const char* serverNameLati = "http://192.168.4.1/latitude";
const char* serverNameLong = "http://192.168.4.1/longitude";
const char* serverNameSpd = "http://192.168.4.1/velocidade";
const char* serverNameData = "http://192.168.4.1/data";
const char* serverNameHora = "http://192.168.4.1/hora";

String latitude;
String longitude;
String velocidade;
String localizacao;
String data;
String hora;
String dataHora;

//esperar tempo de recebimento do gps
unsigned long previousMillis = 0;
const int interval = 5000; 
//variáveis para temporização
unsigned long currentCaptureTime = 0;
unsigned long previousCaptureTime = 0;
const unsigned long captureInterval = 30000; // Intervalo de 30 segundos para captura contínua
//const unsigned long captureDuration = 120000; // Duração de 4 minutos para captura contínua
int capturasDesejadas = 4;
int capturasFeitas = 0;

//booleanos
bool requisicaoFoto = false;
bool enviandoFoto = false;
bool enviandoGPS = false;
bool modoNoturno = false;

//macaddress do esp32
String macAddress = "SIGEAUTO_";
BluetoothSerial SerialBT; //bluetooth

//numero da captura
String pictureNumber;
int numeroCaptura = 1;
//parametro recebido por bluetooth
int paramInt = 0;
//nome da pasta para salvar capturas
char nomePasta[100];

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
  Serial.begin(115200); //comunicação serial do ESP

  pinMode(4, OUTPUT);               // Blinding Disk-Avtive Light
  digitalWrite(4, LOW);             // turn off

  Serial.print("\n setup, core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
  
  initMicroSDCard();
  // Obtém o endereço MAC do ESP32Cam
  uint8_t mac[6];
  WiFi.macAddress(mac);
   // Converte o endereço MAC para uma string hexadecimal
  for (int i = 0; i < 6; i++) {
    macAddress += String(mac[i], HEX);
    if (i < 5) {
      macAddress += ":";
    }
  }
  initBT(macAddress); //inicia o bluetooth com o macAdress do esp32
  delay(500);

  WiFi.begin(ssid, password); // inicia o wifi
  Serial.print("Conectando à rede Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) { //checa o wifi
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado à rede Wi-Fi. Endereço IP: ");
  Serial.println(WiFi.localIP());

  Serial.println("Connected to WiFi");
  
  initCamera(); //Inicia a camera

  xTaskCreatePinnedToCore(LoopDasCapturas, "LoopDasCapturas", 8192, NULL, 1, NULL, 0);//Cria a tarefa "LoopDasCapturas()" com prioridade 1, atribuída ao core 0
  delay(1);
}

//setup do bluetooth
void initBT(String content){
  if(!SerialBT.begin(content)){
    Serial.println("An error occurred initializing Bluetooth");
    ESP.restart();
  }else{
    Serial.println("Bluetooth initialized");
  }

  SerialBT.register_callback(btCallback);
  Serial.println("The device started, now you can pair it with bluetooth");
}

// ****************** FAIXA DE VALORES RECEBIDOS PELO BTCALLBACK ******************
//
//        0 A 4 -- PARAMETROS DE CONFIGURAÇÃO DO FRAMESIZE
//        5 -- LIGAR MODO NOTURNO
//        6 -- INICIAR GRAVAÇÃO
//
// ********************************************************************************

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) { //recebe dados do bluetooth
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("\nClient Connected!");
  } else if (event == ESP_SPP_DATA_IND_EVT) {
    Serial.printf("ESP_SPP_DATA_IND_EVT len=%d, handle=%d\n\n", param->data_ind.len, param->data_ind.handle);
    
    String stringRead = "";
    for (int i = 0; i < param->data_ind.len; i++) {
      stringRead += (char)param->data_ind.data[i];
    }
    
    if (stringRead.toInt() == 5) 
    {
      ativarModoNoturno();
    } 
    else if(stringRead.toInt() >= 0 && stringRead.toInt() <= 4)
    {
      paramInt = stringRead.toInt();
      Serial.printf("paramInt: %d\n", paramInt);
      setCameraParam(paramInt);
    }
    else if(stringRead.toInt() == 6)
    {
      Serial.printf("paramInt: %d\n", paramInt);
      previousCaptureTime = 0;
      capturasFeitas = 0;
      sprintf(nomePasta, "/Pasta%d", numeroCaptura);
      if(SD_MMC.mkdir(nomePasta))
      {
        Serial.printf("Pasta %s criada com sucesso", nomePasta);
      };
      requisicaoFoto = true;
    }
  }
}

//envia dados pelo bluetooth
void writeSerialBT(camera_fb_t *fb){ 
  SerialBT.write(fb->buf, fb->len);
  SerialBT.flush();
  delay(500);
  enviandoFoto = false;
}

void initCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_HD;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 1200);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
}

//ativa/desativa as configurações do modo noturno
void ativarModoNoturno()
{
  sensor_t * s = esp_camera_sensor_get();
  if(modoNoturno == false)
  {
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  modoNoturno = true;
  Serial.print("Modo noturno ativado");
  }
  else if(modoNoturno == true)
  {
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    modoNoturno = false;
    Serial.print("Modo noturno desativado");
  }
}

//inicializa o micro SD card
void initMicroSDCard(){
  // Start Micro SD card
  Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }
}

//faz a troca de caracteres inválidos
String replaceInvalidCharacters(String str, char invalidChar, char replacementChar) {
  String result = "";
  for (size_t i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (c == invalidChar) {
      result += replacementChar;
    } else {
      result += c;
    }
  }
  return result;
}

//gera o nome do arquivo para salvar a imagem
String getPictureFilename(){
  data.trim(); // Remove espaços em branco da string 'data'
  hora.trim(); // Remove espaços em branco da string 'hora'

  pictureNumber = data + "e" + hora;
  pictureNumber = replaceInvalidCharacters(pictureNumber, '/', '-');
  pictureNumber = replaceInvalidCharacters(pictureNumber, ':', '_');
  
  String filename = "/captura_" + pictureNumber + ".jpg";
  return filename;
}

void setCameraParam(int paramInt){
  sensor_t *s = esp_camera_sensor_get();
  switch(paramInt){
    case 4:
      s->set_framesize(s, FRAMESIZE_HD);
    break;

    case 3:
      s->set_framesize(s, FRAMESIZE_XGA);
    break;

    case 2:
      s->set_framesize(s, FRAMESIZE_SVGA);
    break;

    case 1:
      s->set_framesize(s, FRAMESIZE_VGA);
    break;

    case 0:
    default:
      s->set_framesize(s, FRAMESIZE_HVGA);
    break;
  }
  captureIndividual();
}

//utilizada para a captura individual
void captureIndividual(){
  if(enviandoGPS)
  {
    delay(200);
  }
  enviandoFoto = true;
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb); // dispose the buffered image
  fb = NULL; // reset to capture errors
  fb = esp_camera_fb_get(); // get fresh image
  if(!fb){
    esp_camera_fb_return(fb);
    return;
  }
  
  if(fb->format != PIXFORMAT_JPEG){
    return;
  }

//path onde sera salva a captura no SD
  String path = getPictureFilename();
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(),FILE_WRITE);
  if(!file){
    Serial.printf("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved: %s\n", path.c_str());
  }
  file.close();

  writeSerialBT(fb);
  esp_camera_fb_return(fb);
}
//utilizada para a captura continua
void captureContinuous()
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb); // dispose the buffered image
  fb = NULL; // reset to capture errors
  fb = esp_camera_fb_get(); // get fresh image
  if(!fb){
    esp_camera_fb_return(fb);
    return;
  }
  
  if(fb->format != PIXFORMAT_JPEG){
    return;
  }

//path onde sera salva a captura no SD
  String pathContinuous = String(nomePasta) + "/captura_" + numeroCaptura + ".jpg";
  Serial.printf("\nPicture file name: %s\n", pathContinuous.c_str());
  
  fs::FS &fs = SD_MMC; 
  File file = fs.open(pathContinuous.c_str(),FILE_WRITE);
  if(!file){
    Serial.printf("\nFailed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("\nSaved: %s\n", pathContinuous.c_str());
  }
  numeroCaptura++;
  file.close();
  esp_camera_fb_return(fb);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval) {

      if(!enviandoFoto)
      {
      enviandoGPS = true;
      latitude = httpGETRequest(serverNameLati);
      longitude = httpGETRequest(serverNameLong);
      velocidade = httpGETRequest(serverNameSpd);
      data = httpGETRequest(serverNameData);
      hora = httpGETRequest(serverNameHora);
      //Serial.println("Latitude: " + latitude + "  - Longitude: " + longitude + "  - Velocidade: " + velocidade + " km/h" + " - Data: " + data + " - Hora: " + hora);

      localizacao = "Localização:" + latitude + "/" + longitude + ";";
      velocidade = "Velocidade:" + velocidade + " km/h" + "]";
      dataHora = "Data e Hora:" + data + "--- " + hora + ">";

      SerialBT.print(localizacao);
      SerialBT.print(velocidade);
      SerialBT.print(dataHora);
      enviandoGPS = false;
      }
      else
      {
        Serial.print("Estou ocupado enviando uma foto...");
      }
      
      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    delay(10);
  }

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET(); 
  String payload = "--"; 
  
  if (httpResponseCode>0) {
    //Serial.print("HTTP Response code: ");
    //Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void LoopDasCapturas(void* pvParameters)
{
  Serial.printf("\nLoop das Capturas em core: %d", xPortGetCoreID());//Mostra no monitor em qual core o loop das capturas foi chamado
  while (1)
  {
    if(requisicaoFoto)
    {
      currentCaptureTime = millis();

      if(capturasFeitas >= capturasDesejadas)
      {
        requisicaoFoto = false;
        currentCaptureTime = 0;
        Serial.print("\nAcabaram as capturas");
      }
      else if(currentCaptureTime - previousCaptureTime >= captureInterval)
      {
        captureContinuous();
        capturasFeitas++;
        previousCaptureTime = millis();
      }
    }
    delay(10);
  }
}