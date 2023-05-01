/*
  PALISSE VOLIA
  WAECHTER THIBAUT
  GE2
*/

/****************************************************************************************
 * TIPE 
 ****************************************************************************************/

// Include the servo library
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/****************************************************************************************
 * ENUMERATION
 ****************************************************************************************/

// Declaration des actions possibles
enum
{
    OPEN, //pour ouverture de la main
    CLOSE,//pour fermeture de la main
    REST  //la main reste en place
};

/****************************************************************************************/


/****************************************************************************************
 * OLED DISPLAY
 ****************************************************************************************/

#define SCREEN_WIDTH 128 // largeur de l'écran OLED, en nombre de pixels
#define SCREEN_HEIGHT 32 // largeur de l'écran OLED, en nombre de pixels

// Declaration de l'écran SSD1306 connecté en I2C (SDA, SCL pins)
// Les pins I2C sont définis par la bibliothère Wire-library. 
// Sur notre carte arduino UNO, il s'agit des pins :       A4(SDA), A5(SCL)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c ///< Regarder la datasheet pour l'adrresse; 0x3D pour 128x64, 0x3C pour 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //déclaration de l'écran, sous le nom de display

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


/****************************************************************************************
 * PINS
 ****************************************************************************************/

// PWM pins (servo)

const int O_servo = 10; //pin du servo nécessaire à l'ouverture de la main
const int C_servo = 11; //pin du servo nécessaire à la fermeture de la main

/****************************************************************************************/

// Digital pins

const int m_sensor = 8; //pin du capteur MyOWare
const int SELECT_sensor = 4; //pin du bouton

/****************************************************************************************/

// Threshold du capteur MyOWare
const int MyOWare_threshold = 1.7; 

/****************************************************************************************
 * VARIABLES
 ****************************************************************************************/

// Declaration des servo

Servo O; //O pour Open
Servo C; //C pour Close

/****************************************************************************************/

int action_mode = REST;
/*
 * la variable action_mode va nous permettre de définir le mode dans quel mode nous nous trouvons
 * parmi les trois modes énuméré plus tard
 */

double v; //potentiel mesuré par le capteur MyOWare

//Vitesse maximum
int max_speed = 100;

// Vitesse initiale
const int start_speed = 90;

// Rise time between each speed
const int min_delay = 12;


int state_count = 1;                      //Permet de compter le nombre de changement de mode
boolean etalon = false;                   //Permet de savoir si le système à été étalonné ou non
boolean test = false;                     //Variable nécéssaire à l'étalonnage
boolean test_2 = false ;                  //Variable nécéssaire à l'étalonnage
double t ;                                //Variable de temps
double t_max = 10000;                     //Variable de temps
double t_treshold = 10000;                //Variable de temps
double t_init;                            //Variable de temps
boolean mode_choisi;                      //Permet de savoir si un mode a été choisi
int count_error = 0;

/****************************************************************************************
 * DECLARATION FUNCTIONS
 ****************************************************************************************/

/// @brief Fonction ouverture de la main.
void open_hand();

/// @brief Fonction fermeture de la main.
void close_hand();

/// @brief Arret des moteurs.
void stop_bot();

/// @brief Etalonnange du système.
void etalonnage();

/// @brief Fonction renvoyant le potentiel lu par le capteur MyOWare.
double voltage();

/****************************************************************************************/

// Setup:
void setup() 
{
  // Rate of communication
  Serial.begin(9600);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Problème : boucle infinie
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  
  // Driver des servo
  pinMode(O_servo, OUTPUT);
  pinMode(C_servo, OUTPUT);
  
  // MyoWare sensor
  pinMode(m_sensor,OUTPUT);
  pinMode(SELECT_sensor,INPUT);
  
  //Affichage initiale
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(5, 0);
  display.println("Press to");
  display.setTextSize(2);
  display.setCursor(15, 17);
  display.println("Open");
  display.display();      // Show initial text
  
  
}

// loop:
void loop() 
{
    if( digitalRead(SELECT_sensor)==1 && etalon != true )
    {
      etalon = true;
      etalonnage();
    }
    
    if(etalon == true)
    {
      update_mode(); // Selection du mode
      delay(100);
    
    switch(action_mode)
    {
      
      case OPEN:
      //Mode ouverture
      
      if(((millis() - t)) < t_treshold)
      /*
       * 0 étant la "position" pour laquelle la main est ouverte au maximum 
       * t_init la "position" initiale (position en fin de mode précédent)
       * millis() - t la durée depuis que le mode à été lancé
       * t_init - durée_mode_ouverture doit donc toujours être > 0
       */
      
      {
        open_hand();
        Serial.print("duree mode ");
        Serial.print(millis() - t);
        Serial.println("");
        display.clearDisplay();
        display.setTextSize(1); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(5, 0);
        display.println("Opening");
        display.setTextSize(2);
        display.setCursor(15, 17);
        display.println("Hand");
        display.display();      // Show initial text
        
      }
      else
      /*
       *Sinon 
       *On passe au mode suivant
       */
      {
        stop_bot();
        Serial.print("Ouverture max atteinte");
        display.clearDisplay();
        display.setTextSize(1); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(5, 0);
        display.println("Maximum");
        display.setTextSize(2);
        display.setCursor(15, 17);
        display.println("Aperture");
        display.display();      // Show initial text
      
        delay(2000);
       
        count_error = 0; 
        action_mode = REST;
        
        //On change alors de mode
      }
         
        break; 
         
      case CLOSE:
     
      //Mode fermeture
      if((millis()- t)< t_treshold)
      /*
       * t_treshold étant la "position" pour laquelle la main est fermé au maximum 
       * Cette valeur est obtenue après étalonnage
       * t_init la "position" initiale ("position" en fin de mode précédent)
       * millis() - t la durée depuis que le mode à été choisi
       * t_init + durée_mode_fermeture doit donc toujours être < t_treshold
       */
      {
        close_hand();
        Serial.print("duree mode");
        Serial.print(millis()-t);
        Serial.println("");
        display.clearDisplay();
        display.setTextSize(1); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(5, 0);
        display.println("Closing");
        display.setTextSize(2);
        display.setCursor(15, 17);
        display.println("Hand");
        display.display();      // Show initial text
       
      }
      else
      {
    
        stop_bot();
    
        display.clearDisplay();
        display.setTextSize(1); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(5, 0);
        display.println("Maximum");
        display.setTextSize(2);
        display.setCursor(15, 17);
        display.println("Closure");
        display.display();      // Show initial text
        count_error = 0;
        
        Serial.print("avant delais");
        
        delay(2000);
        Serial.print("apres delais");
        action_mode = REST;
        
        //On change alors de mode
      }
        
        break;
        
      case REST:
        stop_bot();
        delay(500);
        break;
        
    }
    }
 }

/****************************************************************************************
 * GENERAL PURPOSE FUNCTIONS
 ****************************************************************************************/

void etalonnage()
/*
 * Dans cette fonction, on va étalonner le système : 
 * On ouvre tout d'abord la main de l'utilisateur au maximum
 * Puis on la ferme et on mesure le temps qu'il met pour se fermer au maximum
 * 
 */

{
  t = millis();
  test = false;
  Serial.print("Début de l'étalonnage");
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(5, 0);
  display.println("Press to");
  display.setTextSize(2);
  display.setCursor(15, 17);
  display.println("Stop");
  display.display();      // Show initial text
  open_hand();
  delay(1000);
  display.clearDisplay();
  while((test == false))
  {
    Serial.println("Appuyer pour arrêter l'ouverture");
    
    if(digitalRead(SELECT_sensor) == 1)
    {
      Serial.println("Ouverture stoppée");
      display.clearDisplay();
      test = true;
      delay(500);
    }
  }
  stop_bot();
  delay(1000);
  
  if(test == true)
  {
    display.clearDisplay();
    display.setTextSize(1); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(5, 0);
    display.println("Press to");
    display.setTextSize(2);
    display.setCursor(15, 17);
    display.println("Close");
    display.display();      // Show initial text
    test = false;
    Serial.println("Appuyer pour lancer la fermeture");
    //on est en position ouverte au maximum
    
    while( test == false)
    {
      
      if(digitalRead(SELECT_sensor) == 1 )
      //Si le bouton est pressé
      {
        test = true;
        test_2 = true ;
      }
    }
    delay(300);
    if(test_2 == true){
      test = false;
      display.clearDisplay();
      display.setTextSize(1); // Draw 2X-scale text
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(5, 0);
      display.println("Press to");
      display.setTextSize(2);
      display.setCursor(15, 17);
      display.println("Stop");
      display.display();      // Show initial text
      
      t = millis(); //début du chronomètre
      
      while( (test == false))
      {
        Serial.println("appuyer sur le bouton pour fermer");
        
        Serial.println(millis() - t);
        close_hand();
        
        if(digitalRead(SELECT_sensor) == 1)
        {
          test = true;
          Serial.println("Main fermé au maximum, fin de l'étalonnage");
          
          t_treshold = millis() - t ; //Durée finale de la fermture
          Serial.println(t_treshold);
          //t_treshold correspond au temps que met la main pour se fermer au maximum
           
          
          action_mode = REST;  //mode choisi     
          count_error = 0;
        }
      }
      stop_bot();
      t=0;
      display.clearDisplay();
      delay(2000);
      
  } 
 }
    
       
}

void update_mode()
/*
 * Cette méthode nous permet de créer un cycle de mode grâce à la variable state_count:
 *  - state_count = 1 : mode = OPEN
 *  - state_count = 2 : mode = CLOSE
 *  Si state_count > 2, alors on met la variable à 1
 *  On parvient ainsi à créer un cycle
 */
{ 
    // Determination du mode en fontion du potentiel
    v = voltage();
    Serial.println("voltage = ");
    Serial.println(v);
    delay(500);

    if(millis()-t<t_treshold)
    {
      mode_choisi = true;
    }
    else
    {
      mode_choisi = false;
    }
    
    if(v > MyOWare_threshold && (mode_choisi == false))
    //le potentiel lu est suffisant et on change de mode
    {
     
      t = millis();
      //début du nouveau mode
      
      if(state_count > 2)
      //On a dépassé le nombre de mode 
      {
        state_count = 1;
      }
      
      
      if(state_count == 1)
      {
        action_mode = OPEN;
        display.clearDisplay();
        state_count++;
        delay(100);
        Serial.println("OPEN");
        display.print("OPEN");
        display.display();      // Show initial text
        mode_choisi = true;
        
 
      }
      else if(state_count == 2)
      {
        action_mode = CLOSE;
        display.clearDisplay();
        state_count++;
        delay(100);
        Serial.println("CLOSE");
        display.print("CLOSE");
        display.display();      // Show initial text
        mode_choisi = true;
        
      }
    }
}

void open_hand()
//Ouverture de la main
{
  O.attach(O_servo);
  C.attach(C_servo);
  O.write(80);
  C.write(100);
 
  
  /*for(int i = start_speed ; i <= max_speed ; i++)
    {
        O.write(i);
        
        C.write(180 - i);
        delay(min_delay);
    }
*/
}

void close_hand()
//fermeture de la main
{
  
  O.attach(O_servo);
  C.attach(C_servo);
  delay(100);
  O.write(100);
  C.write(80);

  /*for(int i = start_speed ; i <= max_speed ; i++)
    {
        O.write(180 - i);
        
        
        C.write(i);
        delay(min_delay);
    }*/

}

void stop_bot()
//arret des moteurs
{
  O.detach();
  C.detach();

  // Wait a bit
  delay(1);

  // Middle position
  O.write(90);
  C.write(90);

  delay(200);
}

double voltage()
{
  // read the input on analog pin m_sensor:
  int sensorValue = analogRead(m_sensor);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float a = sensorValue * (5.0 / 1023.0);
  return a;
}


/****************************************************************************************
 * OLED DISPLAY FUNCTIONS
 ****************************************************************************************/

void print_2lign(char s1, char s2)
{
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(5, 0);
  display.println(s1);
  display.setTextSize(2);
  display.setCursor(15, 17);
  display.println(s2);
}

void print_textL1(char s)
{
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.print(s);
  display.display();      // Show initial 
}

void print_textL2(char s)
{
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.print(s);
  display.display();      // Show initial 
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}
