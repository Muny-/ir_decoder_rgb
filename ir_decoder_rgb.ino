#include <EEPROM.h>
#include <avr/interrupt.h>

//#define DEBUG_MODE

#if defined(__AVR_ATtiny85__)

#if defined(DEBUG_MODE)

#include <BasicSerial.h>

#endif

#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PINB:(((P)>7&&(P)<14)))

#endif

#if defined(__AVR_ATmega328P__)

#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))

#endif

#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

#define PERCENT_TOLERANCE 25  // percent tolerance in measurements

/* 
 * These revised MATCH routines allow you to use either percentage or absolute tolerances.
 * Use ABS_MATCH for absolute and PERC_MATCH for percentages. The original MATCH macro
 * is controlled by the IRLIB_USE_PERCENT definition a few lines above.
 */
 
#define PERCENT_LOW(us) (unsigned int) (((us)*(1.0 - PERCENT_TOLERANCE/100.)))
#define PERCENT_HIGH(us) (unsigned int) (((us)*(1.0 + PERCENT_TOLERANCE/100.) + 1))

#define PERC_MATCH(v,e) ((v) >= PERCENT_LOW(e) && (v) <= PERCENT_HIGH(e))

#define MATCH(v,e) PERC_MATCH(v,e)

// BUTTON CODES (44) (YES, I PRESSED AND RECORDED EVERY SINGLE BUTTON .-.)
const uint32_t POWER            = 16712445;
const uint32_t PLAY_PAUSE       = 16745085;
const uint32_t BRIGHT_LOW       = 16759365;
const uint32_t BRIGHT_HIGH      = 16726725;
const uint32_t RED              = 16718565;
const uint32_t GREEN            = 16751205;
const uint32_t BLUE             = 16753245;
const uint32_t WHITE            = 16720605;
const uint32_t PINK_TOP         = 16716525;
const uint32_t LIGHTER_BLUE      = 16749165;
const uint32_t PALE_GREEN       = 16755285;
const uint32_t ORANGE           = 16722645;
const uint32_t DARK_ORANGE      = 16714485;
const uint32_t PALE_BLUE        = 16747125;
const uint32_t PURPLE           = 16757325;
const uint32_t PINK_BOTTOM      = 16724685;
const uint32_t SKY_BLUE_TOP     = 16775175;
const uint32_t VIOLET           = 16742535;
const uint32_t TEAL             = 16758855;
const uint32_t WHITE_SKIN_COLOR = 16726215;
const uint32_t YELLOW           = 16718055;
const uint32_t DARK_TEAL        = 16750695;
const uint32_t HOT_PINK         = 16734375;
const uint32_t SKY_BLUE_BOTTOM  = 16767015;
const uint32_t QUICK            = 16771095;
const uint32_t UP_BLUE          = 16738455;
const uint32_t UP_GREEN         = 16754775;
const uint32_t UP_RED           = 16722135;
const uint32_t DOWN_RED         = 16713975;
const uint32_t DOWN_GREEN       = 16746615;
const uint32_t DOWN_BLUE        = 16730295;
const uint32_t SLOW             = 16762935;
const uint32_t AUTO             = 16773135;
const uint32_t DIY3             = 16740495;
const uint32_t DIY2             = 16756815;
const uint32_t DIY1             = 16724175;
const uint32_t DIY4             = 16716015;
const uint32_t DIY5             = 16748655;
const uint32_t DIY6             = 16732335;
const uint32_t FLASH            = 16764975;
const uint32_t FADE7            = 16769055;
const uint32_t FADE3            = 16736415;
const uint32_t JUMP7            = 16752735;
const uint32_t JUMP3            = 16720095;
// END BUTTON CODES

#if defined(__AVR_ATtiny85__)

#if defined(DEBUG_MODE)

void serOut(const char* str)
{
   while (*str) TxByte (*str++);
}

#endif

#define RED_PIN PB1
#define GREEN_PIN PB0
#define BLUE_PIN PB4

#endif

#if defined(__AVR_ATmega328P__)

#define RED_PIN 10
#define GREEN_PIN 9
#define BLUE_PIN 6

#endif

void setup()
{
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    #if defined(__AVR_ATtiny85__)

    pinMode(PB2, INPUT);
    //digitalWrite(PB2, HIGH);

    // ATTINY85 (HW Pin 5, SW Pin 0):
    //attachInterrupt(PCINT3, IR_Change, CHANGE);

    attachInterrupt(0,IR_Change,CHANGE);

    #endif

    #if defined(__AVR_ATmega328P__)

    pinMode(2, INPUT);

    attachInterrupt(digitalPinToInterrupt(2), IR_Change, CHANGE);
    Serial.begin(250000);

    #endif

    handleDIYPress(1, true);
}

long lastDelta = 0;
bool lastState = 0;
uint16_t index = 0;
uint16_t lastProcessedIndex = -1;

long lastChange = 0;

uint16_t packetIndex = 0;

bool bits[32];

void IR_Change()
{
    long now = micros();

    lastDelta = now - lastChange;
    lastChange = now;

    lastState = digitalState(2);

    index++;
}

uint32_t buttonCode = 0;

bool Out1State = 1;

uint8_t red_value = 255;
uint8_t green_value = 255;
uint8_t blue_value = 255;

uint8_t target_red_value = 255;
uint8_t target_green_value = 255;
uint8_t target_blue_value = 255;

unsigned long lastColorChange = 0;

float brightness = 0;

float target_brightness = 100;

unsigned long lastBrightnessChange = 0;
unsigned long lastBrightnessChange_micros = 0;

uint16_t flashing_delay = 255;

uint16_t pulsing_delay = 5000;

int8_t pulse_delta = 1;

#define MODE_STEADY 0
#define MODE_FLASH 1
#define MODE_PULSE 2
#define MODE_STROBE 3

uint8_t MODE = MODE_STEADY;

#define SPECIAL_MODE_NORMAL 0
#define SPECIAL_MODE_POLICE 1
#define SPECIAL_MODE_ORANGE_WARNING 2
#define SPECIAL_MODE_COLOR_FLOWERING 3

uint8_t SPECIAL_MODE = 0;

//       POLICE MODE    ORANGE WARNING MODE     COLOR FLOWERING
//======================================================
// 0      FLASH RED      OFF                     255, 255, 255
// 1      FLASH BLUE     FLASH ORANGE            255, 255, 0
// 2                                             255, 0,   255
// 3                                             255, 0,   0
// 4                                             0,   255, 255
// 5                                             0,   0,   255
// 6                                             0,   255, 0
uint8_t specialModeState = 0;

unsigned long lastSpecialModeStateChange = 0;

uint8_t button_held_count = 0;

void loop()
{
    bool buttonPushed = false;
    bool buttonHeld = false;

    noInterrupts();

    if (index != lastProcessedIndex)
    {

        bool state = lastState;
        long delta = lastDelta;

        /*Serial.print(state);
        Serial.print(", ");
        Serial.print(delta);
        Serial.print(", pi: ");
        Serial.println(packetIndex);*/


        // last state was low, now is high
        if (state)
        {
            // packet not in progress
            if (packetIndex == 0)
            {
                if (MATCH(delta, 9000))
                {
                    packetIndex = 1;
                }
            }
            else if (packetIndex == 67)
            {
                buttonCode = ToULong(bits);
                buttonPushed = true;
                packetIndex = 0;
            }
            else if (packetIndex > 1)
            {
                if (packetIndex % 2 == 0)
                {
                    if (MATCH(delta, 660))
                    {
                        packetIndex++;
                    }
                    else
                    {
                        packetIndex = 0;
                    }
                }
            }
        }
        // last state was high, now is low
        else
        {
            if (packetIndex == 1)
            {
                if (MATCH(delta, 4500))
                {
                    packetIndex = 2;
                }
                else if (MATCH(delta, 2250))
                {
                    packetIndex = 0;
                    buttonHeld = true;
                }
                else
                {
                    packetIndex = 0;
                }
            }
            else if (packetIndex > 2 && packetIndex % 2 != 0)
            {
                int mapped_index = map(packetIndex, 3, 65, 0, 31);

                // low bit
                if (MATCH(delta, 460))
                {
                    bits[mapped_index] = 0;
                }
                // high bit
                else if (MATCH(delta, 1560))
                {
                    bits[mapped_index] = 1;
                }

                packetIndex++;
            }
        }

        //lastProcessedIndex = index;
    }

    interrupts();

    if (buttonPushed)
    {
        button_held_count = 1;

        switch(buttonCode)
        {
            default:
            {

                D("Unknown button pushed: ");
                D(buttonCode);
                D("\n");
            }
            break;  

            case POWER:
            {
                Out1State = !Out1State;

                if (!Out1State)
                {
                    MODE = MODE_STEADY;
                    SPECIAL_MODE = SPECIAL_MODE_NORMAL;
                }

                D("Power button pushed\n");
                D("Out state: " );
                D(Out1State);
                D("\n");
            }
            break;

            case BRIGHT_LOW:
            {
                D("b_lo\n");

                if (target_brightness > 4)
                    target_brightness -=2;
                else if (target_brightness <=4 && target_brightness > 1)
                    target_brightness -= 1;

                if (target_brightness < 0)
                    target_brightness = 0;
            }
            break;

            case BRIGHT_HIGH:
            {
                D("b_hi\n");

                if (target_brightness < 4)
                    target_brightness += 1;
                else if (target_brightness < 100)
                    target_brightness+=2;

                if (target_brightness > 100)
                    target_brightness = 100;
            }
            break;

            case FLASH:
            {
                if (Out1State)
                {
                    if (MODE != MODE_FLASH)
                        MODE = MODE_FLASH;
                    else
                    {
                        MODE = MODE_STEADY;
                        SPECIAL_MODE = SPECIAL_MODE_NORMAL;
                    }
                }
            }
            break;

            case FADE7:
            {
                if (Out1State)
                {
                    if (MODE != MODE_PULSE)
                        MODE = MODE_PULSE;
                    else
                        MODE = MODE_STEADY;
                }
            }
            break;

            case JUMP7:
            {
                if (Out1State)
                {
                    if (MODE != MODE_STROBE)
                        MODE = MODE_STROBE;
                    else
                        MODE = MODE_STEADY;
                }
            }
            break;

            case QUICK:
            {   
                if (flashing_delay != 15)
                    flashing_delay -= 15;

                if (pulsing_delay > 50)
                    pulsing_delay-= 100;

                if (pulsing_delay < 50)
                    pulsing_delay = 50;

                D("New flashing delay: ");
                D(flashing_delay);
                D("\n");
            }
            break;

            case SLOW:
            {
                flashing_delay += 15;

                if (pulsing_delay < 16383)
                    pulsing_delay+= 100;

                if (pulsing_delay > 16383)
                    pulsing_delay = 16383;
            }
            break;

            case RED:
            {
                setColor(255, 0, 0);
            }
            break;

            case GREEN:
            {
                setColor(0, 255, 0);
            }
            break;

            case BLUE:
            {
                setColor(0, 0, 255);
            }
            break;

            case WHITE:
            {
                setColor(255, 255, 255);
            }
            break;

            case PINK_TOP:
            {
                setColor(255,0,180);
            }
            break;

            case LIGHTER_BLUE:
            {
                setColor(30,144,255);
            }
            break;

            case PALE_GREEN:
            {
                setColor(0,255,127);
            }
            break;

            case ORANGE:
            {
                setColor(255,50,0);
            }
            break;

            case DARK_ORANGE:
            {
                setColor(255,20,0);
            }
            break;

            case PALE_BLUE:
            {
                setColor(0,191,255);
            }
            break;

            case PURPLE:
            {
                setColor(0,12,255);
            }
            break;

            case PINK_BOTTOM:
            {
                setColor(255,10,130);
            }
            break;

            case SKY_BLUE_TOP:
            {
                setColor(95,158,160);
            }
            break;

            case VIOLET:
            {
                setColor(160,0,215);
            }
            break;

            case TEAL:
            {
                setColor(0,128,128);
            }
            break;

            case WHITE_SKIN_COLOR:
            {
                setColor(255,60,0);
            }
            break;

            case YELLOW:
            {
                setColor(255,255,0);
            }
            break;

            case DARK_TEAL:
            {
                setColor(0,190,190);
            }
            break;

            case HOT_PINK:
            {
                setColor(255,0,10);
            }
            break;

            case SKY_BLUE_BOTTOM:
            {
                setColor(105,168,170);
            }
            break;

            case UP_RED:
            {
                if (red_value < 255)
                    setColor(red_value+1, green_value, blue_value);
            }
            break;

            case UP_GREEN:
            {
                if (green_value < 255)
                setColor(red_value, green_value+1, blue_value);
            }
            break;

            case UP_BLUE:
            {
                if (blue_value < 255)
                    setColor(red_value, green_value, blue_value+1);
            }
            break;

            case DOWN_RED:
            {
                if (red_value > 0)
                    setColor(red_value-1, green_value, blue_value);
            }
            break;

            case DOWN_GREEN:
            {
                if (green_value > 0)
                    setColor(red_value, green_value-1, blue_value);
            }
            break;

            case DOWN_BLUE:
            {
                if (blue_value > 0)
                    setColor(red_value, green_value, blue_value-1);
            }
            break;

            case PLAY_PAUSE:
            {
                if (Out1State)
                {
                    if (SPECIAL_MODE == SPECIAL_MODE_POLICE)
                    {
                        SPECIAL_MODE = SPECIAL_MODE_ORANGE_WARNING;
                        MODE = MODE_FLASH;
                    }
                    else if (SPECIAL_MODE == SPECIAL_MODE_ORANGE_WARNING)
                    {
                        SPECIAL_MODE = SPECIAL_MODE_NORMAL;
                        MODE = MODE_STEADY;
                    }
                    else if (SPECIAL_MODE == SPECIAL_MODE_NORMAL)
                    {
                        SPECIAL_MODE = SPECIAL_MODE_POLICE;
                        MODE = MODE_FLASH;
                    }
                }
            }
            break;

            case DIY1:
            {
                handleDIYPress(1, false);
            }
            break;

            case DIY2:
            {
                handleDIYPress(2, false);
            }
            break;

            case DIY3:
            {
                handleDIYPress(3, false);
            }
            break;

            case DIY4:
            {
                handleDIYPress(4, false);
            }
            break;

            case DIY5:
            {
                handleDIYPress(5, false);
            }
            break;

            case DIY6:
            {
                handleDIYPress(6, false);
            }
            break;

            case AUTO:
            {
                if (SPECIAL_MODE == SPECIAL_MODE_COLOR_FLOWERING)
                    SPECIAL_MODE = SPECIAL_MODE_NORMAL;
                else
                    SPECIAL_MODE = SPECIAL_MODE_COLOR_FLOWERING;
            }
            break;
        }

        if (MODE != MODE_FLASH && MODE != MODE_PULSE && MODE != MODE_STROBE)
            ProcessOutput();
    }
    else if (buttonHeld)
    {
        button_held_count++;

        D("Button being held\n");

        switch(buttonCode)
        {
            case BRIGHT_LOW:
            {
                if (target_brightness > 4)
                    target_brightness -=2;
                else if (target_brightness <=4 && target_brightness > 1)
                    target_brightness -= 1;

                if (target_brightness < 0)
                    target_brightness = 0;
            }
            break;

            case BRIGHT_HIGH:
            {
                if (target_brightness < 4)
                    target_brightness += 1;
                else if (target_brightness < 100)
                    target_brightness+=2;

                if (target_brightness > 100)
                    target_brightness = 100;
            }
            break;

            case QUICK:
            {   
                if (flashing_delay != 15)
                    flashing_delay -= 15;

                if (pulsing_delay > 50)
                    pulsing_delay-= 100;

                if (pulsing_delay < 50)
                    pulsing_delay = 50;
            }
            break;

            case SLOW:
            {
                flashing_delay += 15;

                if (pulsing_delay < 16383)
                    pulsing_delay+= 100;

                if (pulsing_delay > 16383)
                    pulsing_delay = 16383;
            }
            break;

            case UP_RED:
            {
                if (red_value < 255)
                    setColor(red_value+1, green_value, blue_value);
            }
            break;

            case UP_GREEN:
            {
                if (green_value < 255)
                setColor(red_value, green_value+1, blue_value);
            }
            break;

            case UP_BLUE:
            {
                if (blue_value < 255)
                    setColor(red_value, green_value, blue_value+1);
            }
            break;

            case DOWN_RED:
            {
                if (red_value > 0)
                    setColor(red_value-1, green_value, blue_value);
            }
            break;

            case DOWN_GREEN:
            {
                if (green_value > 0)
                    setColor(red_value, green_value-1, blue_value);
            }
            break;

            case DOWN_BLUE:
            {
                if (blue_value > 0)
                    setColor(red_value, green_value, blue_value-1);
            }
            break;

            case DIY1:
            {
                handleDIYHeld(1);
            }

            case DIY2:
            {
                handleDIYHeld(2);
            }

            case DIY3:
            {
                handleDIYHeld(3);
            }

            case DIY4:
            {
                handleDIYHeld(4);
            }

            case DIY5:
            {
                handleDIYHeld(5);
            }

            case DIY6:
            {
                handleDIYHeld(6);
            }
        }

        if (MODE != MODE_FLASH && MODE != MODE_PULSE && MODE != MODE_STROBE)
            ProcessOutput();
    }

    unsigned long now = millis();
    unsigned long now_micros = micros();

    if (now - lastColorChange > 10)
    {
        if (red_value != target_red_value || green_value != target_green_value || blue_value != target_blue_value)
        {
            if (red_value < target_red_value)
                red_value++;
            else if (red_value > target_red_value)
                red_value--;

            if (green_value < target_green_value)
                green_value++;
            else if (green_value > target_green_value)
                green_value--;

            if (blue_value < target_blue_value)
                blue_value++;
            else if (blue_value > target_blue_value)
                blue_value--;

            ProcessOutput();
        }

        lastColorChange = now;


    }

    if (MODE == MODE_STEADY)
    {
        if (now - lastBrightnessChange > 10 && Out1State && brightness != target_brightness)
        {
            if (brightness > target_brightness)
                brightness--;
            else if (brightness < target_brightness)
                brightness++;

            ProcessOutput();
        }
        else if (now - lastBrightnessChange > 10 && !Out1State && brightness > 0)
        {
            ProcessOutput();
        }
    }
    else if (MODE == MODE_FLASH && Out1State)
    {
        if (now - lastBrightnessChange > flashing_delay)
        {
            if (brightness == target_brightness)
                brightness = 0;
            else if (brightness == 0)
                brightness = target_brightness;
            else
                brightness = 0;

            ProcessOutput();
        }
    }
    else if (MODE == MODE_PULSE && Out1State)
    {

        if (now_micros - lastBrightnessChange_micros > pulsing_delay)
        {
            if (brightness == target_brightness)
            {
                pulse_delta = -1;
            }
            else if (brightness == 0)
            {
                pulse_delta = 1;
            }

            brightness += pulse_delta;

            ProcessOutput();
        }
    }
    else if (MODE == MODE_STROBE && Out1State)
    {
        if (brightness > 0 && now_micros - lastBrightnessChange_micros > 2000)
        {
            brightness = 0;

            ProcessOutput();
        }
        else if (brightness == 0 && now - lastBrightnessChange > flashing_delay)
        {
            brightness = 100;

            ProcessOutput();
        }
    }

    if (SPECIAL_MODE == SPECIAL_MODE_POLICE)
    {
        if (now - lastSpecialModeStateChange > flashing_delay*4 && brightness == 0)
        {
            if (specialModeState == 0)
            {
                setColorFast(255, 0, 0);

                specialModeState = 1;
            }
            else if (specialModeState == 1)
            {
                setColorFast(0, 0, 255);

                specialModeState = 0;
            }
            else
                specialModeState = 0;

            lastSpecialModeStateChange = now;
        }
    }
    else if (SPECIAL_MODE == SPECIAL_MODE_ORANGE_WARNING)
    {
        // 255, 25, 0
        if (now - lastSpecialModeStateChange > flashing_delay * 6 && brightness == 0)
        {
            if (specialModeState == 0)
            {
                setColorFast(0, 0, 0);

                specialModeState = 1;
            }
            else if (specialModeState == 1)
            {
                setColorFast(255, 25, 0);

                specialModeState = 0;
            }
            else
                specialModeState = 0;

            lastSpecialModeStateChange = now;
        }
    }
    else if (SPECIAL_MODE == SPECIAL_MODE_COLOR_FLOWERING)
    {
        if (now - lastSpecialModeStateChange > flashing_delay * 12)
        {
            if (specialModeState == 0)
            {
                setColor(255, 255, 255);

                specialModeState = 1;
            }
            else if (specialModeState == 1)
            {
                setColor(255, 255, 0);

                specialModeState = 2;
            }
            else if (specialModeState == 2)
            {
                setColor(255, 0, 255);

                specialModeState = 3;
            }
            else if (specialModeState == 3)
            {
                setColor(255, 0, 0);

                specialModeState = 4;
            }
            else if (specialModeState == 4)
            {
                setColor(0, 255, 255);

                specialModeState = 5;
            }
            else if (specialModeState == 5)
            {
                setColor(0, 0, 255);

                specialModeState = 6;
            }
            else if (specialModeState == 6)
            {
                setColor(0, 255, 0);

                specialModeState = 0;
            }

            lastSpecialModeStateChange = now;
        }
    }
}

uint8_t before_diy_change[3] = {
    0,
    0,
    0
};

void DIYColor(uint8_t number, byte *buf)
{
    uint16_t address = (number-1)*3;

    buf[0] = EEPROM.read(address);

    address++;

    buf[1] = EEPROM.read(address);

    address++;

    buf[2] = EEPROM.read(address);
}

void write_DIYColor(byte color[3], uint8_t number)
{
    uint16_t address = (number-1)*3;

    EEPROM.write(address, color[0]);
    address++;
    EEPROM.write(address, color[1]);
    address++;
    EEPROM.write(address, color[2]);
}

void handleDIYPress(uint8_t number, bool fast)
{
    before_diy_change[0] = red_value;
    before_diy_change[1] = green_value;
    before_diy_change[2] = blue_value;

    byte color[3];

    DIYColor(number, color);

    if (fast)
        setColorFast(color[0], color[1], color[2]);
    else
        setColor(color[0], color[1], color[2]);
}

void handleDIYHeld(uint8_t number)
{
    if (button_held_count == 10)
    {
        byte color[3];

        color[0] = before_diy_change[0];
        color[1] = before_diy_change[1];
        color[2] = before_diy_change[2];

        write_DIYColor(color, number);

        setColorFast(color[0], color[1], color[2]);

        delay(250);

        setColorFast(0,0,0);

        delay(250);

        handleDIYPress(number, false);

        button_held_count = 1;
    }
}

void setColorFast(uint8_t r, uint8_t g, uint8_t b)
{
    if (r > 255)
        r = 255;

    if (g > 255)
        g = 255;

    if (b > 255)
        b = 255;

    red_value = r;
    green_value = g;
    blue_value = b;

    target_red_value = r;
    target_green_value = g;
    target_blue_value = b;

    ProcessOutput();
}

void setColor(uint8_t r, uint8_t g, uint8_t b)
{
    if (r > 255)
        r = 255;

    if (g > 255)
        g = 255;

    if (b > 255)
        b = 255;

    target_red_value = r;
    target_green_value = g;
    target_blue_value = b;

    ProcessOutput();

    D("Color: ");
    D("rgb(");
    D(r);
    D(", ");
    D(g);
    D(", ");
    D(b);
    D(")\n");
}

void ProcessOutput()
{
    lastBrightnessChange = millis();
    lastBrightnessChange_micros = micros();

    if (Out1State)
    {
        float percent = (brightness/100);
        analogWrite(RED_PIN, percent*red_value);
        analogWrite(GREEN_PIN, percent*green_value);
        analogWrite(BLUE_PIN, percent*blue_value);
        /*D("Set brightness to: ");
        D(brightness);
        D("\n");*/
    }
    else
    {
        if (brightness > 0)
            brightness--;

        float percent = (brightness/100);

        analogWrite(RED_PIN, percent*red_value);
        analogWrite(GREEN_PIN, percent*green_value);
        analogWrite(BLUE_PIN, percent*blue_value);
        /*D("Turning(ed) off, brightness: ");
        D(brightness);
        D("\n");*/
    }
}

uint32_t ToULong(bool b[32])
{
    uint32_t c = 0;
    for (int i=0; i < 32; i++)
    {

        c = c << 1;

        if (b[i])
            c |= 1;
    }

    return c;
}

void D(bool out)
{
    #if defined(DEBUG_MODE)

    char textToWrite[ 1 ];
    sprintf(textToWrite,"%s", (out)?"1":"0");

    D(textToWrite);

    #endif
}

void D(long out)
{
    #if defined(DEBUG_MODE)

    char textToWrite[ 16 ];
    sprintf(textToWrite,"%lu", out);

    D(textToWrite);

    #endif
}

void D(uint32_t out)
{
    #if defined(DEBUG_MODE)

    char textToWrite[ 16 ];
    sprintf(textToWrite,"%lu", out);

    D(textToWrite);

    #endif
}

void D(uint16_t out)
{
    #if defined(DEBUG_MODE)

    char textToWrite[ 16 ];
    sprintf(textToWrite,"%u", out);

    D(textToWrite);

    #endif
}

void D(uint8_t out)
{
    #if defined(DEBUG_MODE)

    char textToWrite[ 4 ];
    sprintf(textToWrite,"%hu", out);

    D(textToWrite);

    #endif
}

void D(char* out)
{
    #if defined(DEBUG_MODE)

    #if defined(__AVR_ATtiny85__)

    serOut(out);

    #endif

    #if defined(__AVR_ATmega328P__)

    Serial.print(out);

    #endif

    #endif
}
