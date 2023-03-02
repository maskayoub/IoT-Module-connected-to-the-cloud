#include <msp430.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 */
//  entwicklung und programmierung eines iot messystems mit gsm-anbindung für low power anwendungen
// Software welche 60 Sekunden ruht (LPM3) und anschließend via GSM-Modul und AT-Commandos dafür sorgt, dass die SMS versendet wird
//
//
//
//_______________________________________________________________________________________________________________________________________
//VARIABLEN / CONSTANTEN UND BEDINGUNGEN
//_______________________________________________________________________________________________________________________________________
//Timer
int TimerB0_Task = 0; //Variable für Nutzung in Interrupt Routine des TimersB0, da er für verschiedene Funktionen genutzt wird

//ADC
const double bit = 12; //Bittiefe ADC
const double U0 = 3.0; //VCC in V
double vStep = 0; //auflösung der Spannung
double Auflosung = 0; //anzahl der steps bei 12bit bittiefe
int ADC_Conversion = 0; //variable die anzeigt ob akkuspannung(1) oder tempspannung(2) gemessen wird; gebraucht für interrupt

//AKKU
double U_ADC_Akku = 0; //ADC SPANNUNG AKKU
int Low_Akku = 0; //1 wenn akku zu leer ist

//Temperatur
double U_ADC_T = 0;  //ADC SPANNUNG Temp
const double TR = 298.15; //Bezugstemperatur 25°Celsius
const double B = 4064; //  Thermistorkonstante B25/75=4064kelvin ,, B25/85=4073k
const double R25 = 131742; //Widerstand bei 25°C // eigentlich 150k, durch kalibrieren und messen errechnet
const double R2 = 150000; //festwiderstand R2
double RT = 0; //Berechneter Widerstand
double T = 0; //berechnete Temperatur
double UT = 0; //gemessene TemeraturSpannung

//Umwandlung Temperatur zu String
char Temp[6];
int Tint = 0;

//SMS
int SMS_aktion = 0; //1-4 welcher text wird gesendet 1:temp 2:kurzschluss 3:leerlauf 4:akku leer
char SMS_1[] = { "T = 00.0 C" };  //Buggy um T zu übertragen
//char SMS_2[]={"short"}; //falls kurzschluss fehler gemessen wurde
// char SMS_3[]={"open"}; //falls leerlauf fehler gemessen wurde
char SMS_4[] = { "LOWB" }; //falls baterie leer gemessen wurde
char AT[] = { "AT" }; //vor jedem Befehl an das GSM modul muss ein AT
char AT_PIN[] = { "+CPIN=" }; //Netzwerkverbindung durch Pinübergabe
char PIN[] = { "0051" }; //tatsächliche PIN
char AT_CMGF[] = { "+CMGF=1" }; //setzt das Modul in den richtigen SMS-Text mode
char AT_NUMBER[] = { "+CMGS=\"015731251716\"" }; //telefonnummer und übergabe befehl
char AT_GPS_MODE[] = { "+CGDCONT=1" }; // +CGPS=1 GPS session will choose hot or cold automatically.
char AT_GPS[] = { "+CGATT=1" }; //+CGPSINFO
char AT_GPS1[] = { "+CGACT=1" };

char AT_FUN[] = { "+CFUN=1" };
char AT_CSQ[] = { "+CSQ" };
char AT_COPS[] = { "+COPS?" };
char AT_CPSI[] = { "+CPSI?" };
char AT_CGATT[] = { "+CGATT=1" };
char AT_CGDCONT[] =
        { "+CGDCONT=1,\"IP\",\"pinternet.interkom.de\",\"0.0.0.0\"" };
char AT_CGSOCKCONT[] = { "+CGSOCKCONT=1,\"IP\",\"pinternet.interkom.de\"" };
char AT_CSOCKSETPN[] = { "+CSOCKSETPN=1" };
char AT_CHTTPSSTART[] = { "+CHTTPSSTART" };
char AT_CHTTPSOPSE[] = { "+CHTTPSOPSE=\"api.thingspeak.com\",80,1" };
char AT_CHTTPSSEND[] = { "+CHTTPSSEND=160" };
char AT_CHTTP_CONTENT0[] = {
        "GET /update?api_key=0QRD0JDOTMUI5JIB&field1=50.75941444&field2=6.08316607\r\n" };
char AT_CHTTP_CONTENT1[] = { "Host: api.thingspeak.com:80\r\n" };
char AT_CHTTP_CONTENT2[] = { "User-Agent: curl/7.45.0\r\n" };
char AT_CHTTP_CONTENT3[] = { "Content-Length: 0\r\n\r\n" };
char AT_CHTTPSSTOP[] = { "+CHTTPSSTOP" };

char GPS_WEAK[] =
        { "Die GPS-Verbindung ist nicht stark genug, bitte aendern Sie den Standort des Objekts." };
char TS_CM[30];
// char AT_IPR[]={"+IPR=57600"}; //einstellung baudrate
// char AT_IPR2[]={"+IPR?"}; //abfrage baudrate
int CTRLZ = 26; //CTRL Z -> 26 ->0x1A hinter dem zu sendenden text muss ein ctrlz befehl gesendet werden
char AT_TEXT[107]; //Buggy welcher an das GSM modul übergeben wird. wird in entsprechender FUnktion mit dem richtigen SMStext gefüllt
char reciveString[45];
char Enter[] = { "\r" }; //CARRIAGE RETURN \r, hinter jedem Befehl muss ein carriage return gesendet werden

char Recieve_Test[20]; //recieve Buffer um im test fall die antwort auszulesen
int Recieve_Index = 0; //wird incrementiert wenn ein byte recieved wurde. dadurch wird recieve vorgang irgendwann (der funktion entsprechend) abgebrochen

char command[15];

//watchdog timer B0
int GSM_Frozen = 0; //0-> alles normal   1->frozen
int CMGF_Check = 0; //für uart interrupt um zu erkennen das CMGF befehl aktiv ist und die variable im funktionierenden Falle auf 1 gesetzt wird

//POWER STATUS
int Power_On = 0; // 0 wenn modul ausgeschaltet ist

//Network Status
int T_Capture = 0; //variable für erste messung / zweite wird direkt aus dem capture register gelesen
int Capture_Index = 0; //index um 2 captures durchzuführen und dann abzubrechen
int Capture_Time = 0; //variable in der die Zeit zwischen den Captures steht
int Net_Stat = 0; //1 wenn verbindung herscht

//_______________________________________________________________________________________________________________________________________
//PORT SETUP + INIT UNUSED PORTS für STROMSPAREN
//_______________________________________________________________________________________________________________________________________
void Port_Setup(void)
{
    //PORT A -> P1 und P2
    P1DIR = 0;
    P2DIR = 0;
    P1SEL = 0;
    P2SEL = 0;
    P1DS = 0;
    P2DS = 0;
    P1OUT = 0;
    P2OUT = 0;
    P1REN = 0b11110001;
    P2REN = 0b11111111;
    //LED
//     P1DIR |= BIT4;    //Pin 1.0 als Output festlegen im Datenrichtungsregister, der Rest Input
//     P1OUT &= ~BIT4;
    //KEY
    P1DIR |= BIT1; //1.1 Output
    P1OUT |= BIT1; //set to 1 (normaler zustand)
    //Power Status
    P1DIR &= ~BIT2;       //1.2 INPUT
    P1IES &= ~BIT2;                           // P1.2 Lo to hi edge
    P1IFG &= ~BIT2;                           // P1.2 IFG cleared
    //NetworkStatus
    P1DIR &= ~BIT3; //1.3 Input
    P1SEL |= BIT3; //Select peripheral funktion

    //PORT B -> P3 und P4
    P3DIR = 0;
    P4DIR = 0;
    P3OUT = 0;
    P4OUT = 0;
    P3SEL = 0;
    P3DS = 0;
    P4SEL = 0;
    P4DS = 0;
    P3REN = 0b11100111;
    P4REN = 0b11111111;
    //UART
    P3DIR |= BIT3; //TX 3.3 OUTPUT
    P3DIR &= ~BIT4; //RXist input
    P3OUT &= ~BIT3; //Clear output
    P3SEL |= (BIT3 + BIT4); //use peripheral funktion

    //PORT C -> P5 und P6  XT2 IN / OUT + ADC
    P5SEL = 0;
    P6SEL = 0;
    P5DS = 0;
    P6DS = 0;
    P5REN = 0b11110011;
    P5DIR = 0;
    P5SEL |= (BIT2 + BIT3); //Bit 3 ist eigentlich dont care
    P6DIR = 0;
    P5OUT = 0;
    P6OUT = 0;
    P6REN = 0b11111100;
    //ADC
    P6DIR &= ~(BIT0 + BIT1); //6.0 und 6.1 INPUT
    P6SEL |= (BIT0 + BIT1); //Perpipheral Functions selected ADC (A0, A1)

    //PORT D -> P7 und P8
    P7DIR = 0;
    P8DIR = 0;
    P7OUT = 0;
    P8OUT = 0;
    P7DS = 0;
    P8DS = 0;
    P7REN = 0b11111111;
    P8REN = 0b11111111;
}
//_______________________________________________________________________________________________________________________________________
//TIMER B0 SETUP
//**Wird hauptsächlich für die Warteroutine verwendet (wait();)
// CLocksource ACLK - UP-Mode (wird erst in der Fkt aktiviert) - Compare-Mode - 16 bit Counter TB0
//_______________________________________________________________________________________________________________________________________
void TimerB0_Setup(void)
{
    //Timer Setup
    //Setzen der Bits mit Masken ausmsp430.h und dann einem bitwise or
    TB0CTL |= TBCLR;    //Clear der Timersettings
    TB0CTL |= TBSSEL__ACLK; //ACLK als Clocksource von Timer A
    TB0CCTL0 &= ~CCIFG; //clear CCIFG Flag
    TB0CTL |= MC__STOP; //Timer in Stopmode
    TB0CCTL0 &= ~CAP; //Timer in Compare Mode (nach clear eh auf 0)
}
//_______________________________________________________________________________________________________________________________________
//TIMER A0 SETUP
//**Wird nur für das Messen des NS-Intervalls (Periodenzeit) im Capture-Mode verwendet
// Clocksource ACLK - Continuous-Mode (wird erst in Fkt aktiviert) - Capture-Mode - 16 bit
//_______________________________________________________________________________________________________________________________________
void TimerA0_Setup(void)
{
    //TA0 CCR2
    TA0CTL |= BIT2;    //Clear der Timersettings
    TA0CCTL2 &= ~BIT8; // disabel capture mode for setup
    TA0CTL |= BIT8; //ACLK als Clocksource von Timer A
    TA0CCTL2 &= ~CCIFG; //clear CCIFG Flag
    TA0CTL |= MC__STOP; //timer läuft noch nicht / wird in funktion aktiviert
    TA0CCTL2 |= (BITF + BITE); //capture on rising and falling edge
    TA0CCTL2 |= BIT8; //capture mode aktiv
}
//_______________________________________________________________________________________________________________________________________
//ADC SETUP
// Tsample aus Datenblatt muss größer 18 us sein. Mittels CLockdivider und Clockcycles per Conversion werden 32 us eingestellt
// SMCLK als Clocksource (4Mhz) -> Clockdiv f/4 -> 1Mhz -> 32Samples per Conversion
// Strom sparen: internal Tempsensor aus und ADC erst ON wenn er benutzt wird
// Auflösung berechnen um später die Spannung in der Einheit Volt berechnen zu können
//_______________________________________________________________________________________________________________________________________
void ADC_Setup(void)
{
    ADC12CTL0 &= ~BIT1; //disable conversion um setup durchführen zu können
    ADC12CTL0 |= BIT8 + BIT9; //32 clockcycles für conversion /
    ADC12CTL1 |= BIT9 + BIT5 + BIT6; //Sampcon source from internal timer + clk div=4
    ADC12CTL1 |= BIT4 + BIT3; //Clocksource = SMCLK
    ADC12CTL2 |= BIT7; //turn intern temperature sensor off
    Auflosung = pow(2, bit) - 1;
    vStep = U0 / Auflosung;
}
//_______________________________________________________________________________________________________________________________________
// Clock Setup
// MCLK + SMCLK = 4Mhz XT2 Kristall
// ACLK = REFO (war vorher VLO, aber hat zu Fehlern geführt, sobald die Software auf einen zweiten Controller geflasht wurde, da der VLO eine SPezifikation von +-50% hat)
// Vor Abschluss muss gewartet werden bis die Fehlerflags gelöscht sind, da es sonst dazu kommen kann, dass das SMCLK auf eine andere Clocksource umgestellt wird (DCOCLKDIV)
// ACLK wird mittels Clockdivider geteilt auf f/32
// XT1 wird komplett disabled um Strom zu sparen
//_______________________________________________________________________________________________________________________________________
void Clock_Setup(void)
{
    //UCSCTL6 &= ~(BITC+BITE+BITF); //ENABLE XT2 / adjust drive strength
    UCSCTL3 |= SELREF_2; //FLLref = REFO damit xt1 nicht für fehler sorgt
    UCSCTL6 |= XT1OFF; //disable xt1
    UCSCTL6 &= ~XT2OFF; //enable xt2
//    UCSCTL4 |= SELS_3+SELM_3; //ACLK-VLO, SMCLK-DCO, MCLK-DCO

    //ACLK Setup (VLO als source)
    UCSCTL4 = SELA_2; //ACLK SOURCE SELECT BIT10-8-> 001=VLOCLK
    UCSCTL5 = DIVA_5;   //ACLK SOURCE DIVIDER f/32
    UCSCTL4 |= (BIT4 + BIT6); //clocksource for smclk = 4mhz crystal
    UCSCTL4 |= (BIT0 + BIT2); //clocksource for mclk = 4 mhz crystal

    //Loop bis xt2 stabil ist und alle flags clearen
    do
    {
        UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);  // Clear alle fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear haupt fault flag
    }
    while (SFRIFG1 & OFIFG);                   // oscillator fault flag testen
    UCSCTL6 &= ~(BITE + BITF);                  // xt2 drive für 4mhz einstellen
    UCSCTL4 |= SELS_5 | SELM_5;               // SMCLK=MCLK=XT2
    UCSCTL4 |= SELA_0;
}
//_______________________________________________________________________________________________________________________________________
// UART
//_______________________________________________________________________________________________________________________________________
void UART_Setup(void)
{
    UCA0CTL1 |= BIT0; //software Reset enable
    UCA0CTL0 &= ~(BIT7 + BIT5 + BIT4 + BIT3 + BIT0); //7no parity, 5lsb, 4 8bit,3 one stop bit, 0asyncronous mode
    UCA0CTL0 &= ~(BIT2 + BIT1); //uart mode
    UCA0CTL1 |= (BIT7 + BIT6); //clocksource smclk
    UCA0MCTL &= ~BIT0; //no oversampling mode
//prescaler für baud rate
//einstellung für 115200 -> BR=34 BRS=6 -> BR=BR0+256*BR1->BR1=0
    UCA0BR0 |= (BIT5 + BIT1); //     00100010 //low byte of 16 val BR-> 34  /416->10100000
    UCA0BR1 = 0;
    UCA0MCTL |= (BIT3 + BIT2); //BRS=6
    UCA0CTL1 &= ~BIT0; //clear software reset enable
}
//_______________________________________________________________________________________________________________________________________
//TEMPERATUR MESSEN UND IN STRING UMWANDELN
// Temperaturspannung liegt an KANAL A0 an
//_______________________________________________________________________________________________________________________________________
void Temp_Messung(void)
{
    //ADC NTC-Spannungsabfrage und Berechnung Temperatur
    ADC_Conversion = 2;
    ADC12MCTL0 &= ~(BIT0 + BIT1 + BIT2 + BIT3); // A0 input Channel
    //Interrupt Enable + ADC ON
    ADC12IE |= BIT0; //Interrupt Enable
    __enable_interrupt();
    ADC12CTL0 |= BIT4; //ADC ON
    ADC12CTL0 |= BIT1 + BIT0; //ENABLE + START CONVERSION
    //Enter LPM bis ADC fertig convertiert
    __bis_SR_register(LPM3_bits);
    //Disable global interrupt
    ADC12IE &= ~BIT0; //Interrupt disnable
    __disable_interrupt();
    //sicherstellen das flag gelöscht ist
    ADC12IFG &= ~ADC12IFG0;

    //Berechnung von RT und daraus T in Celsius
//    Auflosung=pow(2, bit)-1;
//    vStep=U0/Auflosung;
    UT = U_ADC_T * vStep;
    SMS_aktion = 1;
//    if (UT>=2.8){
//        SMS_aktion=3;
//    }
//    if (UT<=0.2){
//        SMS_aktion=2;
//    }
    RT = ((U0 * R2) / UT) - R2;
    T = (TR / (1 - (TR / B) * log(R25 / RT))) - 273.15;

    //T IN CHAR ARRAY UMWANDELN
    Tint = (int) (T * 10);
    sprintf(Temp, "%d", Tint);

}
//_______________________________________________________________________________________________________________________________________
// AKKU_SPANNUNG MESSEN UND BEDINGUNGEN CHECKEN
//_______________________________________________________________________________________________________________________________________
void Akku_Messung(void)
{
    //ADC Akku-Spannung abfrage und berechnung
    ADC_Conversion = 1;
    ADC12MCTL0 |= BIT0; // A1 input Channel
    //Interrupt Enable + ADC ON
    ADC12IE |= BIT0; //Interrupt Enable
    __enable_interrupt();
    ADC12CTL0 |= BIT4; //ADC ON
    ADC12CTL0 |= BIT1 + BIT0; //ENABLE + START CONVERSION
    //Enter LPM bis ADC fertig convertiert

    __bis_SR_register(LPM3_bits);
    //Disable global interrupt
    ADC12IE &= ~BIT0;      //Interrupt disable
    __disable_interrupt();

//sicherstellen das flag gelöscht ist
    ADC12IFG &= ~ADC12IFG0;

    //Berechnung der halben Akkuspannung und Setzen der Low-Akku / SMS_Aktion variablen falls akkuspannung zu gering
    U_ADC_Akku = U_ADC_Akku * vStep;
    if (U_ADC_Akku < 1.65)
    {
        Low_Akku = 1;
        SMS_aktion = 4;
    }
}

//_______________________________________________________________________________________________________________________________________
//WAIT FUNCTION
//wartezeit wird in counts an funktion übergeben
// timer B0 wird im comparemode dazu verwendet die Wartezeit zu zählen
// während dessen wird xt2 abgeschaltet und für eine winzige dauer der refo als source für mclk und smclk verwendet
//_______________________________________________________________________________________________________________________________________
void wait(double wait_time, int Task)
{
    //Comparevalue von x sekunden zuweisen; 800ms->N=470
    TimerB0_Task = Task; //bestimmung welche interruptroutine durchfegührt wird
    TB0R = 0; //clear counter register
    TB0CTL |= MC__UP; //upmode, timer beginnt zu zählen
    TB0CCR0 = TB0R + wait_time; // wartezeit
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht

    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    __enable_interrupt(); //enable global maskable interrupts (nur locale können aber tatsächlich feuern)

    UCSCTL4 |= SELS_2 | SELM_2; // XT2 als source trennen //für kurze zeit ist refo die quelle für mclk/smclk
    UCSCTL4 &= ~(BIT0 + BIT2 + BIT4 + BIT6);
    UCSCTL6 |= XT2OFF; //disable xt2

    __bis_SR_register(LPM3_bits); // warten in lpm3 bis comparevalue erreicht ist
    TB0CCTL0 &= ~CCIE; //interrupt disable
    __disable_interrupt();

    //sicherstellen das flag gelöscht ist und timer gestoppt ist sodass kein falscher interrupt ausgelöst wird
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

    UCSCTL6 &= ~XT2OFF; //enable xt2
    do
    {
        UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);  // Clear alle fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear haupt fault flag
    }
    while (SFRIFG1 & OFIFG);                   // oscillator fault flag testen
    UCSCTL6 &= ~(BITE + BITF);                  // xt2 drive für 4mhz einstellen
    UCSCTL4 |= SELS_5 | SELM_5;               // SMCLK=MCLK=XT2
    UCSCTL4 &= ~(BIT1 + BIT5);

}
//_______________________________________________________________________________________________________________________________________
//EIN UND AUSSCHALTEN DES GSM_MODULS
//_______________________________________________________________________________________________________________________________________

void Turn_On_GSM(void)
{
    P1OUT &= ~BIT1;  //pull Port1 pin 2 Down
    wait(205, 1); // warte 200 ms, übergabe timerb0_task=1, sodass nach warteschleife der port1 pin 2 wieder high gesetzt wird
}
void Turn_Off_GSM(void)
{

    P1OUT &= ~BIT1;  //pull Port1 pin 2 Down
    wait(716, 1); //0,6 sek
}
//_______________________________________________________________________________________________________________________________________
//CHECK IF GSM IS POWERED ON
//funktion in der in LPM3 gewartet wird, bis eine Flanke in Key ein Interrupt auslöst und anzeigt, dass das GSM-Modul erfolgreich hochgefahren ist
//um sicher zu gehen, dass sich das Modul hier nicht aufhängt ist ein WD Timer aktiv der das Warten nach 10 sek abbricht
//_______________________________________________________________________________________________________________________________________
void PWR_status(void)
{
    P1IES &= ~BIT2;
    TimerB0_Task = 5;
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 10240; //ca 10 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    P1IE |= BIT2;                             // P1.2 interrupt enabled
    __enable_interrupt(); //enable global maskable interrupts (nur locale können aber tatsächlich feuern)
    __bis_SR_register(LPM3_bits);
    TB0CCTL0 &= ~CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    P1IE &= ~BIT2;                             // P1.2 interrupt disabled
    __disable_interrupt();

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;
    P1IFG &= ~BIT2;                         // Clear P1.2 IFG
}
//_______________________________________________________________________________________________________________________________________
// TRANSMIT PIN
//Pin übermitteln und damit das GSM-Modul auffordern eine Netzwerkverbindung herzustellen
//Damit sich das Modul beimw Warten auf eine Antwort nicht aufhängt ist hier ein WD timer aktiv
//_______________________________________________________________________________________________________________________________________
void TransmitPIN(void)
{
    Recieve_Index = 0;

    //Senden: Einzelne Strings durchgehen. Nach senden eines Chars warten bis das Flag 1 ist
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_PIN[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_PIN[i];
    }
    for (i = 0; PIN[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = PIN[i];
    }
    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

// Warten auf die Antwort: Entweder triggert der Watchdog nach 3 Sekunden oder die UART Antwort kommt erfolgreich zurück. Wenn WD triggert, wird danach das Modul direkt in 60 sek Ruhemodus versetzt
    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;
    __disable_interrupt();
}
//_______________________________________________________________________________________________________________________________________
// überprüft / wartet bis netzwerkverbindung hergestellt ist
// an 2 flanken wird das CR des TA0 gecaptured und anschließend die Periodendauer errechnet und geprüft das sie über ca. 300ms liegt. Dass sich das Modul nicht beim Warten auf die erste Flanke aufhängt ist dort der WD-Timer erneut aktiv
// NETZWERKVERBINDUNG = Periodendauer = 1600ms / GPRSVerbindung = Periodendauer = 400ms
//_______________________________________________________________________________________________________________________________________
void Check_Network(void)
{
    //Watchdog timer der das warten auf den ersten capture interrupt abbricht nach 15 sekunden
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 15360; //15sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 3;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    Net_Stat = 0;
    TA0R = 0;
    TA0CCTL2 &= ~CCIFG;
    Capture_Index = 1;
    TA0CTL |= BIT5; //continuos mode start
    TA0CCTL2 |= BIT4; //capture Intterrupt enable
    __enable_interrupt();
    __bis_SR_register(LPM3_bits); //warten bis erster Capture interrupt ausgelöst wird

    //Hier hat entweder TA oder TB interrupt ausgelöst, deshalb kann TB0 gestoppt werden

    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    TB0CCTL0 &= ~CCIFG; //Clear Flag
    TB0CTL |= MC__STOP;
    TA0CCTL2 &= ~CCIFG;

    //Wenn TA interrupt triggert, wird task auf 0 gesetzt, wenn also task=0 muss zweiter capture ausgeführt werden
    if (TimerB0_Task == 0)
    {
        Capture_Index++;
        __bis_SR_register(LPM3_bits); //warten bis zweiter Capture interrupt ausgelöst wird
        __disable_interrupt();
        TA0CCTL2 &= ~BIT4; //capture Intterrupt disnable
        Capture_Time = TA0R - T_Capture;

        //Prüfen ob halbe periodendauer größer 300ms
        if (Capture_Time >= 307)
        {
            Net_Stat = 1;
        }
    }
    else
    {
        __disable_interrupt();
        TA0CCTL2 &= ~BIT4; //capture Intterrupt disnable
    }

//Sicherstellen dass Flag gelöscht und Timer gestoppt
    TA0CCTL2 &= ~CCIFG;
    TA0CTL |= MC__STOP; //timer stop
    TA0R = 0;

}
//_______________________________________________________________________________________________________________________________________
// ENTER SMS MODE
//_______________________________________________________________________________________________________________________________________
void TransmitCMGF(void)
{
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CMGF[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CMGF[i];
    }
    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht

    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

//sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}
//_______________________________________________________________________________________________________________________________________
// TransmitNumber
//Nummer übertragen-> wenn GSMMODUL antwortet kann danach der Text übertragen werden
//_______________________________________________________________________________________________________________________________________
void TransmitNumber(void)
{
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_NUMBER[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_NUMBER[i];
    }
    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

//sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void activateGPS(void)
{
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_GPS_MODE[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_GPS1[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void activateGPS1(void)
{
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_GPS[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_GPS[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void getGPSCoordinates(void)
{
    Recieve_Index = 0;
    unsigned int i = 0;
    int trigger; // This trigger will be used to detect empty replies from the module. The "empty" GPS coordinates usually look like this: ",,,,,,,"
//do
    //{
    //   trigger = 0; // We initialize the trigger with 0 AT_GPS_MODE

    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_GPS1[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_GPS_MODE[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }
    i = 0;
    do
    {
        while (( UCA0IFG & UCRXIFG) == 0)
            ;
        {
        } // Warte auf Zeichen
        reciveString[i] = UCA0RXBUF;        // Zeichen einlesen
        i++;                              // Index inkrementieren
    }
    while (i < sizeof(reciveString)); //&& reciveString[i - 1] != '\r'
    i--;
    reciveString[i] = '\0';
    for (i = 0; i < sizeof(reciveString); i++)
    {
        if (reciveString[i] == ',' && reciveString[i + 1] == ',')
        { // When two consecutive commas are detected,
            trigger = 1; // the trigger changes to 1 so that the AT command is repeated.
        }
//       if(trigger == 1) //Wenn der Wert 1 ist, bedeutet dies, dass die GPS-Koordinaten leer sind (,,,,,) und für uns nicht von Nutzen sind.
//       {
//           for (i=0; i < sizeof( GPS_WEAK ); i++){
//           reciveString[i] = GPS_WEAK[i];}    //Die Meldung wird durch eine Fehlermeldung ersetzt, die den Benutzer darüber informiert,
//       }                                      // dass sein Objekt kein ausreichend starkes Signal empfängt, um die Koordinaten zu senden.
    } //Dies bedeutet in der Regel, dass sich das Objekt in einem Gebäude befindet.
      //}
      //   while(trigger!=0);

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

//sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}
//&field1=

//_______________________________________________________________________________________________________________________________________
// TransmitTEXT
//_______________________________________________________________________________________________________________________________________
void TransmitText(void)
{
    Recieve_Index = 0;

    unsigned int i = 0;
    for (i = 0; AT_TEXT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_TEXT[i];
    }
    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }
    while (!(UCA0IFG & UCTXIFG))
    {
    };
    UCA0TXBUF = CTRLZ;
    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}
//_______________________________________________________________________________________________________________________________________
// Fill text
//_______________________________________________________________________________________________________________________________________
//FESTLEGEN WELCHE SMS GESENDET WIRD
void Fill_SMS(void)
{
    if (Low_Akku != 1)
    {  //wenn der akku noch genug geladen ist
        unsigned int p = 0;
        unsigned int h = 4; //index um Temp char string in den TExt string zu kopieren / temperatur beginnt im AT_TEXT an 4. stelle
        unsigned int k = 0;
        switch (SMS_aktion)
        {
        case 1: //Voller Akku / temperatur übertragen

            //Einfügen T = 00.00 C in den via AT-Command versendeten TExt
            for (p = 0; p < (sizeof(SMS_1)) - 1; p++)
            {
                AT_TEXT[p] = SMS_1[p];
            }
            if (Tint < 10)
            { //wenn T integer größer zehn ist soll erst ab 5. stelle geschrieben werden
                h++;
            }

            //Einfügen der Temperatur z.b. 2432 => 24.32
            while (h <= 7)
            {
                AT_TEXT[h] = Temp[k];
                if (h == 5)
                {
                    h = h + 2; // komma überspringen
                }
                else
                {
                    h = h + 1;
                }
                k++;
            }
            //for (p=0; p<(sizeof(Enter))-1; p++){
            //    AT_TEXT[p] = Enter[p];
            // }

            break;
//     case 2:  //Voller Akku / kurzschluss
//         for (p=0; p<(sizeof(SMS_2))-1; p++){
//             AT_TEXT[p]=SMS_2[p];}
//         break;
//     case 3: //Voller Akku / leerlauf
//           for (p=0; p<(sizeof(SMS_3))-1; p++){
//              AT_TEXT[p]=SMS_3[p];}
//           break;
        }
    }
    //Akku ist leer
    if (Low_Akku == 1)
    {
        unsigned int i = 0;
        for (i = 0; i < (sizeof(SMS_4)) - 1; i++)
        {
            AT_TEXT[i] = SMS_4[i];
        }
    }
}

void Fill_SMS_GPS(void)
{
    unsigned int p;
    char longitude[] = "50.75941444";
        char latitude[] = "6.08316607";
        char gpsText[] =
                "Ihr Geraet befindet sich derzeit an folgender Stelle:\nhttps://www.google.com/maps?q=";
        char finalText[107];

        //https://www.google.com/maps?q=50.75941444,6.08316607
        for (p = 0; p < (sizeof(gpsText)) - 1; p++)
        {
            finalText[p] = gpsText[p]; //Adding the first section of the message.
        }

        int k = 0; //Counter for longitude
        for (p = sizeof(gpsText)-1; p < (sizeof(gpsText)+sizeof(longitude)) - 1; p++)
        {
            finalText[p] = longitude[k];    //Adding the longitude.
            k++;
        }
        char comma [] = ",";
        finalText[95] = comma[0]; //Adding the comma between longitude and latitude.

        k = 0; //Counter for latitude
            for (p = sizeof(gpsText)+sizeof(longitude)-1; p < (sizeof(gpsText)+sizeof(longitude)+sizeof(latitude)) - 1; p++)
            {
                finalText[p] = latitude[k];
                k++;
            }

        for (p = 0; p < (sizeof(finalText)) - 1; p++)
                    {
                        AT_TEXT[p] = finalText[p];
                    }
}
//_______________________________________________________________________________________________________________________________________
// ShutDown funktion
//_______________________________________________________________________________________________________________________________________
void ShutDown(void)
{        //GSM MODUL ABSCHALTEN

//Abschalten des gsm-moduls und warten bis der port-interrupt (Flanke im Signal PS)  anzeigt, dass das GSM-Modul aus ist; watchdog aktiv um sicherzustellen, dass sich das Programm hier nicht aufhängt
    while (Power_On == 1)
    {
        Turn_Off_GSM();

        P1IES |= BIT2; //trigger interrupt on high to low edge
        TB0R = 0;
        TimerB0_Task = 5;
        TB0CTL |= MC__UP;
        TB0CCR0 = TB0R + 10240; //ca 10 sek
        TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
        TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
        P1IE |= BIT2;                             // P1.2 interrupt enabled
        __enable_interrupt(); //enable global maskable interrupts (nur locale können aber tatsächlich feuern)
        __bis_SR_register(LPM3_bits);
        __disable_interrupt();
        TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
        P1IE &= ~BIT2;                             // P1.2 interrupt disabled
        P1IES &= ~BIT2; //rücksetzen damit einschalt check wieder funktioniert

        //sicherstellen das alle flags gelöscht sind
        P1IFG &= ~BIT2;   // Clear P1.2 IFG
        TB0CTL |= MC__STOP;
        TB0CCTL0 &= ~CCIFG;
    }
    Power_On = 0;

    //Enter LPM4.5 WENN AKKU LEER IST / AUFWACHEN DURCH RESET
    if (Low_Akku == 1 || SMS_aktion > 1)
    {

        UCSCTL4 |= SELS_2 | SELM_2; // XT2 als source trennen //für kurze zeit ist refo die quelle für mclk/smclk
        UCSCTL4 &= ~(BIT0 + BIT2 + BIT4 + BIT6);
        UCSCTL6 |= XT2OFF; //disable xt2
        PMMCTL0_H = PMMPW_H; // open PMM
        PMMCTL0_L |= PMMREGOFF; // set Flag to enter LPM4.5 with LPM4 request
        __bis_SR_register(LPM4_bits);
        __no_operation();
    }

    //Enter 60 sek LPM3 WENN AKKU NOCH GENUG GELADEN IST
    else
    {
        //bedingende Variablen zurücksetzen
        TimerB0_Task = 0;
        ADC_Conversion = 0;
        T = 0;
        Tint = 0;
        Net_Stat = 0;

        int i = 0;
        //SMS text String clearen
        for (i = 0; i < sizeof(AT_TEXT); i++)
        {
            AT_TEXT[i] = 0;
        }
        //rücksetzen Temperatur string
        for (i = 4; i <= 7; i++)
        {
            if (i != 6)
            {  //beachten dass komma bestehen bleibt
                Temp[i] = 0;
            }
        }
        wait(61440, 0);
    }
}

void executeCommand(char c[])
{
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; c[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = c[i];
    }
    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    i = 0;
    do
    {
        while (( UCA0IFG & UCRXIFG) == 0)
            ;
        {
        } // Warte auf Zeichen
        reciveString[i] = UCA0RXBUF;        // Zeichen einlesen
        i++;                              // Index inkrementieren
    }
    while (i < sizeof(reciveString)); //&& reciveString[i - 1] != '\r'
    i--;
    reciveString[i] = '\0';

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;
}
void sendDataToCloud(void)
{
    // char data[] = {"%.6f&field1=%.6f",field1,field2};
    // char TS_DATA[] = strcat(TS_9,data);
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT_CHTTP_CONTENT0[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTP_CONTENT0[i];
    }

    for (i = 0; AT_CHTTP_CONTENT1[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTP_CONTENT1[i];
    }

    for (i = 0; AT_CHTTP_CONTENT2[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTP_CONTENT2[i];
    }

    for (i = 0; AT_CHTTP_CONTENT3[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTP_CONTENT3[i];
    }
    /*
     for (i = 0; Enter[i] != 0; i++)
     {
     while (!(UCA0IFG & UCTXIFG))
     {
     };
     UCA0TXBUF = Enter[i];
     }*/
    /*
     i = 0;
     do
     {
     while (( UCA0IFG & UCRXIFG) == 0)
     ; // Warte auf Zeichen
     if (UCA0RXBUF == "\n")
     {
     i++;
     continue;
     }
     reciveString[i] = UCA0RXBUF;
     // Zeichen einlesen
     i++;                              // Index inkrementieren
     if (i > 70)
     break;
     }
     while (i < sizeof(reciveString) && reciveString[i - 1] != '\r');
     i--;
     reciveString[i] = '\0'; */

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072;//ca 3 sek
    TB0CCTL0 &= ~CCIFG;//sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE;//LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE;//enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE;//disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void httpSendRequest(void)
{
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }

    for (i = 0; AT_CHTTPSSEND[i] != 0; i++)
    { //Contains lenght of message
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTPSSEND[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;
}

// Sets the level of functionality in the MT. Level "full functionality" is where the highest level of power is drawn.
void setFunctionality(void)
{
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_FUN[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_FUN[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void checkSignalStrength(void)
{ //  Returns the signal strength of the device.
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CSQ[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CSQ[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void findNetwork(void)
{ // Find the available networks
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_COPS[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_COPS[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void ueInfo(void)
{ // Used to return UE system information.
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CPSI[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CPSI[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void attachDevice(void)
{ // Used to attach or detach the device to packet domain service.
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CGATT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CGATT[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void setPDP(void)
{ // Sets the PDP context parameters such as PDP type (IP, IPV6, PPP, X.25 etc), APN, data compression, header compression etc.
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CGDCONT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CGDCONT[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void openTCP(void)
{ // Opens a transparent TCP connection
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CGSOCKCONT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CGSOCKCONT[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void stopHTTP(void)
{ // Opens a transparent TCP connection
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CHTTPSSTOP[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTPSSTOP[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void setPDPProfile(void)
{ // Sets active PDP context’s profile number. Values can range from 1 to 16
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CSOCKSETPN[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CSOCKSETPN[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void startHTTP(void)
{
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CHTTPSSTART[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTPSSTART[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void connectToServer(void)
{ // Used to connect to the HTTPS server after acquiring the HTTPS stack.
    Recieve_Index = 0;
    unsigned int i = 0;
    for (i = 0; AT[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT[i];
    }
    for (i = 0; AT_CHTTPSOPSE[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = AT_CHTTPSOPSE[i];
    }

    for (i = 0; Enter[i] != 0; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
        {
        };
        UCA0TXBUF = Enter[i];
    }

    //i = 0;
    //         do
    //           {
    //            while( ( UCA0IFG & UCRXIFG ) == 0 );{} // Warte auf Zeichen
    //               reciveString[i] = UCA0RXBUF;        // Zeichen einlesen
    //           i++;                              // Index inkrementieren
    //           }
    //          while( i < sizeof( reciveString )); //&& reciveString[i - 1] != '\r'
    //         i--;
    //        reciveString[i] = '\0';

    GSM_Frozen = 2; //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
    TB0R = 0;
    TB0CTL |= MC__UP;
    TB0CCR0 = TB0R + 3072; //ca 3 sek
    TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
    TimerB0_Task = 2;
    TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
    UCA0IE |= UCRXIE; //enable uart interrupt
    __enable_interrupt();
    while (GSM_Frozen == 2)
    { //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
    };
    __disable_interrupt();
    TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
    UCA0IE &= ~UCRXIE; //disable uart interrupt

    //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
    UCA0IFG &= ~ UCRXIFG;
    TB0CTL |= MC__STOP;
    TB0CCTL0 &= ~CCIFG;

}

void checkGSMResponse(void)
{
    unsigned int i = 0;
    i = 0;

    char conname[] = { "AT+CHTTPSSTART" };
    do
    {
        while (( UCA0IFG & UCRXIFG) == 0)
            ;
        {
        } // Warte auf Zeichen
        reciveString[i] = UCA0RXBUF;        // Zeichen einlesen
        i++;                              // Index inkrementieren
    }
    while (i < sizeof(reciveString)); //&& reciveString[i - 1] != '\r'
    i--;
    reciveString[i] = '\0';

    int index = 0;
    int j;
    for (j = 0; j < sizeof(reciveString); j++)
    {
        if (reciveString[j] == conname[0] && reciveString[j + 1] == conname[1]
                && reciveString[j + 2] == conname[2]
                && reciveString[j + 3] == conname[3])
        {
            index = j;
        }
    }

    index = index + sizeof(reciveString);
    int k = 0;

    for (j = index; j < sizeof(reciveString); j++)
    {
        AT_TEXT[j] = reciveString[j];
        k++;
    }
    /*

     GSM_Frozen=2;  //wenn alles normal funktioniert setzt der UART-Interrupt die variable auf 0. wenn gsm einfriert setzt watchdog timer die variable auf 1
     TB0R=0;
     TB0CTL |= MC__UP;
     TB0CCR0 =TB0R+3072; //ca 3 sek
     TB0CCTL0 &= ~CCIFG; //sollte aus irgendeinem grund das flag 1 sein, wird es hier sicherheitshalber gelöscht
     TimerB0_Task=2;
     TB0CCTL0 |= CCIE; //LOCAL! enable compare interrupt request of the CCIFG flag
     UCA0IE |= UCRXIE; //enable uart interrupt
     __enable_interrupt();
     while(GSM_Frozen==2){ //warteschleife bis entweder die uartantwort kommt oder der watchdog timer triggert / uartanwort stoppt den watchdog timer
     };
     __disable_interrupt();
     TB0CCTL0 &= ~CCIE; //LOCAL! disable compare interrupt request of the CCIFG flag
     UCA0IE &= ~UCRXIE; //disable uart interrupt

     //sicherstellen das alle flags gelöscht sind und timer b0 gestoppt ist
     UCA0IFG &=~ UCRXIFG;
     TB0CTL |= MC__STOP;
     TB0CCTL0 &= ~CCIFG; */
}

//_______________________________________________________________________________________________________________________________________
//MAIN FUNCTION
//_______________________________________________________________________________________________________________________________________
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;    // stop watchdog timer

    __disable_interrupt();
    Port_Setup();//Port initialisierung
    Clock_Setup();//Clock init
    TimerB0_Setup();//TimerB0 init
    UART_Setup();//UART init
    ADC_Setup();//ADC init
    TimerA0_Setup();//Timer A0 init
    Low_Akku = 0;//laufvariable die anzeigt ob die Akkuspannung unter der Grenze liegt auf 0 setzen

//_________________________________________________________________________________________________________________________________________________________________
//BEGIN DES PERIODISCH AUSGEFÜHRTEN AKTIVEN BETRIEBS (wird erst unterbrochen wenn die Akkuspannung unter der Grenze liegt)
//_________________________________________________________________________________________________________________________________________________________________
    while (Low_Akku != 1)
    {
        SMS_aktion = 0;
        GSM_Frozen = 0;
//_________________________________________________________________________________________________________________________________________________________________
//EINSCHALTEN GSM MODUL: 1. Check ob das Modul schon eingeschaltet ist -> 2. Wenn GSM-Modul aus ist, wird EInschaltvorgang ausgeführt -> 3. Prüfung ob Einschaltvorgang erfolgreich ist
//_________________________________________________________________________________________________________________________________________________________________
        while (Power_On == 0)
        {
            if ((P1IN & BIT2) == 0)
            {
                // GSM MODUL EINSCHALTEN
                Turn_On_GSM();
                //Check when Power Status on
                PWR_status();
            }
            //Wenn das Modul bereits eingeschaltet war power on auf 1 setzen
            else
            {
                Power_On = 1;
            }
        }

//_________________________________________________________________________________________________________________________________________________________________
//MESSUNG TEMPERATUR UND AKKU: 1. Akkumessung -> 2. Wenn Akkuspannung über der Grenze liegt -> Temperaturmessung
//_________________________________________________________________________________________________________________________________________________________________
        //MESSUNG AKKU
        Akku_Messung();
        //MESSUNG TEMP
        if (Low_Akku != 1)
        {
            Temp_Messung();
        }

//______________________________________________________________________________________________________________________________________________________
// TUART WARTEZEIT: Warten, sodass das GSM-Modul aufjedenfall genug Zeit hat hochzufahren
//_________________________________________________________________________________________________________________________________________________________________
        wait(1024, 0);//Wartezeit = 1 Sekunde

//_________________________________________________________________________________________________________________________________________________________________
//FÜLLEN DES STRINGS WELCHER MITTELS DER ÜBERTRAGEFUNKTION VERSENDET WIRD
//_________________________________________________________________________________________________________________________________________________________________

        Fill_SMS();
        Fill_SMS_GPS();

//_________________________________________________________________________________________________________________________________________________________________
//SENDEN DER SMS EINLEITEN UND AUSFÜHREN
//_________________________________________________________________________________________________________________________________________________________________
//NETZWERKVERBINDUNG HERSTELLEN-------------------------------------------------------------------------------------------------
        TransmitPIN();

        if (GSM_Frozen == 1)
        { //Wenn auf CPIN keine Antwort erhalten wurde Neustart ausführen
            ShutDown();
        }

        else
        {
            wait(820, 0); //Wenn CPIN korrekt abgelaufen ist, Verbindung prüfen, bevor die weiteren Schritte ausgeführt werden / vorher 0,8 sekunden wartezeit
            Check_Network();
        }

        if (Net_Stat == 1)
        { //Wenn keine Netzwerkverbindung herrscht folgt unten der Shutdown, wenn Verbindung herrscht Textmode CMGF Befehl übertragen
//TEXTMODE FESTLEGEN-------------------------------------------------------------------------------------------------------

            TransmitCMGF();//ENTER SMS TEXT MODE
            wait(820, 0);//NACH Befehl 0,8 Sekunden warten

//Test ob das GSM Modul auf den CMGF befehl geantwortet hat, wenn nicht -> neustart
            if (GSM_Frozen == 1)
            {
                ShutDown();
            }

            else
            {
//NUMBER MITTEILEN---------------------------------------------------------------------------------------------------------------
                TransmitNumber();

                //Test ob das GSM Modul nicht auf den CMGS befehl geantwortet hat
                if (GSM_Frozen == 1)
                {
                    ShutDown();
                }

                else
                {
                    wait(820, 0);
//TEXT MITTEILEN UND NACHRICHT ABSCHICKEN------------------------------------------------------------------------------------
                    TransmitText();
                    wait(1200, 0);

                    //cloudCommands();

                    setFunctionality();
                    wait(1200, 0);
                    checkSignalStrength();
                    wait(1200, 0);
                    findNetwork();
                    wait(1200, 0);
                    ueInfo();
                    wait(1200, 0);
                    attachDevice();
                    wait(1200, 0);
                    setPDP();
                    wait(1200, 0);
                    openTCP();//Works
                    wait(1200, 0);
                    setPDPProfile();// Might not be working
                    wait(5000, 0);

                    startHTTP();

                    wait(5000, 0);

                    connectToServer();
                    wait(5000, 0);
                    httpSendRequest();
                    wait(2000, 0);
                    sendDataToCloud();
                    wait(4000, 0);
                    stopHTTP();
                    wait(4000, 0);
                    int j;

                    char done[] =
                    {   "Die GPS-Koordinaten wurden erfolgreich auf Ihren ThingSpeak-Kanal hochgeladen."};

                    for (j = 0; j < sizeof(done); j++)
                    {
                        AT_TEXT[j] = done[j];
                    }

                    TransmitNumber();
                    wait(820, 0);
                    TransmitText();
                    ShutDown();
                }
            }
        }

        else
        { //Wenn Netstat nicht ==1, also keine Netzwerkverbindung vorliegt Shutdown ausführen
            ShutDown();//
        }
    } //Ende der HAupt-While Schleife
    return 0;
}
//_______________________________________________________________________________________________________________________________________
// TIMER B0 INTERRUPT ROUTINE
//_______________________________________________________________________________________________________________________________________
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER_B0_ISR(void) //__ bedeutet RTI
{
    __disable_interrupt();
    TB0CCTL0 &= ~CCIFG; //Clear Flag
    TB0CTL |= MC__STOP; //Stop Timer B0

//Interruptroutine wenn Timer B= dafür benutzt wird das GSM-Modul einzuschalten
    if (TimerB0_Task == 1)
    {
        P1OUT |= BIT1; //Port1 Pin 1 wieder auf High setzen
    }

//Interruptroutine wenn TimerB0 bei der UART-Kommunikation als Watchdog benutzt wird
    if (TimerB0_Task == 2)
    {
        GSM_Frozen = 1;
    }

//Routine wenn TimerB0 bei Prüfung der Netzwerkverbindung als WDT benutzt wird
    if (TimerB0_Task == 3)
    {
        Net_Stat = 0;
        TA0CCTL2 &= ~BIT4; //capture Intterrupt disnable
        TA0CTL |= MC__STOP; //timer stop
        TA0R = 0;
        TA0CCTL2 &= ~CCIFG;
    }

//Interrupt Routine wenn TimerB0 bei der Überprüfung des Ein und Ausschaltvorgangs als Watchdog benutzt wird
    if (TimerB0_Task == 5)
    {
        P1IFG &= ~BIT2;   // Clear P1.2 IFG
        P1IE &= ~BIT2;    // Clear P1.2 IE
    }

    __bic_SR_register_on_exit(LPM3_bits);
}
//_______________________________________________________________________________________________________________________________________
//ADC 12 INTERRUPT ROUTINE
//_______________________________________________________________________________________________________________________________________
#pragma vector = ADC12_VECTOR
__interrupt void ADC12ISR(void)
{

//Akku_Messung
    if (ADC_Conversion == 1)
    {
        ADC12CTL0 &= ~(BIT1 + BIT0); //DISABLE + STOP CONVERSION
        ADC12CTL0 &= ~BIT4; //ADC OF
        U_ADC_Akku = ADC12MEM0;
        // P1OUT ^= BIT4; //toggle led1
        ADC12MCTL0 &= ~(BIT0 + BIT1 + BIT2 + BIT3); // clear  input Channel
    }
//Temperatur_Messung
    if (ADC_Conversion == 2)
    {
        ADC12CTL0 &= ~(BIT1 + BIT0); //DISABLE + STOP CONVERSION
        ADC12CTL0 &= ~BIT4; //ADC OF
        U_ADC_T = ADC12MEM0;
        ADC12MCTL0 &= ~(BIT0 + BIT1 + BIT2 + BIT3); //clear input channel (eigentlichnich notwendig, wird jedoch ausgeführt um sicher zu sein, dass kein falscher kanal aktiv ist)
        // P1OUT ^= BIT4; //toggle led1
//    ADC12MCTL0 &= ~BIT1; // clear A1 input Channel
    }
    ADC12IFG &= ~ADC12IFG0;
    __bic_SR_register_on_exit(LPM3_bits);
}
//_______________________________________________________________________________________________________________________________________
// PORT1 INTERRUPT ROUTINE
//_______________________________________________________________________________________________________________________________________
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{

//    P1OUT ^= BIT4; //toggle led1
    if (Power_On == 1)
    {
        P1IFG &= ~BIT2;                         // Clear P1.2 IFG
        P1IE &= ~BIT2;                         // Clear P1.2 IE
        Power_On = 2;
    }

    if (Power_On == 0)
    {
        Power_On = 1;
        //P1OUT ^= BIT4; //toggle led1
        P1IFG &= ~BIT2;                         // Clear P1.2 IFG
        P1IE &= ~BIT2;                         // Clear P1.2 IE
    }
    TB0CCTL0 &= ~CCIFG;
    TB0CTL |= MC__STOP;
    __bic_SR_register_on_exit(LPM3_bits);   // Exit LPM3
}
//_______________________________________________________________________________________________________________________________________
// Capture Timer A0 Interrupt Routine
//_______________________________________________________________________________________________________________________________________
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER_A0_ISR(void) //__ bedeutet RTI
{

    if (Capture_Index == 1)
    {
        T_Capture = TA0R;
        TimerB0_Task = 0;
        TB0CCTL0 &= ~CCIFG; //Clear Flag
        TB0CTL |= MC__STOP; //Stop Timer B0
    }
//if (Capture_Index==2) {
//    Capture_Index=3;
//}
    TA0CCTL2 &= ~CCIFG;
//    P1OUT ^= BIT4; //toggle led1
    __bic_SR_register_on_exit(LPM3_bits);
}
//_______________________________________________________________________________________________________________________________________
// UART RECIEVE INTERRUPT
//_______________________________________________________________________________________________________________________________________
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{

//    Recieve_Test[Recieve_Index]=UCA0RXBUF;
//    Recieve_Index++;
    UCA0IFG &= ~ UCRXIFG;

    GSM_Frozen = 0;
    TB0CCTL0 &= ~CCIFG;
    TB0CTL |= MC__STOP; //Stoppen des WD Timers B0

}

