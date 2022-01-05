
#include <math.h>
#include <SPI.h>        
#include <Ethernet2.h>  // para el shield Ethernet shield 2
#include <EthernetUdp2.h> 


#define DES_RUEDA 509  // Desarrollo de la rueda (mm) que mide la velocidad digital
#define N_PULSOS 1     // Numero de pulsos del encoder por vuelta


class Monitor {
protected:
	String maquina;
	int pinPower;
	int pinAuto;
	int pinVelocidad;
	int pinVelocidadDig;
	int pinPiezaOK;
	int pinPiezaNOK;
public:
	Monitor(String, int, int, int, int, int, int);
	Monitor(const Monitor &);
	int powerValue() const;
	int autoValue() const;
	int getPinPower() const { return pinPower; }
	int getPinAuto() const { return pinAuto; }
	int velocidadValue() const;
	int velocidadDigValue() const;
	int piezaOKValue() const;
	int piezaNOKValue() const;
	int getPinVel() const { return pinVelocidad; }
	int getPinVelDig() const { return pinVelocidadDig; }
	int getPinPiezaOK() const { return pinPiezaOK; }
	int getPinPiezaNOK() const { return pinPiezaNOK; }
	String getMaquina() const { return maquina; }
};


class Comunicador {
protected:
	Monitor mon;
	EthernetUDP Udp;
	IPAddress ip;
	boolean conexion;
	unsigned puerto;
	boolean cambioStatus;
	String powerCtrl;
	String autoCtrl;
	int velocidadCtrl;
	float velocidadDigCtrl;
	float velMmMin;
	unsigned long pcsInterruptMs;
	float longitud;
	float largoPieza;
	float piezasMin;
	String pcOKCtrl;
	boolean pcOKPulso;
	String pcNOKCtrl;
public:
	Comunicador(Monitor, IPAddress, unsigned);
	void setVel(float);
	void setlCorte(float);
	void setPcOKPulso(boolean);
	void setPcsInterruptMs(unsigned long);
	String monitorPower();
	String monitorAuto();
	String monitorVelocidad();
	String monitorVelocidadDig();
	String monitorPcOK();
	String monitorPcNOK();
	void beginUdp();
	IPAddress getIPDestino() const { return ip; }
	boolean conectado() const { return conexion; }
	void sendUDP(String);
	void recibirConexion();
	void envioSerialyUDP(String, boolean);
};

/*_____________________________Definicion de Monitor__________________________________*/


Monitor::Monitor(String maq, int pinP, int pinA, int pinV, int pinVd, int pinPc, int pinPNOK) {
	maquina = maq;
	pinPower = pinP;
	pinAuto = pinA;
	pinVelocidad = pinV;
	pinVelocidadDig = pinVd;
	pinPiezaOK = pinPc;
	pinPiezaNOK = pinPNOK;
	pinMode(pinPower, INPUT_PULLUP);  // son resistencias de pullUp internas, de 20K 
	pinMode(pinAuto, INPUT_PULLUP);
	pinMode(pinVelocidadDig, INPUT_PULLUP);
	pinMode(pinPiezaOK, INPUT_PULLUP);
	pinMode(pinPiezaNOK, INPUT_PULLUP);
}

Monitor::Monitor(const Monitor &c) {  //Constructor de copia
	maquina = c.maquina;
	pinPower = c.pinPower;
	pinAuto = c.pinAuto;
	pinVelocidad = c.pinVelocidad;
	pinVelocidadDig = c.pinVelocidadDig;
	pinPiezaOK = c.pinPiezaOK;
	pinPiezaNOK = c.pinPiezaNOK;
	pinMode(pinPower, INPUT_PULLUP);  // son resistencias de pullUp internas, de 20K 
	pinMode(pinAuto, INPUT_PULLUP);
	pinMode(pinVelocidadDig, INPUT_PULLUP);
	pinMode(pinPiezaOK, INPUT_PULLUP);
	pinMode(pinPiezaNOK, INPUT_PULLUP);
}


int Monitor::powerValue() const {
	return digitalRead(pinPower);
}

int Monitor::autoValue() const {
	return digitalRead(pinAuto);
}

int Monitor::velocidadValue() const {
	return analogRead(pinVelocidad);
}

int Monitor::velocidadDigValue() const {
	return digitalRead(pinVelocidadDig);
}

int Monitor::piezaOKValue() const {
	return digitalRead(pinPiezaOK);
}

int Monitor::piezaNOKValue() const {
	return digitalRead(pinPiezaNOK);
}


/*_____________________________Definicion de Comunicador__________________________________*/

Comunicador::Comunicador(Monitor monitorP, IPAddress dirIP, unsigned p):mon(monitorP) {
	ip = dirIP;
	puerto = p;	
	conexion = false;
	cambioStatus = false;
	powerCtrl = "OFF";
	autoCtrl = "OFF";
	pcOKCtrl = "000";
	pcNOKCtrl = "000";
	pcOKPulso = false;
	velocidadCtrl = 0;
	velocidadDigCtrl = 0;
	velMmMin = 0;
	longitud = 0;
	largoPieza = 0;
	piezasMin = 0;
}

/*___________________________________________________________________________________________*/

void Comunicador::setVel(float veloc) {
	velMmMin= veloc;
}

void Comunicador::setlCorte(float l) {
	largoPieza = l;
}

void Comunicador::setPcOKPulso(boolean flag) {
	pcOKPulso = flag;
}

void Comunicador::setPcsInterruptMs(unsigned long t) {
	pcsInterruptMs = t;
}

/*_____________________________________________________________________________________________*/

String Comunicador::monitorPower() { //Devuelve la cadena de salida para power
	String str;
	str = mon.getMaquina() + "PWR_";
	cambioStatus = false;            // Actualizar cambio de status

	if (mon.powerValue() == HIGH) {  // Si la maquina esta parada (HIGH inverso por el pullUp)
		if (powerCtrl == "ONN") {        // Si el estado anterior era ON
			powerCtrl = "OFF";      // actualizar estado de power
			cambioStatus = true;            // Hay cambio de status
		}
	}
	else {                     // Si la maquina esta en funcionamiento
		if (powerCtrl == "OFF") {      // Si el estado anterior era OFF  
			powerCtrl = "ONN";     // actualizar estado de power
			cambioStatus = true;   //  Hay cambio de status
		}
	}
	str += powerCtrl;
	return str;
}

/*_____________________________________________________________________________________________*/

String Comunicador::monitorAuto() {
	String str;
	str = mon.getMaquina() + "AUT_";
	cambioStatus = false;            // Actualizar cambio de status

	if (mon.autoValue() == HIGH) {  // Si la maquina esta parada (HIGH inverso por el pullUp)
		if (autoCtrl == "ONN") {        // Si el estado anterior era ON
			autoCtrl = "OFF";      // actualizar estado de power
			cambioStatus = true;            // Hay cambio de status
		}
	}
	else {                     // Si la maquina esta en funcionamiento
		if (autoCtrl == "OFF") {      // Si el estado anterior ers OFF  
			autoCtrl = "ONN";     // actualizar estado de power
			cambioStatus = true;   //  Hay cambio de status
		}
	}
	str += autoCtrl;
	return str;
}

/*____________________________________________________________________________________________*/

String Comunicador::monitorVelocidad() {  //Velocidad analogica
	unsigned mapVel;
	String str, velStr;
	str = mon.getMaquina() + "VEL_";
	cambioStatus = false;
	
	if (abs(map(mon.velocidadValue(),0,1023,0,500) - map(velocidadCtrl,0,1023,0,500)) >= 1) {
		velocidadCtrl = mon.velocidadValue();
		cambioStatus = true;
	}
	mapVel = map(velocidadCtrl, 0, 1023, 0, 500);
	velStr = String(mapVel);
	if (mapVel < 100) {
		velStr = "0" + velStr;
		if (mapVel < 10)
			velStr = "0" + velStr;
	}

	str += velStr;
	return str;
}

/*_____________________________________________________________________________________________*/
String Comunicador::monitorVelocidadDig() {

	String str, velStr;
	str = mon.getMaquina() + "VEL_";
	cambioStatus = false;	
	
	if (velMmMin < 3900) {
		velMmMin = 0;
		setLargoEncoder(0);
	}

	longitud = getLargoEncoder();

	if (abs(velMmMin - velocidadDigCtrl)>1000.0) {  //OJO! esta en mm/min
			velocidadDigCtrl = velMmMin;
			cambioStatus = true;
	}
	
	long v = round(velocidadDigCtrl/1000.0);  //devolver velocidad en m/min
	velStr = String(v);
	
	if (v < 100) {                
		velStr = "0" + velStr;
		if (v < 10)
			velStr = "0" + velStr;
	}
	str += velStr;
	return str;
	
	
}

/*_____________________________________________________________________________________________*/

String Comunicador::monitorPcOK() {
	String str;
	str = mon.getMaquina() + "POK_";

	cambioStatus = false;            // Actualizar cambio de status	
	if (pcsInterruptMs > 0) 
		piezasMin = 60000000.0 / pcsInterruptMs;	
	if (mon.piezaOKValue() == LOW) { 
	//if(pcOKPulso) {
		if (pcOKCtrl == "000") {
			pcOKCtrl = "001";
			cambioStatus = true;            // Hay cambio de status
		}
	}
	else
		pcOKCtrl = "000";

	str += pcOKCtrl;
	return str;
}

/*________________________________________________________________________________________________*/

String Comunicador::monitorPcNOK() {
	String str;
	str = mon.getMaquina() + "PNK_";
	cambioStatus = false;            // Actualizar cambio de status

	if (mon.piezaNOKValue() == LOW) {
		if (pcNOKCtrl == "000") {
			pcNOKCtrl = "001";
			cambioStatus = true;            // Hay cambio de status
		}
	}
	else
		pcNOKCtrl = "000";

	str += pcNOKCtrl;
	return str;
}

/*__________________________________________________________________________________________________*/

void Comunicador::beginUdp() {
	Udp.begin(puerto);
	delay(200);
}

/*__________________________________________________________________________________________________*/

void Comunicador::sendUDP(String cad) {
	char* buffer = new char[cad.length() + 1];
	cad.toCharArray(buffer, cad.length() + 1);
	Udp.beginPacket(ip, puerto);
	Udp.write(buffer);
	Udp.endPacket();
	delete[] buffer;
}

/*__________________________________________________________________________________________________*/

void Comunicador::recibirConexion() {
	char packetBuffer[UDP_TX_PACKET_MAX_SIZE]=""; //buffer para el paquete de entrada
	int packetSize = Udp.parsePacket();

	if (packetSize) {
		Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
		String s(packetBuffer);

		if (s.equals("CONECTA")) {  //Si es el codigo del servidor (clave: "CONECTA")
			if (conexion) 
				sendUDP(mon.getMaquina() + "CNX_OFF:datUDP001");			
			ip = Udp.remoteIP();  // asignamos la ip del remitente al comunicador
			conexion = true;
			sendUDP(mon.getMaquina() + "CNX_ON:datUDP001");
			sendUDP(mon.getMaquina() + "CNX_ON:datUDP002");
			delay(100);		
		}
		else if (s.equals("DESCONECTA")) {		
			conexion = false;
			sendUDP(mon.getMaquina() + "CNX_OFF:datUDP001");
		}
	}
}

/*__________________________________________________________________________________________________*/

void Comunicador::envioSerialyUDP(String var, boolean soloSiCambio) {
	String estadoVar;
	/*********** POWER ************/
	if (var == "PWR") {
		estadoVar = monitorPower();
		if (soloSiCambio) {          // envia solo si hay cambio 
			if (cambioStatus) 
				sendUDP(estadoVar);
		}
		else                       // envia siempre
			sendUDP(estadoVar);		
	}

	/************ AUTO *************/
	if (var == "AUT") {
		estadoVar = monitorAuto();
		if (soloSiCambio) {
			if (cambioStatus) 
				sendUDP(estadoVar);
		}
		else                       // envia siempre
			sendUDP(estadoVar);
	}

	/*********** VELOCIDAD **********/
	if (var == "VEL") {
		estadoVar = monitorVelocidadDig();
		if (soloSiCambio) {          // envia solo si hay cambio 
			if (cambioStatus) 
				sendUDP(estadoVar);
		}
		else {                         // envia siempre
			sendUDP(estadoVar);
			if (velocidadDigCtrl > 0) 
				sendUDP(mon.getMaquina() + "LNG_" + String(round(longitud)));
		}
	}

	/************* PiezaOK *************/
	if (var == "POK")
		estadoVar=monitorPcOK();
		if (cambioStatus) {
			sendUDP(estadoVar);	
			sendUDP(mon.getMaquina() + "LPC_" + String(round(largoPieza)));
			sendUDP(mon.getMaquina() + "PPM_" + String(round(piezasMin*1000.0)));
			sendUDP(mon.getMaquina() + "TPC_" + String(pcsInterruptMs));
			sendUDP(mon.getMaquina() + "VMM_" + String(round(velMmMin)));
		}

	/************* PiezaNOK *************/
		if (var == "PNK")
			estadoVar = monitorPcNOK();
		if (cambioStatus) 
			sendUDP(estadoVar);	
}

/*______________________________________________________________________________________*/


/***************************************************************************************/
/**********************************  MAIN **********************************************/
/***************************************************************************************/


volatile boolean pulso = false;
volatile unsigned long pulsoAnteriorVel=0;
volatile unsigned long velocidadDigMicros=0;

volatile boolean pulsoPieza = false;
volatile unsigned long pulsoAntPiezas = 0;
volatile unsigned long piezasMicros = 0;

volatile float largoEncoder = 0;
volatile float incLong = DES_RUEDA / N_PULSOS; // avance de cada pulso del encoder

volatile unsigned long T0 = 0;
volatile unsigned long T1 = 0;
volatile unsigned long tx = 0;
volatile unsigned long txp = 0;

volatile float velMM = 0;
volatile float lg = 0; 
volatile float largoCorte = 0;

unsigned long t=0;
unsigned long previousTime=0;
unsigned long tiempoEnvio = 2000000; //microsegundos
IPAddress gate(10, 25, 0, 1);  // Gateway Gonvarri visible. Solo Hiasa: (0,0,0,0).
IPAddress subRed(255, 255, 0, 0);
IPAddress nullIP(0, 0, 0, 0); // IP 0, de control
/******************* INFORMACION ESPECIFICA PLACA **********************/

byte mac[] = { 0x90, 0xA2, 0xDA, 0x10, 0x6A, 0xE5 }; //Mac de la tarjeta Etehernet 
IPAddress myIP(10, 25, 7, 30); // Ip para la placa
int port = 7501; // Puerto para envio

Monitor m("TUBOS003_", 8, 9, A0, 2, 3, 6); 
// Pines :POWER 8, AUTO 9, VELOCIDAD A0, Vel Digital 2(int), PcOK 3(int), PcNOK 6


/***********************************************************************/
Comunicador c(m, nullIP, port);


void setLargoEncoder(float l) {
	largoEncoder = l;
}
float getLargoEncoder() {
	return largoEncoder;
}

void piezasISR() {
	txp = micros();
	if (txp - T1 > 60000) {		// Elimina rebotes: de menos de 80ms
		if (pulsoPieza) {			// Solo en flanco en el que se inicialice la variable pulso			
			largoCorte = lg + ((incLong*(txp - pulsoAnteriorVel)) / velocidadDigMicros); //**
			lg = 0;
			piezasMicros = txp - pulsoAntPiezas;
			pulsoAntPiezas = txp;
		}
		pulsoPieza = !pulsoPieza;// Cambio de flanco
	}
	T1 = txp;
}

void setVelocidadDigEMR() {		//Rele electromecanico, CHANGE. (Hago control de rebotes y de flanco).
	tx = micros();
	if (tx - T0 > 2500) {		// Elimina rebotes: de menos de 2.5ms
		if (pulso) {			// Solo en flanco: en el que se inicialice la variable pulso
			velocidadDigMicros = tx - pulsoAnteriorVel;
			pulsoAnteriorVel = tx;
			if (lg == 0)	
				lg = (incLong*(tx - pulsoAntPiezas)) / velocidadDigMicros;
			else
				lg += incLong;
			largoEncoder += incLong;
			velMM= (incLong * 60000000.0) / velocidadDigMicros;
		}
		pulso = !pulso;// Cambio de flanco
	}
	T0 = tx;
}




/* Para convertir a .ino
void setup() {
	pinMode(4, OUTPUT);
	digitalWrite(4, HIGH);//Desactivar la Micro SD, es incompatible con ethernet
	Ethernet.begin(mac, myIP, gate, gate, subRed);
	c.beginUdp();
	Serial.begin(9600);
	Serial.println(Ethernet.localIP());

	attachInterrupt(digitalPinToInterrupt(2), setVelocidadDigEMR, CHANGE);  //CHANGE, controlar rebotes y pulsos. 
	attachInterrupt(digitalPinToInterrupt(3), piezasISR, CHANGE);  //CHANGE, controlar rebotes y pulsos. 
	
	if (m.piezaOKValue() == LOW)
		pulsoPieza = true;
	if (m.velocidadDigValue() == LOW)
		pulso = true;
}   

void loop() {
		
		c.recibirConexion();      //Lee paquete y asigna ip del remitente

		if (c.conectado()) {
			c.envioSerialyUDP("PWR", true);  // Enviar datos cuando hay cambio
			c.envioSerialyUDP("AUT", true);		
			c.setVel(velMM);
			c.envioSerialyUDP("VEL", true);	
			c.setPcsInterruptMs(piezasMicros);
			c.setlCorte(largoCorte);
			c.setPcOKPulso(pulsoPieza);
			c.envioSerialyUDP("POK", true); //Enviar true: solo si hay cambio de estado		
		    c.envioSerialyUDP("PNK", true);	

			// Ademas enviar pwr,aut y vel cada tiempoEnvio ms:		
			t = micros();
			if (t < previousTime)
				previousTime = 0;  //Para cuando desborda el unsigned long			
			
			if (t - previousTime >  tiempoEnvio) {
				if (t - pulsoAnteriorVel > velocidadDigMicros) {   // Fader de la velocidad
					velocidadDigMicros = t - pulsoAnteriorVel;
					velMM = (incLong * 60000000.0) / velocidadDigMicros;			
					if (velocidadDigMicros > 500000000)
						velocidadDigMicros = 500000000;
				}
				
				if (t - pulsoAntPiezas > piezasMicros) {   // Fader de las piezas
					piezasMicros = t - pulsoAntPiezas;
					if (piezasMicros > 500000000)
						piezasMicros = 500000000;
				}
				c.envioSerialyUDP("PWR", false);     //Enviar false: envia siempre	
				c.envioSerialyUDP("AUT", false);
				c.setVel(velMM);
				c.envioSerialyUDP("VEL", false);

				previousTime = t;
			}

		}
}
*/
/*____________________________________________________________________________________________*/
