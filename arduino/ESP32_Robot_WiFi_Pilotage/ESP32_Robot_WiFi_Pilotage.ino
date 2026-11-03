// ============================================================================
// ESP32 + LD06 + Mecanum Car V2 : lidar en direct par Wi-Fi + pilotage à distance
//
// v5 : le PWM est envoyé directement sur les 8 broches des moteurs de l'ESP32 (mode GPIO) :
//        M1A = D5    M1B = D18    M2A = D19    M2B = D21
//        M3A = D2    M3B = D4     M4A = D16    M4B = D17
// Les anciens modes I2C (bibliothèque MecanumCar_v2, ou I2C plus lent) restent disponibles
// sur la page de test, pour comparer. ATTENTION : en I2C, D19 et D21 servent de SDA/SCL,
// donc ils ne sont plus M2A/M2B : un seul des deux montages est le bon pour ton robot.
//
// PAGE DE TEST PWM PAR WI-FI : connecte le PC ou le téléphone au réseau « Robot-LiDAR »
// puis ouvre  http://192.168.4.1/  dans un navigateur : choix du mode, 8 curseurs (M1A à M4B),
// boutons avancer / reculer, un moteur à la fois, case « inverser » par moteur.
// Les valeurs réglées sont maintenues tant que la page reste ouverte ; si la liaison coupe
// (page fermée, Wi-Fi perdu), tout s'arrête seul 3 s plus tard.
//
// Le sketch fait aussi : le pilotage continu par WebSocket (page lidar_web.html),
// une vitesse réglable, la combinaison des 4 roues (diagonales, arcs) et un diagnostic.
//
// Messages reçus de la page (texte, par le WebSocket, port 81) :
//   D,vx,vy,w[,v]   vx = 1 avancer, -1 reculer
//                   vy = 1 décaler à gauche, -1 décaler à droite
//                   w  = 1 tourner à gauche, -1 tourner à droite
//                   v  = vitesse en % (10 à 100), facultatif
//   S               arrêt
// La page répète D,... toutes les 100 ms tant qu'une touche est tenue.
// Si plus rien n'arrive pendant 300 ms (touche relâchée, Wi-Fi coupé), le robot s'arrête.
//
// Messages envoyés à la page : un tour de lidar par message, en JSON
//   [{"a":angle_deg,"d":distance_mm,"i":intensite}, ...]
//
// Bibliothèques : LD06, MecanumCar_v2 (celles du module), WebSockets (Markus Sattler).
//
// Moniteur série (115200 bauds), pour tester sans la page :
//   t = fait tourner chaque roue 0,7 s l'une après l'autre (ROBOT SOULEVÉ !)
//   f = avance 1 s          x = arrêt
//   i = cherche la carte moteur sur toutes les broches possibles
// ============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ld06.h>
#include "MecanumCar_v2.h"

// --- Lidar ---
#define RX_LIDAR     33
#define TX_LIDAR     -1
#define LD06_PWM_PIN 32
#define LIDAR_BAUD   230400
LD06 ld06(Serial2, LD06_PWM_PIN);

// --- Wi-Fi : l'ESP32 crée son réseau ---
const char* ssid = "Robot-LiDAR";
const char* password = "password123";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// --- Moteurs : carte Mecanum Car V2 (I2C simulé par la bibliothèque, esclave 0x30) ---
// Broches du module : SDA = GPIO19, SCL = GPIO21.
mecanumCar* car = new mecanumCar(19, 21);
const uint8_t I2C_ADRESSE = 0x30;

// Les 8 broches des moteurs : canal 1 = M1A, 2 = M1B, 3 = M2A, 4 = M2B, 5 = M3A, 6 = M3B, 7 = M4A, 8 = M4B.
// Pour tourner vers l'avant : A = 0 et B = PWM. Vers l'arrière : A = PWM et B = 0.
// Si un moteur tourne à l'envers, coche « inverser » pour lui sur la page de test (puis note-le ici).
const uint8_t PIN_MOTEUR[8] = {5, 18, 19, 21, 2, 4, 16, 17};   // M1A M1B M2A M2B M3A M3B M4A M4B
const char*   NOM_ROUE[4]   = {"avant gauche (M2)", "avant droit (M1)", "arriere gauche (M3)", "arriere droit (M4)"};
const uint8_t MOTEUR_DE_ROUE[4] = {2, 1, 3, 4};   // avant gauche = M2, avant droit = M1, arrière gauche = M3, arrière droit = M4
int8_t sensMoteur[4] = {1, 1, 1, 1};              // M1..M4 : -1 pour inverser le sens d'un moteur

// Façon d'envoyer le PWM
enum { MODE_BIB = 0, MODE_LENT = 1, MODE_GPIO = 2 };
int modeMoteur = MODE_GPIO;                       // GPIO direct par défaut
const int PWM_FREQ = 2000;                        // Hz (PWM sur les broches en mode GPIO)

const int VITESSE_DEFAUT_PCT = 70;      // utilisée si la page n'envoie pas de vitesse
const int PWM_MINI = 60;                // en dessous, les petits moteurs jaunes ne démarrent pas

const uint32_t DELAI_ARRET_MS = 300;    // arrêt automatique si aucun ordre pendant ce temps
uint32_t dernierOrdreMs = 0;
uint32_t dernierMessageMs = 0;
uint32_t dernierRafraichissementMs = 0;
int pwmRoue[4]  = {0, 0, 0, 0};          // vitesse demandée à chaque roue (-255 à 255)
int pwmEnvoye[4] = {1000, 1000, 1000, 1000};   // dernière valeur envoyée à la carte (1000 = inconnue)

// La carte répond-elle à son adresse ? (test ACK, avec l'horloge de la bibliothèque)
bool cartePresente(mecanumCar* c) {
  c->IIC_Start();
  c->IIC_SendByte(I2C_ADRESSE << 1);
  bool nack = c->IIC_RecvACK();          // true = pas de réponse
  c->IIC_Stop();
  return !nack;
}

// --- I2C « lent » : même protocole que la bibliothèque, horloge ≈ 100 kHz ---
const int PIN_SDA = 19;
const int PIN_SCL = 21;

inline void attente() { delayMicroseconds(5); }

void lStart() {
  digitalWrite(PIN_SDA, HIGH); digitalWrite(PIN_SCL, HIGH); attente();
  digitalWrite(PIN_SDA, LOW);  attente();
  digitalWrite(PIN_SCL, LOW);  attente();
}
void lStop() {
  digitalWrite(PIN_SCL, LOW);  attente();
  digitalWrite(PIN_SDA, LOW);  attente();
  digitalWrite(PIN_SCL, HIGH); attente();
  digitalWrite(PIN_SDA, HIGH); attente();
}
bool lOctet(uint8_t b) {                 // true = la carte a répondu (ACK)
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_SCL, LOW); attente();
    digitalWrite(PIN_SDA, (b & 0x80) ? HIGH : LOW); b <<= 1; attente();
    digitalWrite(PIN_SCL, HIGH); attente();
  }
  digitalWrite(PIN_SCL, LOW); attente();
  pinMode(PIN_SDA, INPUT_PULLUP); attente();
  digitalWrite(PIN_SCL, HIGH); attente();
  bool ack = (digitalRead(PIN_SDA) == LOW);
  digitalWrite(PIN_SCL, LOW); attente();
  pinMode(PIN_SDA, OUTPUT); digitalWrite(PIN_SDA, LOW);
  return ack;
}
bool lEcrire(uint8_t canal, uint8_t valeur) {
  pinMode(PIN_SDA, OUTPUT); pinMode(PIN_SCL, OUTPUT);
  lStart();
  bool a = lOctet(I2C_ADRESSE << 1);
  bool b = lOctet(canal);
  bool c = lOctet(valeur);
  lStop();
  return a && b && c;
}

// --- PWM direct sur les broches (mode GPIO) ---
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(i)  ledcAttach(PIN_MOTEUR[i], PWM_FREQ, 8)
  #define PWM_WRITE(i,v) ledcWrite(PIN_MOTEUR[i], v)
  #define PWM_DETACH(i)  ledcDetach(PIN_MOTEUR[i])
#else
  #define PWM_ATTACH(i)  do { ledcSetup(i, PWM_FREQ, 8); ledcAttachPin(PIN_MOTEUR[i], i); } while (0)
  #define PWM_WRITE(i,v) ledcWrite(i, v)
  #define PWM_DETACH(i)  ledcDetachPin(PIN_MOTEUR[i])
#endif

// Passe les 8 broches en PWM (actif = true) ou les rend aux montages I2C (actif = false).
void gpioActiver(bool actif) {
  for (int i = 0; i < 8; i++) {
    if (actif) {
      pinMode(PIN_MOTEUR[i], OUTPUT);
      digitalWrite(PIN_MOTEUR[i], LOW);
      PWM_ATTACH(i);
      PWM_WRITE(i, 0);
    } else {
      PWM_DETACH(i);
      pinMode(PIN_MOTEUR[i], OUTPUT);
      digitalWrite(PIN_MOTEUR[i], LOW);
    }
  }
  if (!actif) {                          // repos de l'I2C : SDA et SCL à l'état haut
    pinMode(PIN_SDA, OUTPUT); pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SDA, HIGH); digitalWrite(PIN_SCL, HIGH);
  }
}

const char* nomMode() {
  return modeMoteur == MODE_GPIO ? "GPIO direct" : (modeMoteur == MODE_LENT ? "I2C lent" : "I2C bibliotheque");
}

// Envoie une valeur de PWM (0 à 255) sur un canal (1 à 8), selon le mode choisi.
void ecrireCanal(uint8_t canal, uint8_t valeur) {
  if (modeMoteur == MODE_GPIO) PWM_WRITE(canal - 1, valeur);
  else if (modeMoteur == MODE_LENT) lEcrire(canal, valeur);
  else car->PWM_OUT(canal, valeur);
}

// La carte répond-elle (ACK) ? Seulement pour les modes I2C ; en mode GPIO il n'y a rien à tester.
bool carteRepond() {
  if (modeMoteur == MODE_GPIO) return true;
  if (modeMoteur == MODE_BIB) return cartePresente(car);
  pinMode(PIN_SDA, OUTPUT); pinMode(PIN_SCL, OUTPUT);
  lStart();
  bool ack = lOctet(I2C_ADRESSE << 1);
  lStop();
  return ack;
}

// Moteur m (1 à 4) : v de -255 (arrière) à 255 (avant) ; canaux A = 2m-1, B = 2m
void moteurM(int m, int v) {
  v = constrain(v * sensMoteur[m - 1], -255, 255);
  uint8_t A = 2 * m - 1, B = 2 * m;
  if (v >= 0) { ecrireCanal(A, 0); ecrireCanal(B, v); }
  else        { ecrireCanal(B, 0); ecrireCanal(A, -v); }
}

// Applique la vitesse d'une roue : -255 (arrière) à +255 (avant).
void moteur(int roue, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm == pwmEnvoye[roue]) return;    // déjà fait : on évite du trafic inutile
  pwmEnvoye[roue] = pwm;
  moteurM(MOTEUR_DE_ROUE[roue], pwm);
}

void arreter() {
  for (int r = 0; r < 4; r++) { pwmRoue[r] = 0; moteur(r, 0); }
}

// Coupe les 8 canaux (utilisé par la page de test et à la fin d'un essai manuel).
bool essaiManuel = false;                // un essai de la page de test est en cours
uint32_t essaiFinMs = 0;

void arreterTout() {
  for (uint8_t c = 1; c <= 8; c++) ecrireCanal(c, 0);
  for (int r = 0; r < 4; r++) { pwmRoue[r] = 0; pwmEnvoye[r] = 0; }
  essaiManuel = false;
}

bool enMouvement() {
  for (int r = 0; r < 4; r++) if (pwmRoue[r] != 0) return true;
  return false;
}

// Traduit (vx, vy, w) et une vitesse en % en vitesses des 4 roues, puis les applique.
void piloter(int vx, int vy, int w, int pct) {
  vx = constrain(vx, -1, 1); vy = constrain(vy, -1, 1); w = constrain(w, -1, 1);
  pct = constrain(pct, 10, 100);

  // Roues mecanum : avant = +, côté gauche = vy > 0, rotation gauche (anti-horaire) = w > 0.
  float v[4] = {
    (float)(vx - vy - w),   // avant gauche
    (float)(vx + vy + w),   // avant droit
    (float)(vx + vy - w),   // arrière gauche
    (float)(vx - vy + w)    // arrière droit
  };
  float m = 0;
  for (int r = 0; r < 4; r++) m = max(m, fabsf(v[r]));
  if (m < 1) m = 1;                      // pas d'amplification, seulement une réduction si une roue dépasse 100 %

  bool change = false;
  for (int r = 0; r < 4; r++) {
    float f = v[r] / m;
    int p = (int)lroundf(fabsf(f) * pct * 2.55f);
    if (p > 0 && p < PWM_MINI) p = PWM_MINI;
    p = f < 0 ? -p : p;
    if (p != pwmRoue[r]) change = true;
    pwmRoue[r] = p;
  }
  for (int r = 0; r < 4; r++) moteur(r, pwmRoue[r]);

  if (change) {
    Serial.printf("Ordre vx=%d vy=%d w=%d v=%d%%  ->  AG=%d AD=%d RG=%d RD=%d\n",
                  vx, vy, w, pct, pwmRoue[0], pwmRoue[1], pwmRoue[2], pwmRoue[3]);
    if (!carteRepond() && millis() - dernierMessageMs > 2000) {
      dernierMessageMs = millis();
      Serial.println("!! La carte moteur ne repond pas (I2C 0x30). Batteries branchees ? Interrupteur de la carte sur ON ? Tape 'i' pour chercher les broches.");
    }
  }
}

void traiterCommande(uint8_t* payload, size_t length) {
  char buf[48];
  size_t n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);
  buf[n] = 0;

  if (buf[0] == 'S') {
    if (enMouvement()) Serial.println("Ordre S : arret");
    arreter();
  } else if (buf[0] == 'D') {
    int vx = 0, vy = 0, w = 0, pct = VITESSE_DEFAUT_PCT;
    int lus = sscanf(buf, "D,%d,%d,%d,%d", &vx, &vy, &w, &pct);
    if (lus >= 3) {
      essaiManuel = false;
      dernierOrdreMs = millis();
      piloter(vx, vy, w, pct);
    }
  }
}

// --- WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] Page connectee\n", num);
      arreter();
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Page deconnectee\n", num);
      arreter();                         // plus personne ne pilote : on s'arrête
      break;
    case WStype_TEXT:
      traiterCommande(payload, length);
      break;
    default:
      break;
  }
}

// Un tour de lidar = un message JSON, avec tous les points et l'intensité.
void envoyerDonneesLidarWebSocket() {
  uint16_t nbPoints = ld06.getNbPointsInScan();
  if (nbPoints == 0) return;

  String json;
  json.reserve(nbPoints * 34 + 8);
  json = "[";
  bool premier = true;
  for (uint16_t i = 0; i < nbPoints; i++) {
    const auto* pt = ld06.getPoints(i);
    if (pt->distance == 0) continue;
    if (!premier) json += ",";
    premier = false;
    json += "{\"a\":" + String(pt->angle, 1) + ",\"d\":" + String(pt->distance)
          + ",\"i\":" + String(pt->intensity) + "}";
  }
  json += "]";
  webSocket.broadcastTXT(json);
}

// ============================================================================
// Page de test PWM par Wi-Fi (servie à http://192.168.4.1/)
// ============================================================================
const char PAGE_TEST[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Test PWM du robot</title>
<style>
 body{font:15px system-ui,sans-serif;margin:0;padding:14px;max-width:560px;margin:auto;color:#1c2330;background:#f6f7f9}
 h1{font-size:19px;margin:0 0 4px} h2{font-size:15px;margin:18px 0 6px}
 .carte{background:#fff;border:1px solid #d9dde4;border-radius:10px;padding:10px 12px;margin-top:10px}
 .ligne{display:grid;grid-template-columns:48px 1fr 42px;gap:8px;align-items:center;margin:5px 0}
 .ligne input{width:100%} .ligne output{text-align:right;font-variant-numeric:tabular-nums}
 button{font:inherit;padding:8px 12px;border:1px solid #b9c0cc;border-radius:8px;background:#fff;cursor:pointer;margin:3px 3px 3px 0}
 button:active{background:#dce6ff} button.stop{background:#c0301f;color:#fff;border-color:#c0301f;font-weight:600}
 #etat{font-weight:600} #log{font:12px ui-monospace,monospace;white-space:pre-wrap;background:#10151f;color:#cfe3ff;border-radius:8px;padding:8px;height:130px;overflow:auto}
 small{color:#5b6577}
</style></head><body>
<h1>Test PWM du robot</h1>
<small>Ce que tu règles ici part vers les moteurs (PWM direct sur les broches, ou I2C selon le mode). Si la liaison coupe, tout s'arrête seul au bout de 3 s. Robot soulevé pour les premiers essais.</small>

<div class="carte"><div id="etat">Connexion au robot…</div>
 <div style="margin-top:6px">I2C :
  <label><input type="radio" name="mode" value="2" checked> GPIO direct</label>
  <label><input type="radio" name="mode" value="0"> I2C bibliothèque</label>
  <label><input type="radio" name="mode" value="1"> I2C lent</label>
  <button id="ack">Tester</button></div></div>

<div class="carte"><h2 style="margin-top:0">Avancer / reculer (les 4 moteurs)</h2>
 <div class="ligne" style="grid-template-columns:78px 1fr 42px"><span>Puissance</span><input type="range" id="puiss" min="60" max="255" value="200"><output id="puissV">200</output></div>
 <button id="av">Avancer 1 s</button><button id="re">Reculer 1 s</button><button class="stop" id="stop">STOP</button>
 <h2>Un moteur à la fois (0,7 s)</h2><small>Si la roue tourne dans le mauvais sens pour « avant », coche « inverser ».</small><div id="unm"></div></div>

<div class="carte"><h2 style="margin-top:0">Les 8 canaux PWM (0 à 255)</h2>
 <small>Avant = A à 0 et B > 0. Arrière = A > 0 et B à 0.</small><div id="canaux"></div>
 <button id="zero">Tout à 0</button></div>

<h2>Journal</h2><div id="log"></div>

<script>
const $=id=>document.getElementById(id), noms=['M1A','M1B','M2A','M2B','M3A','M3B','M4A','M4B'];
function log(t){const l=$('log');l.textContent+=t+'\n';l.scrollTop=l.scrollHeight}
async function api(p){try{const r=await fetch(p);const t=await r.text();log(p+'  ->  '+t);
  if(/mode=/.test(t)){$('etat').textContent=t.replace(/^.*\| /,'');
  const m=/mode=GPIO/.test(t)?2:/mode=I2C lent/.test(t)?1:0;document.querySelector('input[name=mode][value="'+m+'"]').checked=true}return t}
 catch(e){log('Pas de réponse du robot ('+e.message+')');$('etat').textContent='Robot injoignable : es-tu connecté au réseau Robot-LiDAR ?'}}
const cv=[0,0,0,0,0,0,0,0], sl=[];
noms.forEach((n,i)=>{const d=document.createElement('div');d.className='ligne';
 d.innerHTML='<span>'+n+'</span><input type="range" min="0" max="255" value="0"><output>0</output>';
 const r=d.querySelector('input'),o=d.querySelector('output');sl.push(r);
 let t=0;r.addEventListener('input',()=>{cv[i]=+r.value;o.textContent=r.value;const now=Date.now();
  if(now-t>90){t=now;api('/pwm?c='+(i+1)+'&v='+r.value)}});
 r.addEventListener('change',()=>{cv[i]=+r.value;api('/pwm?c='+(i+1)+'&v='+r.value)});
 $('canaux').appendChild(d)});
setInterval(()=>{cv.forEach((v,i)=>{if(v>0)fetch('/pwm?c='+(i+1)+'&v='+v).catch(()=>{})})},1000);   // maintient les canaux réglés
function zero(){sl.forEach((r,i)=>{r.value=0;cv[i]=0;r.nextElementSibling.textContent='0'});return api('/stop')}
$('zero').onclick=zero; $('stop').onclick=zero;
$('puiss').oninput=()=>{$('puissV').textContent=$('puiss').value};
const P=()=>$('puiss').value;
$('av').onclick=()=>api('/avance?v='+P()+'&ms=1000');
$('re').onclick=()=>api('/avance?v=-'+P()+'&ms=1000');
for(let m=1;m<=4;m++){const b=document.createElement('div');
 b.innerHTML='<button>M'+m+' avant</button><button>M'+m+' arrière</button> <label><input type="checkbox"> inverser M'+m+'</label>';
 const[bb1,bb2]=b.querySelectorAll('button');b.querySelector('input').onchange=e=>api('/sens?m='+m+'&s='+(e.target.checked?-1:1));
 bb1.onclick=()=>api('/moteur?m='+m+'&v='+P()+'&ms=700');bb2.onclick=()=>api('/moteur?m='+m+'&v=-'+P()+'&ms=700');
 $('unm').appendChild(b)}
document.querySelectorAll('input[name=mode]').forEach(r=>r.onchange=()=>api('/mode?m='+r.value));
$('ack').onclick=()=>api('/ack');
api('/ack');
</script></body></html>)rawliteral";

void repondre(const char* texte) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", texte);
}

int argEntier(const char* nom, int defaut) {
  return server.hasArg(nom) ? server.arg(nom).toInt() : defaut;
}

// « mode=... » : ce que la page affiche en haut
void repondreEtat(const char* debut) {
  char buf[200];
  if (modeMoteur == MODE_GPIO)
    snprintf(buf, sizeof(buf), "%s | mode=GPIO direct (PWM sur D5 D18 D19 D21 D2 D4 D16 D17)", debut);
  else
    snprintf(buf, sizeof(buf), "%s | mode=%s carte=%s", debut, nomMode(),
             carteRepond() ? "repond (ACK)" : "NE REPOND PAS");
  repondre(buf);
}

void handleRoot() {
  server.send_P(200, "text/html", PAGE_TEST);
}

// /pwm?c=1..8&v=0..255 : écrit une valeur de PWM sur un canal
void handlePwm() {
  int c = constrain(argEntier("c", 1), 1, 8);
  int v = constrain(argEntier("v", 0), 0, 255);
  ecrireCanal(c, v);
  essaiManuel = true; essaiFinMs = millis() + 3000;
  char t[40]; snprintf(t, sizeof(t), "canal %d (%s) = %d", c, (c & 1) ? "A" : "B", v);
  Serial.printf("Page test : %s\n", t);
  repondreEtat(t);
}

// /moteur?m=1..4&v=-255..255&ms=700 : un moteur, pendant ms millisecondes
void handleMoteur() {
  int m = constrain(argEntier("m", 1), 1, 4);
  int v = constrain(argEntier("v", 200), -255, 255);
  int ms = constrain(argEntier("ms", 700), 50, 3000);
  arreterTout();                         // un seul moteur à la fois
  moteurM(m, v);
  essaiManuel = true; essaiFinMs = millis() + ms;
  char t[60]; snprintf(t, sizeof(t), "M%d = %d pendant %d ms (canaux %d et %d)", m, v, ms, 2 * m - 1, 2 * m);
  Serial.printf("Page test : %s\n", t);
  repondreEtat(t);
}

// /avance?v=-255..255&ms=1000 : les 4 moteurs ensemble
void handleAvance() {
  int v = constrain(argEntier("v", 200), -255, 255);
  int ms = constrain(argEntier("ms", 1000), 50, 3000);
  arreterTout();
  for (int m = 1; m <= 4; m++) moteurM(m, v);
  essaiManuel = true; essaiFinMs = millis() + ms;
  char t[60]; snprintf(t, sizeof(t), "4 moteurs = %d pendant %d ms", v, ms);
  Serial.printf("Page test : %s\n", t);
  repondreEtat(t);
}

void handleStop() {
  arreterTout();
  Serial.println("Page test : stop");
  repondreEtat("stop");
}

// /mode?m=0|1|2 : 0 = I2C bibliothèque, 1 = I2C lent, 2 = GPIO direct
void handleMode() {
  arreterTout();
  int ancien = modeMoteur;
  int nouveau = constrain(argEntier("m", MODE_GPIO), 0, 2);
  if (nouveau != ancien) {
    if (ancien == MODE_GPIO) gpioActiver(false);
    modeMoteur = nouveau;
    if (nouveau == MODE_GPIO) gpioActiver(true);
  }
  for (int r = 0; r < 4; r++) pwmEnvoye[r] = 1000;
  Serial.printf("Page test : mode %s\n", nomMode());
  repondreEtat("mode change");
}

// /sens?m=1..4&s=1|-1 : inverse le sens d'un moteur
void handleSens() {
  int m = constrain(argEntier("m", 1), 1, 4);
  sensMoteur[m - 1] = argEntier("s", 1) < 0 ? -1 : 1;
  char t[90];
  snprintf(t, sizeof(t), "sens M%d = %d  (sensMoteur = {%d, %d, %d, %d})", m, sensMoteur[m - 1],
           sensMoteur[0], sensMoteur[1], sensMoteur[2], sensMoteur[3]);
  Serial.printf("Page test : %s\n", t);
  repondreEtat(t);
}

void handleAck() {
  repondreEtat("test");
}

// --- Tests au moniteur série ---
void testRoues() {
  Serial.println("Test des roues (robot SOULEVE !). Chaque roue tourne vers l'avant 0,7 s.");
  if (!carteRepond()) Serial.println("  !! la carte ne repond pas (I2C 0x30), on essaie quand meme");
  for (int r = 0; r < 4; r++) {
    Serial.printf("  roue %d : %s\n", r + 1, NOM_ROUE[r]);
    moteur(r, 170);
    delay(700);
    moteur(r, 0);
    delay(300);
  }
  Serial.println("Fin du test. Si une roue tourne a l'envers : coche « inverser » pour son moteur sur la page de test. Si ce n'est pas la bonne roue qui bouge, dis-le moi.");
}

// Recherche large : toutes les paires de broches libres (on teste avec un objet temporaire).
void scannerBroches() {
  if (modeMoteur == MODE_GPIO) { Serial.println("Mode GPIO direct : pas d'I2C a chercher. Passe en mode I2C sur la page de test (http://192.168.4.1/)."); return; }
  const int8_t libres[] = {4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27};
  Serial.println("Recherche de la carte moteur (0x30) sur toutes les broches libres...");
  for (int8_t sda : libres) {
    for (int8_t scl : libres) {
      if (sda == scl) continue;
      mecanumCar essai(sda, scl);
      if (cartePresente(&essai)) {
        Serial.printf("  REPONSE : SDA = GPIO%d, SCL = GPIO%d\n", sda, scl);
        delete car;
        car = new mecanumCar(sda, scl);   // on garde ces broches jusqu'au prochain redémarrage
        for (int r = 0; r < 4; r++) pwmEnvoye[r] = 1000;
        Serial.println("  Pour que ce soit permanent : remplace 19 et 21 dans la ligne  mecanumCar* car = new mecanumCar(19, 21);");
        return;
      }
    }
  }
  Serial.println("  Aucune reponse. Batteries branchees et interrupteur de la carte sur ON ? Fils SDA/SCL ?");
  delete car;
  car = new mecanumCar(19, 21);           // remet les broches du module
}

void commandeSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') testRoues();
    else if (c == 'f') { Serial.println("Avance 1 s"); piloter(1, 0, 0, 70); delay(1000); arreter(); }
    else if (c == 'x') { Serial.println("Arret"); arreter(); }
    else if (c == 'i') scannerBroches();
  }
}

// --- Setup & loop ---
// Le Wi-Fi démarre EN PREMIER pour que le réseau « Robot-LiDAR » existe quoi qu'il arrive.
void setup() {
  if (modeMoteur == MODE_GPIO) gpioActiver(true);   // moteurs à l'arrêt (PWM à 0) dès le démarrage
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[1/4] Demarrage du Wi-Fi...");

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(ssid, password);          // mot de passe : 8 caracteres minimum
  Serial.print("[1/4] Reseau ");
  Serial.print(ssid);
  Serial.println(ok ? " cree" : " ECHEC");
  Serial.print("      Adresse : http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/pwm", HTTP_GET, handlePwm);
  server.on("/moteur", HTTP_GET, handleMoteur);
  server.on("/avance", HTTP_GET, handleAvance);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/mode", HTTP_GET, handleMode);
  server.on("/sens", HTTP_GET, handleSens);
  server.on("/ack", HTTP_GET, handleAck);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[2/4] Serveur pret : page de test http://192.168.4.1/  et WebSocket port 81");

  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  Serial2.setTimeout(10);
  ld06.init();
  Serial.println("[3/4] Lidar initialise");

  if (modeMoteur == MODE_GPIO) {
    Serial.println("[4/4] Moteurs : PWM direct sur M1A=D5 M1B=D18 M2A=D19 M2B=D21 M3A=D2 M3B=D4 M4A=D16 M4B=D17");
    Serial.println("      (pour tester l'I2C a la place : page http://192.168.4.1/, choix du mode)");
  } else {
    Serial.println("[4/4] Carte moteur (I2C 0x30, SDA = GPIO19, SCL = GPIO21)...");
    delay(1000);                                  // la carte moteur a besoin d'un instant après la mise sous tension
    car->Stop();
    if (cartePresente(car)) Serial.println("[4/4] La carte moteur repond (ACK)");
    else Serial.println("[4/4] !! La carte moteur ne repond pas. Batteries ? Interrupteur de la carte ?");
  }
  Serial.println("Commandes serie : t = test des roues (robot souleve), f = avance 1 s, x = arret, i = chercher la carte (I2C)");
}

void loop() {
  server.handleClient();
  webSocket.loop();
  commandeSerie();

  // Fin d'un essai de la page de test (3 s sans nouvel ordre, ou durée demandée écoulée)
  if (essaiManuel && (int32_t)(millis() - essaiFinMs) >= 0) {
    arreterTout();
    Serial.println("Page test : arret automatique");
  }

  // Arrêt automatique : plus d'ordre récent (touche relâchée, Wi-Fi coupé)
  if (enMouvement()) {
    if (millis() - dernierOrdreMs > DELAI_ARRET_MS) {
      arreter();
    } else if (millis() - dernierRafraichissementMs > 1000) {
      dernierRafraichissementMs = millis();
      for (int r = 0; r < 4; r++) pwmEnvoye[r] = 1000;   // on renvoie tout : la carte a pu redémarrer (coup de tension)
      for (int r = 0; r < 4; r++) moteur(r, pwmRoue[r]);
    }
  }

  if (ld06.readScan()) {
    envoyerDonneesLidarWebSocket();
  }
}
