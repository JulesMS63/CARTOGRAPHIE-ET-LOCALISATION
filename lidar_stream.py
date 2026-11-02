"""Réception du flux du LD06 (ESP32) : décodage, port série, simulation.

Aucune interface graphique ici : ce module est utilisé par `lidar_app.py` mais se teste seul.

Deux formats de texte sont compris, ceux de la librairie de l'ESP32 :
  - `printScanCSV`      : lignes  N,Angle(°),Distance(mm),Intensity,x,y   (ligne vide = fin d'un scan)
  - `printScanTeleplot` : lignes  >lidar:x:y;x:y;...|xy                    (une ligne = un scan, en mm)
Dans les deux cas on recalcule x et y nous-mêmes à partir de l'angle et de la distance.
"""

import math
import queue
import threading
import time

import numpy as np

from lidar_io import _make_scan

MIN_POINTS_PER_SCAN = 50        # un tour avec moins de points est ignoré (tour incomplet)

# Constructeurs de puces USB-série fréquentes sur les cartes ESP32 (identifiant USB -> nom)
USB_VENDORS = {
    0x10C4: "CP210x (Silicon Labs)",
    0x1A86: "CH340 (WCH)",
    0x0403: "FTDI",
    0x303A: "ESP32 USB natif (Espressif)",
    0x2341: "Arduino",
    0x067B: "Prolific",
}


# --------------------------------------------------------------------------
# Décodage du texte reçu
# --------------------------------------------------------------------------
class StreamParser:
    """Reconstruit des scans à partir de morceaux de texte arrivant au fil de l'eau.

    on_scan(rows) reçoit une liste [[angle_deg, distance_mm, intensité], ...].
    """

    def __init__(self, on_scan, tele_flip_y=True):
        self.on_scan = on_scan
        self.tele_flip_y = tele_flip_y      # la librairie imprime un y de signe opposé en Teleplot
        self.buf = ""
        self.rows = []
        self.last_angle = None

    def reset(self):
        self.buf = ""
        self.rows = []
        self.last_angle = None

    def feed(self, text):
        self.buf += text
        while True:
            i = self.buf.find("\n")
            if i < 0:
                break
            line = self.buf[:i].strip()
            self.buf = self.buf[i + 1:]
            self._line(line)
        if len(self.buf) > 200_000:         # pas de fin de ligne : flux illisible, on repart à zéro
            self.buf = ""

    def _flush(self):
        if len(self.rows) >= MIN_POINTS_PER_SCAN:
            self.on_scan(self.rows)
        self.rows = []
        self.last_angle = None

    def _line(self, line):
        if line == "":
            self._flush()
            return
        if line.startswith(">lidar:"):
            self._flush()
            self._teleplot(line)
            return
        fields = line.split(",")
        if len(fields) < 4:
            return
        try:
            values = [float(f) for f in fields]
        except ValueError:
            return                          # en-tête ou ligne de debug
        angle = values[1]
        # l'angle repart de ~0 : nouveau tour même sans ligne vide
        if self.last_angle is not None and abs(angle - self.last_angle) > 180:
            self._flush()
        self.rows.append([angle, values[2], values[3]])
        self.last_angle = angle

    def _teleplot(self, line):
        body = line[len(">lidar:"):]
        bar = body.find("|")
        if bar >= 0:
            body = body[:bar]
        rows = []
        for pair in body.split(";"):
            if not pair:
                continue
            try:
                xs, ys = pair.split(":")[:2]
                x, y = float(xs), float(ys)
            except ValueError:
                continue
            angle = math.degrees(math.atan2(-y if self.tele_flip_y else y, x)) % 360.0
            rows.append([angle, math.hypot(x, y), 255.0])
        if len(rows) >= MIN_POINTS_PER_SCAN:
            self.on_scan(rows)


def rows_to_scan(rows, name="live", offset_deg=0.0, mirror=False):
    """[[angle_deg, distance_mm, intensité], ...] -> Scan (mètres, radians, trié par angle).

    offset_deg et mirror corrigent le montage du capteur sur le robot.
    """
    arr = np.asarray(rows, dtype=float).reshape(-1, 3)
    angles = (-arr[:, 0] if mirror else arr[:, 0]) + offset_deg
    return _make_scan(name, angles, arr[:, 1] / 1000.0, arr[:, 2])


# --------------------------------------------------------------------------
# Ports série
# --------------------------------------------------------------------------
def pyserial_available():
    try:
        import serial  # noqa: F401
        import serial.tools.list_ports  # noqa: F401
        return True
    except ImportError:
        return False


def list_ports():
    """Ports série vus par le système : liste de dicts {device, description, vendor, vid, pid}."""
    from serial.tools import list_ports as lp
    ports = []
    for p in sorted(lp.comports(), key=lambda p: p.device):
        vendor = USB_VENDORS.get(p.vid, "") if p.vid is not None else ""
        ports.append({"device": p.device, "description": p.description or "",
                      "vendor": vendor, "vid": p.vid, "pid": p.pid})
    return ports


def port_label(p):
    """Texte court pour une liste déroulante."""
    extra = p["vendor"] or p["description"]
    return f"{p['device']} · {extra}" if extra and extra != "n/a" else p["device"]


def explain_serial_error(exc):
    """Traduit une exception pyserial en consigne compréhensible. Retourne (texte, port_occupé)."""
    msg = str(exc)
    low = msg.lower()
    if isinstance(exc, PermissionError) or "permission" in low or "access is denied" in low \
            or "accès refusé" in low or "acces refuse" in low or "busy" in low:
        return ("Le port est occupé ou refusé : fermez le moniteur série d'Arduino, Teleplot ou "
                "tout autre programme qui l'utilise, puis rebranchez la carte. (" + msg + ")", True)
    if isinstance(exc, FileNotFoundError) or "could not open port" in low or "no such file" in low \
            or "le fichier spécifié est introuvable" in low:
        return ("Port introuvable : la carte a été débranchée ou le nom du port a changé. "
                "Actualisez la liste des ports. (" + msg + ")", False)
    return ("Connexion impossible : " + msg, False)


EMPTY_PORT_HELP = (
    "Pistes, de la plus fréquente à la plus rare :\n"
    "1. Utilisez un câble USB qui transmet des données (certains ne servent qu'à charger).\n"
    "2. Gestionnaire de périphériques Windows > « Ports (COM) » : un port CP210x ou CH340 doit "
    "apparaître quand vous branchez l'ESP32.\n"
    "3. Si la carte est sous « Autres périphériques » avec un triangle jaune, installez le pilote "
    "CP210x (Silicon Labs) ou CH340 (WCH).\n"
    "4. Débranchez, rebranchez, puis appuyez une fois sur le bouton EN de la carte.\n"
    "5. Cliquez sur « Actualiser »."
)


# --------------------------------------------------------------------------
# Sources de scans (threads) : elles déposent des événements dans une queue
#   ("scan", rows) | ("bytes", total, tail_text) | ("error", message) | ("closed", None)
# --------------------------------------------------------------------------
class SerialSource(threading.Thread):
    def __init__(self, port, baud, events, tele_flip_y=True):
        super().__init__(daemon=True)
        self.port, self.baud, self.events = port, baud, events
        self.parser = StreamParser(lambda rows: events.put(("scan", rows)), tele_flip_y)
        self._stop_event = threading.Event()
        self.total_bytes = 0
        self.tail = ""

    def stop(self):
        self._stop_event.set()

    def run(self):
        import serial
        try:
            link = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as exc:                       # noqa: BLE001 - on explique l'erreur à l'écran
            self.events.put(("open_error", exc))
            return
        self.events.put(("opened", self.port))
        last_report = 0.0
        try:
            while not self._stop_event.is_set():
                try:
                    data = link.read(4096)
                except Exception as exc:               # câble retiré en cours de route
                    self.events.put(("error", f"Lecture interrompue : {exc}"))
                    break
                if not data:
                    continue
                self.total_bytes += len(data)
                text = data.decode("utf-8", errors="replace")
                self.tail = (self.tail + text)[-900:]
                self.parser.feed(text)
                now = time.monotonic()
                if now - last_report > 0.3:
                    last_report = now
                    self.events.put(("bytes", (self.total_bytes, self.tail)))
        finally:
            try:
                link.close()
            except Exception:                          # noqa: BLE001
                pass
            self.events.put(("bytes", (self.total_bytes, self.tail)))
            self.events.put(("closed", None))


class SimulatedSource(threading.Thread):
    """Lidar virtuel dans la salle 6 x 4 m de make_synthetic.py (4 scans par seconde)."""

    def __init__(self, get_pose, events, period=0.25):
        super().__init__(daemon=True)
        self.get_pose, self.events, self.period = get_pose, events, period
        self._stop_event = threading.Event()
        self.rng = np.random.default_rng(1)

    def stop(self):
        self._stop_event.set()

    def run(self):
        from make_synthetic import simulate_scan
        while not self._stop_event.is_set():
            angles, ranges = simulate_scan(self.get_pose(), self.rng)
            rows = [[a, r * 1000.0, 120.0] for a, r in zip(angles, ranges) if not np.isnan(r)]
            self.events.put(("scan", rows))
            self._stop_event.wait(self.period)
        self.events.put(("closed", None))


def new_event_queue():
    return queue.Queue()
