"""Acquisition des scans du LD06 via le port série de l'ESP32.

Pré-requis :
  1. Dans ESP32_LD06_Test.ino, remplacer dans loop() :
         ld06.printScanTeleplot(Serial);
     par :
         ld06.printScanCSV(Serial);
     puis téléverser (le moniteur série de l'IDE Arduino doit être FERMÉ pendant la capture).
  2. pip install pyserial
  3. Trouver le port : Gestionnaire de périphériques Windows > Ports (COM) , ex. COM5

Utilisation :
    python capture_serial.py --port COM5 --n 5 --out scans_salle --prefix pos1
        -> enregistre 5 scans consécutifs : scans_salle/pos1_000.csv ... pos1_004.csv

Astuce : le robot doit rester IMMOBILE pendant la capture d'une position.
Pour chaque position du robot dans la salle, lancez une capture (--prefix différent) et
notez la pose approximative (x, y, orientation) mesurée au mètre / au ruban.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from lidar_io import _make_scan, save_scan


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="ex. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--n", type=int, default=5, help="nombre de scans à enregistrer")
    parser.add_argument("--out", default="scans", help="dossier de sortie")
    parser.add_argument("--prefix", default="scan", help="préfixe des fichiers")
    parser.add_argument("--min-points", type=int, default=100,
                        help="un scan avec moins de points est ignoré (tour incomplet)")
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        import serial
    except ImportError:
        sys.exit("pyserial manquant : pip install pyserial")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    link = serial.Serial(args.port, args.baud, timeout=1.0)
    print(f"Port {args.port} ouvert à {args.baud} bauds. Attente des scans...")

    rows = []
    saved = 0
    start = time.time()
    while saved < args.n:
        if time.time() - start > args.timeout:
            sys.exit(f"Timeout : {saved}/{args.n} scans reçus. "
                     "Vérifier le port, le baudrate et que printScanCSV est bien activé.")
        raw = link.readline().decode("utf-8", errors="replace").strip()

        if raw == "":
            # ligne vide = fin d'un scan
            if len(rows) >= args.min_points:
                angle = [r[1] for r in rows]
                dist = [r[2] / 1000.0 for r in rows]
                inten = [r[3] for r in rows]
                scan = _make_scan(f"{args.prefix}_{saved:03d}", angle, dist, inten)
                path = out / f"{scan.name}.csv"
                save_scan(scan, path)
                print(f"scan {saved + 1}/{args.n} : {len(scan)} points -> {path}")
                saved += 1
            elif rows:
                print(f"(scan incomplet ignoré : {len(rows)} points)")
            rows = []
            continue

        fields = raw.split(",")
        try:
            values = [float(f) for f in fields]
        except ValueError:
            continue                     # en-tête ou ligne de debug
        if len(values) >= 4:
            rows.append(values)

    link.close()
    print("Terminé.")


if __name__ == "__main__":
    main()
