# TP SLAM – Jour 1 : extraction, visualisation, cartographie

Outils Python (numpy + matplotlib) pour exploiter le lidar LD06 : acquisition, nuages de points,
grille d'occupation, segments et carte vectorielle.

## Installation (Windows)

```powershell
cd TP_SLAM_Jour1
python -m venv .venv
.venv\Scripts\activate
python -m pip install -r requirements.txt
python -m pytest -q            # tous les tests doivent passer
```

## Essayer tout de suite, sans robot

```powershell
python make_synthetic.py                         # crée data_synth/ (4 scans simulés d'une salle)
python viewer.py data_synth --poses poses.json   # visualiseur : bouger les scans à la main
python gridmap_tool.py build data_synth --poses data_synth\poses_true.json --out carte_grille --steps
python vectormap_tool.py data_synth --poses data_synth\poses_true.json --out carte_vect
```

Tu peux aussi dessiner une salle dans `Tools/MapTool/MapTool.html`, exporter le scan
(`scan_lidar.csv`) et le charger directement : les formats MapTool et RobotControl sont reconnus.

## Page web : `lidar_web.html` (le plus simple pour acquérir et visualiser)

Un seul fichier, sans installation : **double-clic**, à ouvrir avec **Chrome ou Edge** (pas dans une fenêtre
intégrée à une application, pas Firefox ni Safari : ils n'ont pas l'accès série).

La colonne de gauche est une **chronologie en 5 étapes**. L'étape en cours est jaune et ouverte, les étapes
terminées passent en bleu avec une coche, les suivantes restent grisées. Chaque étape se replie ou se déplie d'un clic.

1. **Connecter le lidar** (USB de l'ESP32, moniteur série d'Arduino fermé). L'image s'affiche en direct.
   Marche avec le sketch tel quel (`printScanTeleplot`) ou avec `printScanCSV` (recommandé, plus fiable).
2. **Capturer des scans** : indiquer la position du robot, puis Capturer (bouton ou Espace). Deux façons de la donner :
   *depuis l'origine* (x, y, θ dans le repère du premier scan) ou *depuis le scan précédent* (avant, gauche, rotation
   mesurés dans le repère du robot au scan précédent ; la page calcule la position absolue).
3. **Placer les scans** : onglet **Carte**, « Déplacer le scan choisi » puis glisser (Maj + glisser = tourner),
   ou régler les poses dans la liste, ou flèches / `,` `.`.
4. **Régler les cartes** (facultatif) : tolérances de la grille et des segments.
5. **Exporter** : `.zip` avec `scans/*.csv`, `poses.json`, `carte_vectorielle.txt`, `grille.csv`, `carte.png`,
   `projet.json` (ré-importable). Les outils Python lisent directement ce dossier :
   `python gridmap_tool.py build scans --poses poses.json --out carte`.

Le bouton **Simulation** permet d'essayer la page sans robot.

**Si « Connecter le lidar » ne trouve pas le robot** : cliquer sur **Diagnostic** (en haut à gauche). La page teste
le navigateur et explique l'erreur exacte. Pistes, de la plus fréquente à la plus rare : câble USB « charge seulement » ;
pilote CP210x ou CH340 absent (Gestionnaire de périphériques → Ports COM) ; port tenu par le moniteur série d'Arduino ;
page ouverte dans une fenêtre intégrée au lieu de Chrome/Edge. Si la page est ouverte depuis un fichier et que rien
ne réagit, lancer `python -m http.server` dans ce dossier et ouvrir `http://localhost:8000/lidar_web.html`.
Une fois connecté, « Données reçues » (étape 1) montre les derniers octets reçus : du texte lisible = bon câble et
bonne vitesse ; des caractères étranges = mauvaise vitesse ; rien = le sketch n'envoie pas (bouton EN de la carte).
Les réglages « Décalage » et « Miroir » corrigent le montage du capteur.

## Application de bureau : `lidar_app.py` (à ouvrir dans VS Code)

La même démarche que la page web, dans une vraie fenêtre Python (Tkinter + matplotlib, rien d'autre à installer
que `requirements.txt`). Elle réutilise directement `gridmap.py`, `segments.py` et `vectormap.py` : la carte que
tu vois est calculée par le code du TP.

**Dans VS Code**
1. `Fichier > Ouvrir le dossier…` : choisir `TP_SLAM_Jour1`. VS Code propose d'installer l'extension Python : accepter.
2. Terminal VS Code : `python -m venv .venv`, puis `.venv\Scripts\activate`, puis `python -m pip install -r requirements.txt`
   (ou `Ctrl+Maj+P > Tasks: Run Task > Installer les dépendances`). Choisir l'interpréteur `.venv` en bas à droite.
3. **F5** (ou l'onglet « Exécuter et déboguer » > « Lidar : lancer l'application ») ouvre la fenêtre.
   `Ctrl+Maj+P > Tasks: Run Task > Lancer les tests` lance les tests.

**Dans la fenêtre** : mêmes 5 étapes que la page web (chronologie à gauche, vue Direct / Carte à droite).
- Étape 1 : la liste des ports série se remplit toute seule (CP210x, CH340…). Pas de robot : « Simulation sans robot ».
  Si rien n'est détecté, le bouton **Diagnostic** liste les pistes (câble « charge seulement », pilote, port occupé).
- Étape 2 : position du robot *depuis l'origine* (x, y, θ) ou *depuis le précédent* (avant, gauche, rotation).
  Espace = capturer. Chaque scan a sa couleur.
- Étape 3 : onglet Carte, « Déplacer le scan choisi », glisser (Maj : tourner). Molette : zoom. Flèches, `,` `.` : réglage fin.
- Étape 5 : `.zip` (ou dossier) avec `scans/*.csv`, `poses.json`, `projet.json`, `carte_vectorielle.txt`, `grille.csv`, `carte.png`.
  Les fichiers sont lisibles par `gridmap_tool.py`, `vectormap_tool.py` et par la page web (import de `projet.json`).

Pour intégrer l'application à un autre projet, copier `lidar_app.py`, `lidar_widgets.py`, `lidar_plot.py`,
`lidar_session.py`, `lidar_stream.py` et les modules du TP qu'ils importent (`geometry.py`, `lidar_io.py`,
`gridmap.py`, `segments.py`, `vectormap.py`, `make_synthetic.py` pour la simulation).

## Ordre de travail conseillé aujourd'hui

1. **Acquisition (point 1)** – `capture_serial.py`
   - Dans `ESP32_LD06_Test.ino`, remplacer `ld06.printScanTeleplot(Serial);` par
     `ld06.printScanCSV(Serial);` et téléverser. Fermer le moniteur série de l'IDE Arduino.
   - `python capture_serial.py --port COM5 --n 5 --out scans_salle --prefix pos1`
   - Robot **immobile** pendant la capture. Refaire pour plusieurs positions (`pos2`, `pos3`…).
   - **Noter pour chaque position** la pose réelle approximative (x, y en m, orientation en °)
     avec un mètre ruban : elle sert de pose de départ pour la suite.
2. **Visualisation (point 2)** – `viewer.py scans_salle` puis ajuster les poses, touche `s` pour sauver.
3. **Grille (point 3)** – `gridmap_tool.py` (`scan2grid`, `update`, `build`).
4. **Segments (point 4)** – `segments.py` + `tests/test_segments.py`.
5. **Carte vectorielle (point 5)** – `vectormap_tool.py`.
6. **Compte rendu** – compléter `RAPPORT_Jour1.md` avec TES résultats réels.

## Contenu

| Fichier | Rôle |
| --- | --- |
| `geometry.py` | rotations, passage repère lidar → monde (pose = x, y en m, theta en °) |
| `lidar_io.py` | lecture des scans (ESP32 `printScanCSV`, MapTool, RobotControl), poses JSON |
| `lidar_app.py` | application de bureau (fenêtre) : lidar en direct, capture, poses, cartes, export |
| `lidar_widgets.py`, `lidar_plot.py` | éléments d'interface et dessins de l'application |
| `lidar_session.py`, `lidar_stream.py` | données d'un relevé, poses relatives, export ; lecture du port série (sans interface) |
| `.vscode/` | lancement par F5, tâches, tests, extensions conseillées |
| `lidar_web.html` | page web autonome : lidar en direct, capture, poses, grille, segments, export .zip |
| `capture_serial.py` | enregistre les scans reçus sur le port série de l'ESP32 (alternative en ligne de commande) |
| `viewer.py` | affiche plusieurs scans (couleurs distinctes), pose réglable par scan |
| `gridmap.py`, `gridmap_tool.py` | carte d'occupation : création depuis un scan, mise à jour |
| `segments.py` | extraction de segments, angle, proximité, fusion, intersection, découpe |
| `vectormap.py`, `vectormap_tool.py` | carte vectorielle |
| `make_synthetic.py` | simule un lidar dans une salle pour tester sans robot |
| `tests/` | tests unitaires et de bout en bout |
| `exemples_synthetiques/` | images obtenues sur les données simulées (pas des données réelles) |

## Raccourcis du visualiseur

Boutons radio : scan actif · flèches : déplace de 5 cm (Maj : 1 cm) · `,` et `.` : tourne de 1° (Maj : 0,2°) ·
`Tab` : scan suivant · `s` : sauver les poses · `r` : remettre la pose du scan à zéro.

## Git

```powershell
git init
git add .
git commit -m "Jour 1 : outils lidar, grille d'occupation, segments, carte vectorielle"
```
Fais plusieurs commits (un par étape) : c'est ce qui montre la progression.

## Conventions

Pose `(x, y, theta)` : mètres et degrés, X vers la droite, Y vers le haut, angle positif anti-horaire,
theta = 0 vers +X (conventions de « Utiliser les outils du module »). Le LD06 tourne dans le sens
horaire : on convertit avec `theta = 360 - theta_LD06` (déjà fait par la librairie ESP32).
