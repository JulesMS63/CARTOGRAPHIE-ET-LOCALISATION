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

La colonne de gauche est une **chronologie en 7 étapes**. L'étape en cours est jaune et ouverte, les étapes
terminées passent en bleu avec une coche, les suivantes restent grisées. Chaque étape se replie ou se déplie d'un clic.

1. **Connecter le lidar** (USB de l'ESP32, moniteur série d'Arduino fermé). L'image s'affiche en direct.
   Marche avec le sketch tel quel (`printScanTeleplot`) ou avec `printScanCSV` (recommandé, plus fiable).
2. **Capturer des scans** : indiquer la position du robot, puis Capturer (bouton ou Espace). Deux façons de la donner :
   *depuis l'origine* (x, y, θ dans le repère du premier scan) ou *depuis le scan précédent* (avant, gauche, rotation
   mesurés dans le repère du robot au scan précédent ; la page calcule la position absolue).
3. **Placer les scans** : onglet **Carte**, « Glisser : le scan choisi » puis glisser (Maj + glisser = tourner),
   ou régler les poses dans la liste, ou flèches / `,` `.`.
4. **Régler les cartes** (facultatif) : tolérances de la grille et des segments, et filtre des points isolés.
   La case **Ignorer les points isolés** (au-dessus de la vue, cochée par défaut) retire les points seuls ou en tout
   petit groupe et ceux collés au capteur : la carte est plus nette. Réglages : « Points min. par groupe » (8),
   « Distance min. au capteur » (0,20 m), « Écart toléré dans un groupe » (0,10 m) et, en option, « Exiger un 2e scan »
   (un point n'est gardé que si un autre scan le revoit à moins de 0,10 m : demande des poses justes). Les scans bruts
   sont conservés : décocher la case les réaffiche.
   Les `scans/*.csv` exportés sont filtrés et commencent par une ligne `# pose: x=… y=… theta=…` : la position suit le fichier et est retrouvée à l'import ; `projet.json` reste brut.
5. **Grille d'occupation 2D** : onglet **Grille 2D**. La grille sans filtre (à gauche) et avec filtre (à droite) se comparent avec un curseur de séparation ; un second curseur règle la force du filtre ; la taille des carreaux se saisit en millimètres (entier, 10 à 500). Un tableau compte les cellules occupées et celles qui sont isolées.
6. **Localisation (Jour 2)** : onglet **Localisation**, voir la section ci-dessous.
7. **Exporter** : `.zip` avec `scans/*.csv`, `poses.json`, `carte_vectorielle.txt`, `grille.csv`, `carte.png`,
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

### Jour 2 : localisation (onglet Localisation de `lidar_web.html`)

Le but : retrouver la pose (x, y, θ) d'un scan, de trois façons, et **comparer les méthodes avec les mêmes conditions initiales**.

| Référence (barre d'outils) | Entrée | Méthodes |
| --- | --- | --- |
| **A · Carte d'occupation** | scan + grille | force brute, multi-résolution, filtre à particules (Monte Carlo), descente de gradient |
| **B · Carte vectorielle** | scan + segments | ICP point-à-ligne (calcul direct), force brute, multi-résolution, filtre à particules, descente de gradient |
| **C · Scan à scan** | scan + scan voisin | ICP point-à-point, ICP point-à-ligne, corrélation (force brute), multi-résolution, filtre à particules |

Les 14 méthodes sont opérationnelles. Pour A et B, la carte est construite avec **tous les autres scans** (le scan testé n'y est
jamais : sinon il se retrouverait lui-même). Pour C, la référence est le scan le plus proche (ou celui choisi dans « Scan de référence »).

**Vérité terrain.** Deux sources :
- *Banc d'essai simulé* : une salle connue avec 8 poses exactes, bruit, parasites et graine réglables : à utiliser pour valider.
- *Mes scans* : les poses de la liste (étape 3) servent de vérité. Elles ne sont aussi justes que ton placement : **Figer les poses**
  les copie pour pouvoir ensuite déplacer les scans sans fausser les mesures, ou **Importer poses.json** (format de l'export).

**Pose initiale.** Écart (Δx, Δy, Δθ) ajouté à la vérité, identique pour toutes les méthodes. Les curseurs montrent la pose
initiale sur la carte (« Au hasard » en tire une). Trois modes :
1. *Une localisation* : tableau des erreurs, du temps et du nombre d'évaluations ; un clic sur une ligne affiche la pose trouvée sur la carte.
2. *Essais multiples* : N essais à direction aléatoire, taux de réussite, médianes, temps moyen et maximal.
3. *Sensibilité* : 7 niveaux d'écart croissant (jusqu'à « écart max »), courbe taux de réussite = f(écart initial) par méthode.

**Métriques.** Erreur de position (m), erreur d'angle (°, repliée dans [0, 180]), temps de calcul (ms), taux de réussite.
**Un essai est réussi si erreur de position ≤ 0,05 m ET erreur d'angle ≤ 2°** (seuils réglables). Le temps ne compte que la
localisation, pas la préparation de la carte (affichée à part). Résultats exportables en `localisation_resultats.csv`.

**À savoir.** La force brute n'explore que ±fenêtre (0,5 m / 15° par défaut) autour de la pose initiale : au-delà, elle échoue
par construction. Le filtre à particules et la multi-résolution tolèrent davantage ; l'ICP et le gradient sont rapides (1 à 8 ms)
mais exigent une pose initiale proche. Sur le banc simulé (bruit 2 cm, 6 % de parasites) : tout réussit à 0,1 m / 3° ; l'ICP
tombe à 75 % vers 0,5 m / 15° ; le filtre à particules tient encore à 1,2 m / 40°. **Rien de cela n'a été mesuré avec ton lidar** :
refaire les mesures sur tes scans.

Test du noyau (Node) : `node tests/web_loc_test.js lidar_web.html` (150 vérifications).

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

**Dans la fenêtre** : mêmes 6 étapes que la page web (chronologie à gauche, vue Direct / Carte / Grille 2D à droite).
- Étape 1 : la liste des ports série se remplit toute seule (CP210x, CH340…). Pas de robot : « Simulation sans robot ».
  Si rien n'est détecté, le bouton **Diagnostic** liste les pistes (câble « charge seulement », pilote, port occupé).
- Étape 2 : position du robot *depuis l'origine* (x, y, θ) ou *depuis le précédent* (avant, gauche, rotation).
  Espace = capturer. Chaque scan a sa couleur.
- Étape 3 : onglet Carte, « Glisser : le scan choisi », glisser (Maj : tourner). Molette : zoom. Flèches, `,` `.` : réglage fin.
- Étape 4 : taille des carreaux en **millimètres** (entier de 10 à 500, par défaut 50), filtre des points isolés
  (points min. 8, distance min. 0,20 m, écart de groupe 0,10 m, confirmation par un 2e scan en option).
- Étape 5 : onglet **Grille 2D**. Deux grilles sur la même étendue : *sans filtre* à gauche, *avec filtre* à droite, séparées
  par un curseur (boutons « Comparer / Sans filtre / Avec filtre »). Le curseur **Force du filtre** règle le nombre de points
  minimum d'un groupe (0 = filtre désactivé). Le champ **Carreaux (mm)** affine ou grossit le maillage. Le HUD et l'étape
  comptent les cellules occupées et les cellules *isolées* (sans voisine occupée) : un bon filtre fait tomber les isolées
  sans réduire beaucoup les cellules des murs. « Enregistrer la comparaison (PNG) » sauve l'image pour le rapport.
- Étape 6 : `.zip` (ou dossier) avec `scans/*.csv` (filtrés, chacun commence par la ligne `# pose: x=… y=… theta=…`),
  `poses.json`, `projet.json` (brut), `carte_vectorielle.txt`, `grille.csv`, `carte.png`, et, si la grille 2D a été ouverte,
  `grille2d_comparaison.png` et `grille2d_stats.txt`.
  Les fichiers sont lisibles par `gridmap_tool.py`, `vectormap_tool.py` et par la page web (import de `projet.json`).
- La position d'un scan **suit le fichier** : la ligne `# pose:` est relue à l'import (application, page web et outils
  Python). Un `poses.json` à côté des scans reste prioritaire s'il est fourni.

**En ligne de commande : la grille 2D sans / avec filtre** (`grid2d_tool.py`)
```powershell
python grid2d_tool.py scans_salle --poses poses.json --cell-mm 50 --min-points 8 --sweep-points 1,4,8,12 --out resultats/grille2d
```
Écrit `grille2d_sans_filtre.png`, `grille2d_avec_filtre.png`, `grille2d_comparaison.png`, `grille2d_stats.txt` (tableau
des cellules occupées / isolées / points ignorés) et `grille2d_avec_filtre.csv`. `--sweep-points` teste plusieurs valeurs
de « points min. » et affiche un tableau (à recopier dans le rapport). Autres options : `--min-range`, `--break-min`,
`--confirm --confirm-tol 0.10`, `--no-filter`. Sans `--poses`, la ligne `# pose:` des `.csv` est utilisée.
Exemple sur données simulées : `exemples_synthetiques/grille2d_comparaison.png`.

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
| `lidar_io.py` | lecture des scans (ESP32 `printScanCSV`, MapTool, RobotControl), poses JSON, ligne `# pose:` des .csv |
| `lidar_app.py` | application de bureau (fenêtre) : lidar en direct, capture, poses, cartes, export |
| `lidar_widgets.py`, `lidar_plot.py` | éléments d'interface et dessins de l'application |
| `lidar_session.py`, `lidar_stream.py` | données d'un relevé, poses relatives, export ; lecture du port série (sans interface) |
| `.vscode/` | lancement par F5, tâches, tests, extensions conseillées |
| `lidar_web.html` | page web autonome : lidar en direct, capture, poses, grille, segments, export .zip |
| `capture_serial.py` | enregistre les scans reçus sur le port série de l'ESP32 (alternative en ligne de commande) |
| `viewer.py` | affiche plusieurs scans (couleurs distinctes), pose réglable par scan |
| `gridmap.py`, `gridmap_tool.py` | carte d'occupation : création depuis un scan, mise à jour |
| `scanfilter.py` | filtre des points isolés + confirmation par un 2e scan (même algorithme dans `lidar_web.html`, comparé par les tests) |
| `grid2d.py`, `grid2d_tool.py` | grille d'occupation 2D sans / avec filtre (carreaux en mm, statistiques) et son outil en ligne de commande |
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
git init                       # seulement si le dossier n'est pas déjà un dépôt
git status                     # vérifier ce qui sera ajouté (pas de .venv, pas de __pycache__ : voir .gitignore)
git add .
git commit -m "Jour 1 : acquisition lidar, filtre des points isolés, grille 2D sans/avec filtre, segments, carte vectorielle"
git remote add origin https://github.com/<ton-compte>/<ton-depot>.git    # une seule fois
git push -u origin main
```
Fais plusieurs commits (un par étape) : c'est ce qui montre la progression. Exemple :
```powershell
git add capture_serial.py lidar_io.py viewer.py
git commit -m "Acquisition : lecture et affichage des scans, pose enregistrée avec le scan"
git add scanfilter.py gridmap.py gridmap_tool.py grid2d.py grid2d_tool.py
git commit -m "Grille d'occupation 2D, filtre des points isolés et comparaison sans/avec filtre"
git add segments.py vectormap.py vectormap_tool.py tests
git commit -m "Segments, carte vectorielle et tests"
git add lidar_app.py lidar_widgets.py lidar_plot.py lidar_session.py lidar_stream.py lidar_web.html README.md RAPPORT_Jour1.md
git commit -m "Application de bureau, page web et compte rendu"
git push
```

## Conventions

Pose `(x, y, theta)` : mètres et degrés, X vers la droite, Y vers le haut, angle positif anti-horaire,
theta = 0 vers +X (conventions de « Utiliser les outils du module »). Le LD06 tourne dans le sens
horaire : on convertit avec `theta = 360 - theta_LD06` (déjà fait par la librairie ESP32).
