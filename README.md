# TP SLAM – cartographie et localisation avec le lidar LD06

Tout le TP tient dans **une seule page web autonome** : `lidar_web.html`. Elle lit le lidar LD06 branché à l'ESP32
(ou une simulation), capture des scans, les place, construit la grille d'occupation et la carte vectorielle,
compare des méthodes de localisation et retrouve la position d'un scan à partir d'un autre. Rien à installer :
**double-clic**, dans Chrome ou Edge.

## Essayer tout de suite, sans robot

1. Ouvrir `lidar_web.html`, étape 1, bouton **Simulation** (salle simulée de 6 × 4 m).
2. Ou **importer** les scans d'exemple du dossier `data_synth/` (4 scans simulés d'une salle ; `poses_true.json` contient leurs vraies poses) :
   étape 3, « Importer des fichiers », puis onglets Carte et Grille 2D. Pour la localisation : onglet Localisation,
   « Mes scans », « Importer poses.json ».
3. Tu peux aussi dessiner une salle dans `Tools/MapTool/MapTool.html`, exporter le scan (`scan_lidar.csv`) et le charger :
   les formats MapTool et RobotControl sont reconnus.

## La page `lidar_web.html`

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
   **Scan à scan** (étape 6, onglet « Scan à scan », ou case « Placer chaque nouveau scan par recalage » à l'étape 2) :
   la page recale le scan choisi dans la liste, sans qu'on mesure sa position.
   - **Référence : le scan précédent (par défaut).** Chaque scan est placé par rapport à celui d'avant : c'est la méthode adaptée
     à un robot qui scanne en avançant. La liste déroulante permet de choisir un autre scan (le 1er, un scan précis), « Tous les autres scans »,
     ou **« Carte locale (N derniers scans) »** : le scan est recalé sur la carte fusionnée des N derniers scans *fiables* (N = 5 conseillé).
     En simulation (bruit 1 cm, obstacle mobile) la carte locale donne ~0,5 cm d'erreur moyenne contre 1 à 2 cm avec le scan précédent seul ; sur des scans réels
     (22 scans, ~250 à 900 points, recouvrement de 40 à 90 %) les deux donnent une carte aussi cohérente (85 % des points sur un mur, 1,2 cm) : le scan précédent reste le défaut.
     La recherche est bornée par le rayon et l'angle demandés. Si le lien est perdu (robot déplacé de plusieurs mètres, scans manquants) ou peu sûr (recouvrement
     < 60 % ou position au bord de la fenêtre), la page cherche **partout** sur les scans déjà fiables (≈ 1–2 s) et garde cette position si elle est nettement meilleure.
     Le « mouvement prévu » (même pas que le précédent) départage deux positions aussi bonnes (le scan est alors marqué « À vérifier »). « Fiable » dès 35 % de recouvrement.
   - **Où chercher** : *Près de la référence* (par défaut : autour de la pose du scan de référence, rayon 1,5 m et ±45°, à adapter à la distance
     réellement parcourue entre deux scans), *Près de la pose saisie* (autour de la position tapée dans la liste) ou *Partout* (6 m, tous les angles :
     dans une salle rectangulaire la symétrie peut tromper, la page affiche alors **Ambigu** et propose les autres positions).
   - **Recaler tous les scans** (facultatif) : avec « le scan précédent » ou la carte locale, recalage en chaîne du 1er au dernier (avec le scan précédent seul, une erreur se propage
     aux scans suivants ; avec la carte locale elle se dilue ; un scan « Ambigu » ou en « Échec » ne sert jamais de référence, et un second essai, plus large, est tenté ;
     vérifier d'abord les lignes « À vérifier » ou « Ambigu »). Avec un scan choisi, tous les scans sont recalés sur lui et sur ceux déjà trouvés fiables.
   - Case de l'étape 2 (désactivée par défaut) : chaque scan capturé est recalé aussitôt sur le précédent.
   - Verdicts : *Fiable* (bon recouvrement, une seule solution), *À vérifier* (recouvrement faible), *Ambigu* (plusieurs solutions aussi bonnes),
     *Échec* (les scans ne se recoupent pas : la pose n'est pas modifiée). Le bouton **Annuler** rétablit les anciennes positions.
   - En simulation : chaîne de 14 scans espacés de 0,6 m, erreur finale 0,5 cm ; tout reste fiable jusqu'à 1,2 m / 25° entre deux scans.
   - Limites : les deux scans doivent voir une partie commune de la pièce ; si le robot a bougé plus que le rayon ou l'angle de recherche, la bonne
     position n'est pas trouvée (augmenter les valeurs) ; un capteur collé à un mur ou un couloir sans relief laisse une position possible le long du mur.
     Toujours vérifier sur l'onglet Carte. Non validé avec le vrai lidar.
   Les `scans/*.csv` exportés sont filtrés et commencent par une ligne `# pose: x=… y=… theta=…` : la position suit le fichier et est retrouvée à l'import ; `projet.json` reste brut.
5. **Grille d'occupation 2D** : onglet **Grille 2D**. La grille sans filtre (à gauche) et avec filtre (à droite) se comparent avec un curseur de séparation ; un second curseur règle la force du filtre ; la taille des carreaux se saisit en millimètres (entier, 10 à 500). Un tableau compte les cellules occupées et celles qui sont isolées.
6. **Localisation** : choisir le scan à localiser, puis l'un des trois onglets. La page affiche **x, y et θ** (et applique la pose au scan ; *Annuler* la rétablit).
   - **Grille** : la grille est faite avec les autres scans, ou **importée** (`grille.csv` exporté par la page : une ligne par rangée, le haut en premier,
     `-1` inconnu, `0` libre, `≥ 1` occupé). L'origine et le carreau sont repris du nom `grille_50mm_x-1.63_y-3.90.csv` (bouton « Grille ») ou de `grille.json` (.zip) ; sinon les taper.
   - **Vectorielle** : segments faits avec les autres scans, ou **importés** (`carte_vectorielle.txt` : une ligne `x1,y1,x2,y2` par segment, en mètres).
   - **Scan à scan** : recalage sur le scan précédent, une carte locale de N scans, un scan choisi ou tous les autres (voir ci-dessous).
   - Pendant la localisation, **le fond de la carte change** : grille, segments, ou scans de référence en couleur (les autres effacés), avec le scan à localiser par-dessus et un bandeau « Localisation sur : … ».
   - *Chercher* : **partout** sur la carte, ou près d'une position approximative (plus rapide, lève les ambiguïtés des pièces symétriques).
   - *Comparer les méthodes* (replié) : le banc d'essai du Jour 2 (14 méthodes, vérité terrain, essais multiples, courbe de sensibilité ; onglet Localisation).
   - Seuls x, y et θ sont utilisés (pas de hauteur z). En simulation : 0 à 2 cm et moins de 2° sur des scans absents de la carte (≈ 2 s partout, 0,4 s près d'une position).
7. **Exporter** : `.zip` avec `scans/*.csv`, `poses.json`, `carte_vectorielle.txt`, `grille.csv` (+ `grille.json` : carreau et origine), `carte.png`,
   `projet.json` (ré-importable dans la page).

Le bouton **Simulation** permet d'essayer la page sans robot.

**Si « Connecter le lidar » ne trouve pas le robot** : cliquer sur **Diagnostic** (en haut à gauche). La page teste
le navigateur et explique l'erreur exacte. Pistes, de la plus fréquente à la plus rare : câble USB « charge seulement » ;
pilote CP210x ou CH340 absent (Gestionnaire de périphériques → Ports COM) ; port tenu par le moniteur série d'Arduino ;
page ouverte dans une fenêtre intégrée au lieu de Chrome/Edge. Si la page est ouverte depuis un fichier et que rien
ne réagit, ouvrir la page par un petit serveur local (par exemple `python -m http.server` ou `npx serve` dans ce dossier, puis `http://localhost:8000/lidar_web.html`).
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

Tests du noyau (Node) : `node tests/web_loc_test.js lidar_web.html` (150 vérifications) et `node tests/web_reg_test.js lidar_web.html` (40 vérifications, recalage automatique et carte locale) et `node tests/web_grid_test.js lidar_web.html` (localisation sur grille et carte vectorielle importées).

## Ordre de travail conseillé

1. **Acquisition** : dans `ESP32_LD06_Test.ino`, remplacer `ld06.printScanTeleplot(Serial);` par `ld06.printScanCSV(Serial);`
   et téléverser. Fermer le moniteur série d'Arduino. Étape 1 de la page. Robot **immobile** pendant la capture ; refaire pour plusieurs positions.
2. **Visualisation et placement** : étapes 2 et 3 (onglet Carte). Noter la pose approximative de chaque position (mètre ruban), ou utiliser le recalage automatique.
3. **Grille d'occupation, segments, carte vectorielle** : étapes 4 et 5 (onglets Carte et Grille 2D).
4. **Localisation (Jour 2)** : étape 6.
5. **Compte rendu** : compléter `RAPPORT_Jour1.md` avec TES résultats réels.

## Contenu

| Fichier | Rôle |
| --- | --- |
| `lidar_web.html` | la page : lidar en direct, capture, poses, grille, segments, filtre, localisation, recalage, export .zip |
| `data_synth/` | 4 scans simulés et leurs vraies poses, pour essayer sans robot |
| `exemples_synthetiques/` | images obtenues sur les données simulées (pas des données réelles) |
| `tests/` | tests du noyau de calcul de la page (Node) |
| `RAPPORT_Jour1.md` | compte rendu (parties à compléter avec les résultats réels) |

## Tests

Les tests chargent le noyau de calcul directement depuis `lidar_web.html`. Il faut [Node.js](https://nodejs.org) (aucune autre installation) :

```powershell
node tests/run_all.js
```

Ils vérifient la géométrie des segments, la chaîne scans → grille et carte vectorielle, les 14 méthodes de localisation et le recalage automatique.

## Git

```powershell
git status                     # vérifier ce qui sera ajouté
git add lidar_web.html README.md RAPPORT_Jour1.md tests
git commit -m "Description courte de la version"
git push
```
Un commit par version de la page (v1, v2, …) montre la progression : voir `git log --oneline`.

## Conventions

Pose `(x, y, theta)` : mètres et degrés, X vers la droite, Y vers le haut, angle positif anti-horaire,
theta = 0 vers +X (conventions de « Utiliser les outils du module »). Le LD06 tourne dans le sens
horaire : on convertit avec `theta = 360 - theta_LD06` (déjà fait par la librairie ESP32).
