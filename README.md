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

1. **Connecter le lidar** : en **USB** (moniteur série d'Arduino fermé) ou en **Wi-Fi** (voir plus bas). L'image s'affiche en direct.
   En USB, marche avec le sketch tel quel (`printScanTeleplot`) ou avec `printScanCSV` (recommandé, plus fiable).
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
5. **Grille d'occupation 2D** : onglet **Grille 2D**. La grille sans filtre (à gauche) et avec filtre (à droite) se comparent avec un curseur de séparation ; un second curseur règle la force du filtre ; la taille des carreaux se saisit en millimètres (entier, 10 à 500). Un tableau compte les cellules occupées et celles qui sont isolées. Les cellules **jamais vues par le lidar** (derrière un mur ou un meuble, hors de la pièce) sont dessinées en **gris foncé** (v28), les cellules vues et libres restent claires et les obstacles en noir.
6. **Localisation** : choisir le scan à localiser, puis l'un des trois onglets. La page affiche **x, y et θ** (et applique la pose au scan ; *Annuler* la rétablit).
   - **Grille** : la grille est faite avec les autres scans, ou **importée** (`grille.csv` exporté par la page : une ligne par rangée, le haut en premier,
     `-1` inconnu, `0` libre, `≥ 1` occupé). L'origine et le carreau sont repris du nom `grille_50mm_x-1.63_y-3.90.csv` (bouton « Grille ») ou de `grille.json` (.zip) ; sinon les taper.
   - **Vectorielle** : segments faits avec les autres scans, ou **importés** (`carte_vectorielle.txt` : une ligne `x1,y1,x2,y2` par segment, en mètres).
   - **Scan à scan** : recalage sur le scan précédent, une carte locale de N scans, un scan choisi ou tous les autres (voir ci-dessous).
   - Pendant la localisation, **le fond de la carte change** : grille, segments, ou scans de référence en couleur (les autres effacés), avec le scan à localiser par-dessus et un bandeau « Localisation sur : … ».
   - *Chercher* : **partout** sur la carte, ou près d'une position approximative (plus rapide, lève les ambiguïtés des pièces symétriques).
   - **Scanner avec le robot** (v15, en haut de l'étape) : le robot fait un scan (le prochain tour reçu en USB, Wi-Fi ou simulation), la page le cherche
     sur la carte de l'onglet choisi (grille, vectorielle, ou tous les scans) et affiche **x, y, θ** avec un verdict (Fiable / À vérifier / Ambigu).
     Le robot s'affiche en orange sur la carte, avec un trait vers chaque point qu'il mesure (ce qu'il voit dans la pièce) ; la vue bascule d'elle-même sur l'onglet Carte. Le scan du robot n'est pas ajouté à la liste des scans. Avec « Près de… » (onglets Grille et Vectorielle),
     la recherche est plus rapide quand on sait à peu près où il est ; sinon elle couvre toute la carte.
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

### Connexion Wi-Fi (v14)

L'ESP32 crée son propre réseau (point d'accès). Sur l'ordinateur : se connecter à ce réseau (il perd alors internet), ouvrir
`lidar_web.html` **depuis le fichier** (une page https bloque `ws://`), puis **Connecter en Wi-Fi**. L'adresse par défaut est
`192.168.4.1` (port 81 du WebSocket ajouté tout seul) ; elle est mémorisée par le navigateur. Si Chrome demande l'autorisation
d'accéder au réseau local, l'accepter.

Un message WebSocket = un tour complet. Formats lus : JSON `[{"a":angle°,"d":distance mm,"i":intensité}, ...]`
(l'intensité `i` est facultative ; une virgule en trop avant `]` est tolérée) ou texte CSV / Teleplot comme en USB.
Au moins 50 points par tour. Si la liaison tombe, la page retente 10 fois toutes les 2 s. Sans `i`, tous les points ont
l'intensité 255 : le filtre d'intensité n'a alors rien à filtrer. Le mot de passe du réseau n'est jamais demandé ni stocké.

#### Coupures du Wi-Fi (v33)
Si le lien tombe pendant un essai :
- **La page** retente pendant une minute (30 essais, un toutes les 2 s), détecte un lien « ouvert mais muet » (5 s sans aucun message : PC parti sur un autre réseau, carte plantée) et rouvre la connexion tout de suite. Elle compte les coupures (« Liaison : 12 ms · 2 coupures ») et indique la cause probable (code 1006 = coupure sans prévenir : portée, interférences ou carte redémarrée). Si le SLAM tournait, le robot reste arrêté : clique « Passer en automatique » ou « Suivre le trajet » pour reprendre, la carte est conservée.
- **Le sketch** envoie à chaque connexion la durée de fonctionnement de la carte (`E,U`) et la raison de son dernier démarrage (`E,B`) : si la carte a **redémarré par chute de tension (brownout)** quand les moteurs ont démarré, la page l'affiche en rouge (« Robot redémarré : chute de tension »). C'est une cause fréquente de coupure en roulant : batteries faibles, ESP32 alimenté par la même ligne que les moteurs, pas de condensateur sur le 5 V. Le moniteur série affiche aussi la raison au démarrage et le nombre de clients à chaque déconnexion.
- Le sketch détecte un lien mort en ~16 s (ping WebSocket toutes les 10 s) et le robot s'arrête de toute façon dès que les ordres cessent (250 ms). Canal Wi-Fi fixé (`WIFI_CANAL`, 6 par défaut : essaie 1 ou 11 si un autre réseau est proche), 2 appareils au plus, puissance d'émission maximale.
- Si la page ne dit rien de précis : colle-moi la ligne « Liaison : … » et les messages du moniteur série au moment de la coupure.
Le sketch utilise `enableHeartbeat` : il faut la bibliothèque WebSockets de Markus Sattler en version 2.3 ou plus récente.

### Piloter le robot (v16)

Connecté en Wi-Fi, un **pavé de pilotage** apparaît en bas à droite de la vue Direct : on voit le lidar en direct pendant qu'on conduit.
Boutons (à tenir appuyés) ou clavier (case **Clavier**, cochée par défaut) : ↑ ↓ avancer / reculer, ← → tourner, Maj + ← → (ou Q / D) décaler sur le côté,
**Espace** arrêt. Les flèches combinées donnent les diagonales (↑ + Maj + ←, etc.).

Commandes envoyées par le même WebSocket que le lidar : `D,vx,vy,w,v` (vx > 0 avance, vy > 0 décale à gauche, w > 0 tourne à gauche ; −1, 0 ou 1 ; v = vitesse en %, curseur **Vitesse**, 20 à 100),
répétée toutes les 60 ms tant qu'une touche est tenue, et `S` pour arrêter. Un arrêt est aussi envoyé au relâchement, à la perte de focus de la fenêtre,
au changement d'onglet et à la déconnexion. Le sketch doit en plus s'arrêter seul si les ordres cessent (250 ms dans le sketch fourni).
Le curseur **Vitesse** (v18) règle la vitesse des 4 roues : si les petits moteurs ne démarrent pas, il faut l'augmenter.

**Pas à pas (v19)** : la case **Pas à pas** du pavé fait qu'un appui sur une touche (ou un clic) = **un seul petit déplacement** ; garder la touche enfoncée n'en refait pas.
Le curseur **Durée du pas** (50 à 600 ms) règle la longueur d'un pas ; avec le curseur Vitesse, on dose l'avance à chaque appui. La page envoie `P,vx,vy,w,v,ms`
et c'est l'ESP32 qui coupe les moteurs à la fin des `ms` millisecondes (30 à 1000), sans dépendre de la latence du Wi-Fi. **Espace** (ou ■) envoie `S`.

**Réactivité (v23)** : un tour de lidar fait ~15 Ko ; son envoi bloquait la boucle de l'ESP32 pendant que le Wi-Fi le prenait, donc les ordres arrivaient en retard et l'arrêt aussi
(le robot continuait à rouler ou à tourner après le relâchement). Le sketch a maintenant :
- un **minuteur matériel** (toutes les 10 ms, sans `delay()`) qui coupe les moteurs si le dernier ordre a plus de 250 ms (+ 100 ms) ou si la durée d'un pas `P` est écoulée, même si la boucle principale est bloquée ;
- un **envoi du lidar adaptatif** : en roulant, 3 tours par seconde et sans l'intensité ; si un envoi dure plus de 40 ms (Wi-Fi lent), les suivants sont espacés de 3 fois cette durée ; dès que le robot s'arrête, le tour suivant part tout de suite (le SLAM l'attend) ;
- `WiFi.setSleep(false)` (l'économie d'énergie du Wi-Fi ajoute des dizaines de ms) ;
- une **rotation adoucie** : les ordres de rotation `D` et `P` sont multipliés par `ECHELLE_ROTATION` (0,5 ; 1,0 = pleine vitesse), la rotation étant bien plus vive que l'avance.
La page n'empile plus d'ordres `D` périmés si la liaison est engorgée (l'arrêt `S` passe toujours) et affiche la **latence** sous le pavé : elle envoie `T,n` toutes les 1,5 s, le sketch répond `E,T,n` à ce seul client.

**Profils de vitesse (sketch, idée reprise du projet MecanumWheelRobot)** : les roues ne passent plus de 0 à pleine puissance d'un coup. Le sketch applique une rampe fondée sur l'horloge (aucun `delay()`) entre la vitesse demandée et celle des roues : **courbe en S** par défaut (départ et arrivée doux), montée de 150 ms, freinage de 100 ms. Les rouleaux mecanum patinent moins, le robot dérive moins de son cap et les déplacements sont plus prévisibles pour la page. Un ordre `D` identique répété (toutes les 60 ms) ne relance pas la rampe ; `S` et la fin d'un pas `P` freinent en douceur. Les **arrêts de sécurité restent immédiats** (coupe-circuit de 250 ms, minuteur de 10 ms, déconnexion, pilotage ultrason). Commande `R,profil[,montée_ms[,freinage_ms]]` : profil 0 = carré (ancien comportement, aucune rampe), 1 = trapézoïdal (rampe linéaire), 2 = courbe en S. À téléverser pour en profiter ; la page n'a rien à changer, elle calibre toute seule les vitesses mesurées.
Au-delà de 250 ms d'aller-retour, la page signale « Liaison lente » : rapproche-toi du robot, évite d'autres appareils sur le réseau Robot-LiDAR.

### SLAM automatique (v20)

L'onglet **SLAM** et l'étape **SLAM automatique** du menu : on pilote le robot avec le pavé (mode continu ou **Pas à pas**) et la carte se construit toute seule.
Après chaque arrêt du robot (commande terminée + « Attente après l'arrêt », 700 ms par défaut), la page prend un tour de lidar complet, le recale sur les derniers scans fiables
(`trackerStep` : carte locale de N scans, mouvement prévu, recherche élargie si le lien est perdu) et l'ajoute à la carte avec sa position `x, y, θ`.
Le premier scan définit l'origine ; si des scans existent déjà, le SLAM continue à leur suite. Un scan dont la position est introuvable n'est pas ajouté ; un scan pris alors que
le robot n'a presque pas bougé (« Garder si bougé de », 3 cm) est ignoré ; un recalage peu sûr est signalé. Boutons : **Scanner maintenant**, **Annuler le dernier scan**, **Recommencer**, **Voir la carte**.
Autres déclencheurs (Réglages) : toutes les N secondes, ou seulement à la demande. Pendant le SLAM, la grille et la carte vectorielle ne sont refaites qu'en quittant l'onglet.

#### Option : SLAM automatique piloté par le lidar (v22)

Le pilotage manuel reste la façon de faire par défaut. En option, le bouton **SLAM automatique** (onglet SLAM, ou étape du menu) lance le SLAM et laisse la page conduire le robot **d'après le lidar** (le capteur ultrason n'intervient pas).
La page décide, le sketch exécute : il suffit de la commande `P,vx,vy,w,v,ms` déjà présente (un pas, arrêté par l'ESP32 lui-même), rien à modifier ni à téléverser en plus.

À chaque arrêt du robot (scan recalé sur la carte, comme en manuel) :
1. une carte d'exploration (cases de 10 cm : inconnu, libre, obstacle) est refaite à partir de tous les scans placés ;
2. la cible est la zone inconnue la plus proche par un chemin libre (frontière libre / inconnu, au moins 0,4 m de large) ; quand il n'en reste plus, un point de vue encore éloigné de tous les scans (réglage « écart entre points de vue », 2 m par défaut, 0 = aucun) pour voir derrière les meubles ;
3. **conduite « voiture » (v25)** : en espace dégagé, le robot **se tourne d'abord vers la destination** (rotations de 35° au plus par pas, la durée est réglée sur la rotation réellement mesurée), puis **avance droit sur les 4 roues** (flux d'ordres `D,1,0,0,v` toutes les 50 ms, sur la plus longue ligne droite libre, 50 cm au plus par défaut). Pendant ce trajet la page regarde le lidar en direct et coupe dès qu'un obstacle approche ou si le lidar ne répond plus (le sketch coupe aussi de lui-même si les ordres cessent). Les **déplacements latéraux et en diagonale** (pas `P` de 40 cm au plus parmi les 8 directions, avec les roues mecanum : le robot garde son cap) ne servent qu'en **zone resserrée** (un obstacle à moins de 65 cm tout autour) ou quand aucune ligne droite n'est libre ; il faut toujours que le lidar mesure assez de place devant (marge de 25 cm entre le lidar et l'obstacle) ;
4. la durée de chaque pas est ajustée sur le déplacement réellement mesuré (les deux premiers pas sont courts) et la position attendue après le pas (consigne × vitesses mesurées) est donnée au recalage, ce qui le rend bien plus fiable après une rotation ou une longue ligne droite.
   **Repli (v29)** : si la rotation sur place ne fait pas tourner le robot (3 essais, de plus en plus longs), la page passe aux pas latéraux comme avant la v25 ; si la ligne droite ne le fait pas avancer, elle passe aux petits pas. Un message l'indique dans l'onglet SLAM.

**La base (v23)** : le premier scan du SLAM définit l'origine de la carte, et c'est la **base** du robot, son point de départ. Elle est marquée en permanence (maison orange « Base ») sur SLAM, Carte et Grille 2D, et la barre d'état de l'onglet SLAM donne la distance du robot à la base.
En automatique, une fois la pièce scannée (plus de zone inconnue ni de point de vue à ajouter), ou quand une limite est atteinte (scans, durée), **le robot rentre à sa base** par un chemin libre, puis s'arrête à moins de 25 cm de la base (« Exploration terminée : … revenu à sa base (à 11 cm du départ) »). Réglage « Auto : revenir à la base à la fin » (Oui par défaut ; Non : arrêt direct à la fin ou à la limite). Le cap n'est pas rétabli : le robot garde son cap pendant l'exploration, l'écart est annoncé s'il dépasse 5°.

**Filtres (v26)**, réglables (Oui / Non) dans Réglages du SLAM, actifs par défaut :
- *Filtre du bruit lidar* : sur chaque scan pris par le SLAM, avant le recalage, un point isolé (seul à sortir de la ligne tracée par ses deux voisins) est retiré et le tremblement de 1 à 2 cm est lissé (moyenne 1-2-1 sur les points alignés ; coins et pentes préservés). Le scan en direct qui arrête le robot devant un obstacle n'est jamais filtré, pour ne pas effacer un obstacle fin. Les scans enregistrés dans la carte sont ceux qui ont été filtrés.
- *Filtre de Kalman* : filtre la position (x, y, cap) en mélangeant la position prévue par la consigne (vitesses mesurées) et la position recalée sur le lidar, chacune pondérée par son incertitude. Un recalage « fiable » est gardé tel quel (le lidar est plus précis que la consigne des roues : le corriger abîmait la carte, voir v27), seul un recalage « à vérifier » est tiré vers la position prévue ; l'incertitude (± cm) est affichée dans la barre de l'onglet SLAM.
L'effet est donc nul quand le recalage est bon ; le filtre ne joue que dans les endroits où il hésite.

Le trajet prévu (pointillés) et la cible s'affichent sur la carte. L'exploration s'arrête seule : retour à la base une fois plus de zone ni de point de vue à voir (« Exploration terminée »), 250 scans, 30 minutes (le robot rentre d'abord à sa base), 3 recalages ratés de suite
(le robot revient d'un pas, puis s'arrête s'il est perdu), 3 pas sans déplacement (robot coincé), lien Wi-Fi coupé, ou ordres incohérents avec ce que voit le lidar (avant ↔ arrière ou gauche ↔ droite inversés : lidar monté tourné, moteurs inversés).
Une touche du pavé, le bouton ■ ou **Reprendre la main** rendent la main (le SLAM continue en manuel) ; **Passer en automatique** reprend l'exploration.
Réglages (Réglages du SLAM) : pas latéral maximal, ligne droite maximale, marge autour du robot, écart entre points de vue, nombre de scans et durée maximale.
Limites : le lidar voit à hauteur de son plan seulement (pas les pieds de table ni un obstacle plus bas) et le robot n'a pas de capteur de choc ; testé en simulation (salle de 6 × 4 m avec pilier et meuble, robot virtuel lent, normal et rapide, 0 collision) et avec un faux lien Wi-Fi, mais pas encore sur le vrai robot : premier essai à vitesse faible, sous surveillance.

**Moins de pas latéraux (v31)** : en automatique le robot ne passait en pas latéraux que « en zone resserrée », mais le critère (un mur à moins de 65 cm, n'importe où autour) était vrai presque tout le temps, et les pas diagonaux n'utilisent que 2 roues sur 4. Désormais la zone resserrée ne commence qu'à ~37 cm d'un obstacle, et les pas latéraux se font d'abord en avant / arrière / gauche / droite (4 roues) ; une diagonale n'est utilisée que si aucun de ces 4 pas ne convient. Le reste du temps : rotation sur place puis ligne droite sur 4 roues.

**Trajets plus longs en zone déjà cartographiée (v32)** : là où la carte est déjà faite (au moins 80 % des cases vues dans un rayon de 60 cm), le scan ne sert plus qu'à se repérer, donc le robot fait des lignes droites jusqu'à **150 cm** entre deux scans (réglage « ligne droite en zone déjà cartographiée », 0 = désactivé, au lieu de 50 cm) et des rotations jusqu'à 80° par pas (au lieu de 35°). Cela ne joue qu'après quelques pas (vitesses calibrées), et pas si le robot vient de ne pas bouger. En zone inconnue, rien ne change : 50 cm et 35° maximum, un scan à chaque pas. Mesure en simulation : un tour de 12 m dans la pièce déjà scannée passe de 49 à 32 pas et de 121 s à 103 s (le reste est le temps de roulage), pose toujours à 1–6 cm près, 0 collision.

#### Suivi de trajectoire (v30, v31, v32)
Dans l'onglet SLAM, sous les boutons de l'automatique : **Tracer un trajet**, puis dessine le chemin à la souris sur la carte (maintiens le bouton, trace, relâche, comme un crayon sur Paint). Le trait est simplifié (un point au plus tous les 30 cm) et s'affiche en orange avec un drapeau à l'arrivée. **Suivre le trajet** lance le robot (le SLAM démarre tout seul s'il ne l'était pas ; connexion Wi-Fi du robot nécessaire). **Retracer un trajet** ou **Effacer le trajet** remplacent ou retirent le tracé ; **Arrêter le suivi**, une touche du pavé ou **Reprendre la main** rendent la main.
Le robot conduit comme en exploration automatique (il se tourne, puis avance droit sur les 4 roues ; pas latéraux seulement en zone resserrée), mais sa cible est le prochain point du trajet. À chaque pas, le chemin est recalculé sur la carte du lidar : un obstacle ou un meuble sur le trait est contourné, puis le robot rejoint la suite du tracé. Un point qui tombe dans un meuble ou un mur est sauté (v31 : le robot ne s'acharne plus devant, seul le dernier point est approché au plus près ; v32 : un point dépassé est considéré atteint, et un trajet qui revient à son point de départ n'est plus écrasé à l'import) (« Trajet terminé : … 3 points sautés »). La partie déjà faite passe en vert. Le suivi s'arrête à l'arrivée (rayon 20 cm), sur la durée maximale des réglages, ou dans les mêmes cas que l'automatique (robot perdu, bloqué, Wi-Fi coupé). Pas de limite de scans en suivi, ni de retour à la base.
Testé en simulation : trait droit à travers le pilier (contourné, 0 collision, arrivée à 19 cm), à travers le meuble (points sautés, arrivée à 28 cm), robot lent avec latence et dérive (arrivée à 11 cm, 0 collision). Pas encore essayé sur le vrai robot.

### Pilotage automatique (capteur ultrason + servo)

Matériel : HC-SR04 (TRIG = D23, ECHO = D22, 5 V) et micro servo 9 g sur D26 (5 V), le servo porte le capteur (90° = droit devant).
**Pont diviseur sur ECHO** (1 kΩ entre l'écho et D22, 2 kΩ entre D22 et GND) : l'écho sort en 5 V, l'ESP32 supporte 3,3 V.
Bouton **Pilotage automatique** du pavé (ou `A,1,v,pause` / `A,0` sur le WebSocket, touche `a` au moniteur série) ; une touche ou un bouton du pavé reprend la main, ainsi qu'une déconnexion de la page.

Algorithme (dans le sketch, sans `delay()`, le temps d'écho est relevé par interruption) : il avance par segments de 0,9 s en ralentissant sous 70 cm, s'arrête sous 30 cm ou à la fin d'un segment,
regarde à gauche puis à droite avec le servo (±55°, médiane de 3 mesures). Obstacle devant : il tourne vers le côté le plus dégagé jusqu'à ce que le devant soit libre à 55 cm
(il recule 0,5 s si les deux côtés sont à moins de 25 cm). Devant libre mais un mur à moins de 35 cm sur un côté : il glisse sur le côté (roues mecanum) pour s'en écarter.
Il ne roule jamais sans réponse du capteur et abandonne après 4 s de silence (capteur débranché). Constantes en tête du bloc : distances, angles du servo, `SERVO_SENS_GAUCHE` (à passer à −1 si le servo regarde du mauvais côté).
Le sketch prévient la page (`E,M` il bouge, `E,R` il est arrêté, `E,D,cm` distance devant) : avec le **SLAM** en marche, un scan est pris à chaque arrêt de l'automate.
Limite : un capteur ultrason ne voit pas bien une surface très inclinée ou molle, et regarde un seul faisceau.

**Placer les scans** : « Recaler le scan choisi » (comparé au scan juste avant) et « Recaler tous à la suite » refont le recalage par rapport aux scans précédents (même moteur que Localisation › Scan à scan, qui garde les autres références et la recherche « Partout »).
Les étapes « Grille d'occupation 2D » et « Régler les cartes » ont été retirées du menu (l'onglet Grille 2D et les valeurs par défaut des cartes restent).
Espace arrête le robot sans plus déclencher une capture de scan.

#### Sketch ESP32 : `arduino/ESP32_Robot_WiFi_Pilotage/`

Lidar en direct par WebSocket (port 81), réception des ordres `D,vx,vy,w,v` / `P,vx,vy,w,v,ms` / `S`, arrêt automatique 300 ms sans ordre. Aucun `delay()` bloquant : le test des roues (`t`), `f`, le pas à pas et l'init de la carte I2C passent par des minuteurs dans `loop()` (seul reste un `delayMicroseconds(5)` dans l'I2C simulé).
Les moteurs reçoivent le **PWM directement sur les 8 broches** (M1A = D5, M1B = D18, M2A = D19, M2B = D21, M3A = D2, M3B = D4, M4A = D16, M4B = D17 ;
M1 avant droit, M2 avant gauche, M3 arrière gauche, M4 arrière droit). Les montages I2C (bibliothèque `MecanumCar_v2`, ou I2C plus lent) restent
sélectionnables pour comparer. Le sketch n'est pas encore validé sur le robot : le sens de chaque moteur se règle avec les cases « inverser » de la page de test.

**Page de test PWM** : connecté au réseau `Robot-LiDAR`, ouvrir `http://192.168.4.1/` : choix du mode, 8 curseurs (M1A à M4B), avancer / reculer, un moteur à la fois,
case « inverser » par moteur (enregistrée dans l'ESP32, conservée après redémarrage ; par défaut M2 et M3, les moteurs de gauche, sont inversés), boutons de mouvement (avancer, translations, rotations, 1 s) pour valider la
matrice mecanum au sol. Tout s'arrête seul 3 s après la perte de la liaison.
Bibliothèques : LD06 (archive du TP), WebSockets (Markus Sattler), MecanumCar_v2 (module).

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
| `tests/` | tests du noyau de calcul de la page (Node), dont la lecture des messages Wi-Fi |
| `RAPPORT_Jour1.md` | compte rendu (parties à compléter avec les résultats réels) |

## Tests

Les tests chargent le noyau de calcul directement depuis `lidar_web.html`. Il faut [Node.js](https://nodejs.org) (aucune autre installation) :

```powershell
node tests/run_all.js
```

Ils vérifient la géométrie des segments, la chaîne scans → grille et carte vectorielle, les 14 méthodes de localisation, le recalage automatique et la lecture des messages Wi-Fi.

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
