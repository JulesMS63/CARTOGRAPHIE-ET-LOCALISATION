"""Application de bureau : lidar LD06 en direct, capture de scans, carte, export.

Lancer (depuis le dossier du projet, environnement virtuel activé) :
    python lidar_app.py

Les 6 étapes de la colonne de gauche reprennent la page web `lidar_web.html` :
    1. Connecter le lidar      (port série de l'ESP32, ou simulation sans robot)
    2. Capturer des scans      (position du robot : depuis l'origine ou depuis le scan précédent)
    3. Placer les scans        (onglet Carte : glisser un scan à la souris, ou saisir sa pose)
    4. Régler les cartes       (facultatif : tolérances, taille des carreaux en mm, filtre des points isolés)
    5. Grille d'occupation 2D  (onglet « Grille 2D » : comparer sans / avec filtre grâce à un curseur)
    6. Exporter                (.zip lisible par gridmap_tool.py et vectormap_tool.py)

Le calcul (grille d'occupation, segments, carte vectorielle) vient des modules du TP :
    gridmap.py, segments.py, vectormap.py. L'interface n'ajoute que l'acquisition et l'affichage.
"""

import platform
import queue
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import matplotlib
import numpy as np

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import lidar_plot  # noqa: E402
from grid2d import CELL_MM_MAX, CELL_MM_MIN, clamp_cell_mm, stats_table  # noqa: E402
from lidar_session import MapParams, Session  # noqa: E402
from lidar_stream import (EMPTY_PORT_HELP, SerialSource, SimulatedSource,  # noqa: E402
                          explain_serial_error, list_ports, port_label, pyserial_available,
                          rows_to_scan)
from lidar_widgets import (BAD, BLUE, BLUE_SOFT, CHALK, HAIR, HAIR_STRONG, INK, MIST,  # noqa: E402
                           MUTED, OK, WARN, Fonts, ScrollFrame, Step, make_button, setup_ttk)
from scanfilter import filter_isolated  # noqa: E402
from segments import extract_segments  # noqa: E402

STEPS = [("connect", "Connecter le lidar"), ("capture", "Capturer des scans"),
         ("place", "Placer les scans"), ("maps", "Régler les cartes"),
         ("grid", "Grille d'occupation 2D"), ("export", "Exporter")]
OPTIONAL = {"maps"}
BAUDS = ("115200", "230400", "921600")


def plural(n, one, many):
    return f"{n} {one if n <= 1 else many}"


def digits_only(text):
    """Contrôle de saisie : seulement des chiffres (le champ peut être vide le temps de taper)."""
    return text == "" or text.isdigit()


def to_float(text, default=0.0):
    try:
        return float(str(text).replace(",", "."))
    except ValueError:
        return default


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Lidar LD06 – acquisition et cartographie")
        self.geometry("1380x860")
        self.minsize(1040, 640)
        self.configure(bg=MIST)
        self.fonts = Fonts(self)
        setup_ttk(self, self.fonts)

        self.session = Session()
        self.events = queue.Queue()
        self.source = None
        self.live = None
        self.live_segs = []
        self.clean_var = tk.BooleanVar(value=True)      # ignorer les points isolés
        self._live_used = None              # (scan brut, réglages, scan filtré) : évite de refiltrer à chaque dessin
        self.live_times = []
        self.last_live_draw = 0.0
        self.conn_state = "idle"            # idle | live | sim | error
        self.port_name = ""
        self.bytes_total = 0
        self.got_scan = False
        self.queue_left = 0
        self.batch_pose = None
        self.sim_pose = (0.0, 0.0, 0.0)
        self.flags = dict(connected_once=False, map_seen=False, posed=False, tuned=False, exported=False,
                          grid_seen=False)
        self.tab = "live"
        self.selected = -1
        self.map_view = None
        self.grid_view = None               # cadrage de la grille 2D (None = ajusté au contenu)
        self.grid_art = {}                  # éléments graphiques de la comparaison (pour le curseur)
        self.last_pair = None
        self.strength_job = None
        self._grid_sync = False             # vrai pendant qu'on règle les curseurs par programme
        self.artists = {}
        self.drag = None
        self.rebuild_job = None
        self.toast_job = None
        self.watchdogs = []
        self.current_step = None
        self._updating_vars = False
        self.pose_vars = []
        self.ports = []

        self._build_ui()
        self.refresh_ports()
        self.update_pose_ui()
        self.render_scan_list()
        self.set_tab("live")
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind_all("<Key>", self.on_key)
        self.after(30, self.poll)
        self.after(150, self.warn_missing_serial)

    # ------------------------------------------------------------------
    # Construction de l'interface
    # ------------------------------------------------------------------
    def _build_ui(self):
        f = self.fonts
        left = tk.Frame(self, bg=MIST, width=440)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)
        tk.Frame(self, bg=HAIR, width=1).pack(side="left", fill="y")

        header = tk.Frame(left, bg=MIST)
        header.pack(fill="x", padx=16, pady=(16, 6))
        tk.Label(header, text="Lidar LD06", font=f.brand, bg=MIST, fg=INK).pack(side="left")
        make_button(header, "Diagnostic", self.open_diagnostic, "quiet", f).pack(side="right")

        pill = tk.Frame(left, bg=CHALK, highlightthickness=1, highlightbackground=HAIR)
        pill.pack(fill="x", padx=16, pady=(4, 10))
        self.led = tk.Canvas(pill, width=16, height=16, bg=CHALK, highlightthickness=0)
        self.led.pack(side="left", padx=(10, 6), pady=7)
        self.pill_text = tk.Label(pill, text="Non connecté", font=f.bold, bg=CHALK, fg=INK)
        self.pill_text.pack(side="left")
        for w in (pill, self.led, self.pill_text):
            w.bind("<Button-1>", lambda e: self.open_step("connect"))

        self.scroll = ScrollFrame(left)
        self.scroll.pack(fill="both", expand=True)
        self.steps = {}
        for i, (key, title) in enumerate(STEPS):
            step = Step(self.scroll.inner, key, i + 1, title, f, optional=key in OPTIONAL,
                        is_last=i == len(STEPS) - 1)
            step.pack(fill="x", padx=(10, 0))
            step.on_toggle = lambda s: self.after(50, lambda: self.scroll.scroll_to(s))
            self.steps[key] = step
        self._build_connect(self.steps["connect"].body)
        self._build_capture(self.steps["capture"].body)
        self._build_place(self.steps["place"].body)
        self._build_maps(self.steps["maps"].body)
        self._build_grid(self.steps["grid"].body)
        self._build_export(self.steps["export"].body)

        right = tk.Frame(self, bg=MIST)
        right.pack(side="left", fill="both", expand=True)
        self._build_toolbar(right)
        self._build_stage(right)

    # ---- petits utilitaires de mise en page ----
    def lab(self, parent, text, muted=True, small=False, wrap=0, **grid):
        w = tk.Label(parent, text=text, bg=CHALK, fg=MUTED if muted else INK, anchor="w", justify="left",
                     font=self.fonts.small if small else self.fonts.text, wraplength=wrap)
        return w

    def hint(self, parent, text):
        return self.lab(parent, text, small=True, wrap=310)

    def status_label(self, parent):
        return tk.Label(parent, text="", bg=CHALK, fg=MUTED, anchor="w", justify="left",
                        font=self.fonts.small, wraplength=310)

    def set_status(self, label, text, kind=""):
        label.configure(text=text, fg={"ok": OK, "err": BAD, "warn": WARN}.get(kind, MUTED))

    def entry(self, parent, var, width=8):
        return ttk.Entry(parent, textvariable=var, width=width)

    def field(self, parent, text, widget_factory, col, row=0, pad=(0, 8)):
        box = tk.Frame(parent, bg=CHALK)
        box.grid(row=row, column=col, sticky="ew", padx=pad)
        parent.columnconfigure(col, weight=1)
        tk.Label(box, text=text, bg=CHALK, fg=MUTED, font=self.fonts.small).pack(anchor="w")
        w = widget_factory(box)
        w.pack(fill="x", pady=(2, 0))
        return w

    # ---- étape 1 : connexion ----
    def _build_connect(self, body):
        f = self.fonts
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=BAUDS[0])
        row = tk.Frame(body, bg=CHALK)
        row.pack(fill="x", pady=(0, 8))
        row.columnconfigure(0, weight=1)
        self.lab(row, "Port de l'ESP32", small=True).grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, width=24)
        self.port_combo.grid(row=1, column=0, sticky="ew")
        make_button(row, "Actualiser", self.refresh_ports, "default", f).grid(row=1, column=1, padx=(6, 0))
        self.lab(row, "Vitesse (bauds)", small=True).grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(row, textvariable=self.baud_var, values=BAUDS, width=10, state="readonly") \
            .grid(row=3, column=0, sticky="w")

        btns = tk.Frame(body, bg=CHALK)
        btns.pack(fill="x", pady=(2, 6))
        self.btn_connect = make_button(btns, "Connecter le lidar", self.connect, "primary", f)
        self.btn_connect.pack(side="left")
        self.btn_disconnect = make_button(btns, "Déconnecter", self.disconnect, "default", f)
        self.btn_disconnect.pack(side="left", padx=6)
        self.btn_disconnect.configure(state="disabled")
        self.btn_sim = make_button(body, "Simulation sans robot", self.toggle_sim, "default", f)
        self.btn_sim.pack(anchor="w", pady=(0, 6))

        self.conn_status = self.status_label(body)
        self.conn_status.pack(fill="x")
        self.help_btn = make_button(body, "Ouvrir le diagnostic", self.open_diagnostic, "default", f)
        self.hint(body, "Fermez le moniteur série d'Arduino avant de connecter. Le sketch doit appeler "
                        "printScanCSV (conseillé) ou printScanTeleplot.").pack(fill="x", pady=(6, 4))

        self.mon_open = tk.BooleanVar(value=False)
        ttk.Checkbutton(body, text="Afficher les données reçues", variable=self.mon_open,
                        command=self._toggle_monitor).pack(anchor="w")
        self.mon_frame = tk.Frame(body, bg=CHALK)
        self.mon_text = tk.Text(self.mon_frame, height=6, width=40, font=f.mono, bg=MIST, fg=MUTED,
                                relief="flat", wrap="char", highlightthickness=1,
                                highlightbackground=HAIR)
        self.mon_text.pack(fill="x")
        self.mon_bytes = self.lab(self.mon_frame, "0 octet reçu", small=True)
        self.mon_bytes.pack(anchor="w")

        self.orient_open = tk.BooleanVar(value=False)
        ttk.Checkbutton(body, text="Orientation du capteur", variable=self.orient_open,
                        command=self._toggle_orient).pack(anchor="w", pady=(4, 0))
        self.orient_frame = tk.Frame(body, bg=CHALK)
        self.tele_flip = tk.BooleanVar(value=True)
        self.mirror = tk.BooleanVar(value=False)
        self.offset_var = tk.StringVar(value="0")
        ttk.Checkbutton(self.orient_frame, text="Teleplot : annuler le signe « − » de y de la librairie",
                        variable=self.tele_flip).pack(anchor="w")
        r = tk.Frame(self.orient_frame, bg=CHALK)
        r.pack(fill="x", pady=4)
        self.lab(r, "Décalage (°)", small=True).pack(side="left")
        self.entry(r, self.offset_var, 6).pack(side="left", padx=6)
        ttk.Checkbutton(r, text="Miroir", variable=self.mirror).pack(side="left", padx=8)
        self.hint(self.orient_frame, "À régler avec un obstacle de position connue : un carton à gauche "
                                     "du robot doit apparaître à gauche.").pack(fill="x")

    def _toggle_monitor(self):
        if self.mon_open.get():
            self.mon_frame.pack(fill="x", pady=(4, 0), after=self._mon_anchor())
        else:
            self.mon_frame.pack_forget()

    def _mon_anchor(self):
        return [w for w in self.steps["connect"].body.winfo_children() if isinstance(w, ttk.Checkbutton)][0]

    def _toggle_orient(self):
        if self.orient_open.get():
            self.orient_frame.pack(fill="x", pady=(2, 0))
        else:
            self.orient_frame.pack_forget()

    # ---- étape 2 : capture ----
    def _build_capture(self, body):
        f = self.fonts
        self.prefix_var = tk.StringVar(value="pos1")
        self.ncap_var = tk.StringVar(value="1")
        g = tk.Frame(body, bg=CHALK)
        g.pack(fill="x")
        self.field(g, "Nom de la position", lambda p: ttk.Entry(p, textvariable=self.prefix_var), 0)
        self.field(g, "Tours à capturer", lambda p: ttk.Spinbox(p, from_=1, to=50, width=5,
                                                                textvariable=self.ncap_var), 1, pad=(0, 0))

        self.lab(body, "Position du robot pour ce scan", muted=False).pack(anchor="w", pady=(12, 4))
        self.pose_mode = tk.StringVar(value="abs")
        seg = tk.Frame(body, bg=CHALK)
        seg.pack(fill="x")
        ttk.Radiobutton(seg, text="Depuis l'origine", variable=self.pose_mode, value="abs",
                        command=self.on_mode_change).pack(side="left")
        ttk.Radiobutton(seg, text="Depuis le précédent", variable=self.pose_mode, value="rel",
                        command=self.on_mode_change).pack(side="left", padx=12)

        self.cap_vars = [tk.StringVar(value="0") for _ in range(3)]
        self.cap_labels = [tk.StringVar() for _ in range(3)]
        gp = tk.Frame(body, bg=CHALK)
        gp.pack(fill="x", pady=(8, 0))
        for k in range(3):
            box = tk.Frame(gp, bg=CHALK)
            box.grid(row=0, column=k, sticky="ew", padx=(0, 6 if k < 2 else 0))
            gp.columnconfigure(k, weight=1)
            tk.Label(box, textvariable=self.cap_labels[k], bg=CHALK, fg=MUTED, font=f.small).pack(anchor="w")
            ttk.Entry(box, textvariable=self.cap_vars[k], width=7).pack(fill="x", pady=(2, 0))
            self.cap_vars[k].trace_add("write", lambda *a: self.update_pose_ui())
        self.pose_hint = self.hint(body, "")
        self.pose_hint.pack(fill="x", pady=(6, 6))

        row = tk.Frame(body, bg=CHALK)
        row.pack(fill="x")
        self.btn_capture = make_button(row, "Capturer (Espace)", self.capture_now, "primary", f)
        self.btn_capture.pack(side="left")
        self.freeze = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text="Figer l'affichage", variable=self.freeze).pack(side="left", padx=12)
        self.cap_status = self.status_label(body)
        self.cap_status.pack(fill="x", pady=(6, 0))
        self.hint(body, "Le robot reste immobile pendant la capture. Origine : le repère du premier scan. "
                        "Depuis le précédent : avant, gauche et rotation sont mesurés dans le repère du robot "
                        "au scan précédent. La pose se corrige ensuite à l'étape 3.").pack(fill="x", pady=(4, 0))

    # ---- étape 3 : placement ----
    def _build_place(self, body):
        f = self.fonts
        make_button(body, "Ouvrir l'onglet Carte", lambda: self.set_tab("map"), "default", f).pack(anchor="w")
        self.scan_box = tk.Frame(body, bg=CHALK)
        self.scan_box.pack(fill="x", pady=8)
        row = tk.Frame(body, bg=CHALK)
        row.pack(fill="x")
        make_button(row, "Importer des fichiers", self.import_files, "default", f).pack(side="left")
        make_button(row, "Tout effacer", self.clear_all, "danger", f).pack(side="left", padx=6)
        self.hint(body, "Dans l'onglet Carte, cochez « Glisser : le scan choisi », puis faites glisser "
                        "(Maj enfoncée : tourner). Flèches, « , » et « . » affinent la pose.").pack(fill="x", pady=(8, 0))

    # ---- étape 4 : cartes ----
    def _build_maps(self, body):
        f = self.fonts
        self.res_var = tk.StringVar(value="50")                 # taille des carreaux, en mm (entier)
        self.forget_var = tk.StringVar(value="1")
        self.ang_var = tk.StringVar(value="10")
        self.dist_var = tk.StringVar(value="0.06")
        self.gap_var = tk.StringVar(value="0.15")
        self.minw_var = tk.StringVar(value="1")
        self.minpts_var = tk.StringVar(value="8")
        self.minrange_var = tk.StringVar(value="0.20")
        self.brk_var = tk.StringVar(value="0.10")
        self.conf_tol_var = tk.StringVar(value="0.10")
        self.confirm_var = tk.BooleanVar(value=False)
        g = tk.Frame(body, bg=CHALK)
        g.pack(fill="x")
        specs = [("Carreaux (mm, entier)", self.res_var),
                 ("Oubli par scan", self.forget_var),
                 ("Écart d'angle (°)", self.ang_var), ("Écart latéral (m)", self.dist_var),
                 ("Trou admis (m)", self.gap_var), ("Observations min.", self.minw_var),
                 ("Points min. par groupe", self.minpts_var), ("Distance min. (m)", self.minrange_var),
                 ("Écart de groupe (m)", self.brk_var), ("Confirmation (m)", self.conf_tol_var)]
        for i, (text, var) in enumerate(specs):
            def make(p, var=var, first=(i == 0)):
                return self.cell_entry(p, 8) if first else ttk.Entry(p, textvariable=var, width=8)
            w = self.field(g, text, make, i % 2, row=i // 2, pad=(0, 8 if i % 2 == 0 else 0))
            handler = self.on_cell_change if i == 0 else self.on_param_change
            w.bind("<Return>", lambda e, h=handler: h())
            w.bind("<FocusOut>", lambda e, h=handler: h())
        ttk.Checkbutton(body, text="Exiger un 2e scan (confirmation)",
                        variable=self.confirm_var, command=self.on_param_change).pack(anchor="w", pady=(10, 0))
        self.auto_build = tk.BooleanVar(value=True)
        ttk.Checkbutton(body, text="Recalculer à chaque changement", variable=self.auto_build).pack(anchor="w", pady=(10, 4))
        make_button(body, "Recalculer les cartes", self.rebuild, "default", f).pack(anchor="w")
        self.map_status = self.status_label(body)
        self.map_status.pack(fill="x", pady=(6, 0))
        self.hint(body, "Filtre des points isolés (case « Ignorer les points isolés » au-dessus de la vue) : "
                        "un groupe de points voisins plus petit que « Points min. » est ignoré, comme tout point "
                        "plus proche que la distance min. Carreaux : entier de " f"{CELL_MM_MIN} à {CELL_MM_MAX} mm. "
                        "La confirmation ne garde que les points revus par un autre scan (poses justes nécessaires). Les scans bruts sont conservés ; projet.json reste brut."
                   ).pack(fill="x", pady=(0, 4))
        self.hint(body, "Cette étape est facultative : les valeurs par défaut fonctionnent pour une salle "
                        "de classe.").pack(fill="x")

    # ---- étape 5 : grille d'occupation 2D ----
    def cell_entry(self, parent, width=5):
        """Champ « taille des carreaux » : un entier en millimètres (la même valeur partout)."""
        return ttk.Entry(parent, textvariable=self.res_var, width=width, validate="key",
                         validatecommand=(self.register(digits_only), "%P"))

    def _build_grid(self, body):
        f = self.fonts
        make_button(body, "Ouvrir la grille 2D", lambda: self.set_tab("grid"), "primary", f).pack(anchor="w")
        row = tk.Frame(body, bg=CHALK)
        row.pack(fill="x", pady=(10, 0))
        tk.Label(row, text="Carreaux (mm)", bg=CHALK, fg=MUTED, font=f.small).pack(side="left")
        w = self.cell_entry(row, 6)
        w.pack(side="left", padx=8)
        w.bind("<Return>", lambda e: self.on_cell_change())
        w.bind("<FocusOut>", lambda e: self.on_cell_change())
        self.grid_stats_lbl = tk.Label(body, text="", bg=CHALK, fg=INK, anchor="w", justify="left",
                                       font=f.mono)
        self.grid_stats_lbl.pack(fill="x", pady=(10, 0))
        make_button(body, "Enregistrer la comparaison (PNG)", self.export_grid_png, "default", f).pack(anchor="w", pady=8)
        self.hint(body, "Deux grilles sur la même étendue : sans filtre à gauche, avec filtre à droite. Le curseur du "
                        "bas déplace la séparation ; « Force du filtre » règle le nombre de points minimum d'un "
                        "groupe (0 : filtre désactivé). Un bon filtre fait tomber les cellules isolées sans "
                        "entamer les murs.").pack(fill="x")

    # ---- étape 6 : export ----
    def _build_export(self, body):
        f = self.fonts
        make_button(body, "Tout exporter (.zip)", self.export_zip, "primary", f).pack(anchor="w")
        row = tk.Frame(body, bg=CHALK)
        row.pack(fill="x", pady=8)
        for text, cmd in (("Dossier", self.export_folder), ("Carte vectorielle", self.export_vec),
                          ("Grille", self.export_grid), ("Image", self.export_png)):
            make_button(row, text, cmd, "default", f).pack(side="left", padx=(0, 6))
        self.hint(body, "Le .zip contient scans/*.csv, poses.json, la carte vectorielle, la grille, une image "
                        "et un projet.json ré-importable. Les outils Python le lisent directement : "
                        "python gridmap_tool.py build scans --poses poses.json --out carte").pack(fill="x")

    # ---- barre d'outils et scène ----
    def _build_toolbar(self, parent):
        f = self.fonts
        bar = tk.Frame(parent, bg=CHALK, highlightthickness=1, highlightbackground=HAIR)
        bar.pack(fill="x")
        seg = tk.Frame(bar, bg=MIST, highlightthickness=1, highlightbackground=HAIR_STRONG)
        seg.pack(side="left", padx=14, pady=9)
        self.tab_btns = {}
        for key, text in (("live", "Direct"), ("map", "Carte"), ("grid", "Grille 2D")):
            b = tk.Button(seg, text=text, font=f.bold, relief="flat", bd=0, padx=14, pady=6, cursor="hand2",
                          takefocus=0, command=lambda k=key: self.set_tab(k))
            b.pack(side="left", padx=2, pady=2)
            self.tab_btns[key] = b

        self.live_opts = tk.Frame(bar, bg=CHALK)
        self.opt_live_seg = tk.BooleanVar(value=False)
        self.opt_intensity = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.live_opts, text="Segments", variable=self.opt_live_seg,
                        command=self.on_live_option).pack(side="left", padx=6)
        ttk.Checkbutton(self.live_opts, text="Intensité", variable=self.opt_intensity,
                        command=self.redraw).pack(side="left", padx=6)

        self.map_opts = tk.Frame(bar, bg=CHALK)
        self.lay_pts, self.lay_grid, self.lay_seg = (tk.BooleanVar(value=True) for _ in range(3))
        self.mode_var = tk.StringVar(value="view")
        for text, var in (("Points", self.lay_pts), ("Grille", self.lay_grid), ("Segments", self.lay_seg)):
            ttk.Checkbutton(self.map_opts, text=text, variable=var, command=self.redraw).pack(side="left", padx=6)
        tk.Frame(self.map_opts, bg=HAIR, width=1, height=22).pack(side="left", padx=10)
        tk.Label(self.map_opts, text="Glisser :", font=f.small, bg=CHALK, fg=MUTED).pack(side="left", padx=(0, 0))
        ttk.Radiobutton(self.map_opts, text="la vue", variable=self.mode_var, value="view").pack(side="left", padx=4)
        ttk.Radiobutton(self.map_opts, text="le scan choisi", variable=self.mode_var, value="scan").pack(side="left", padx=6)
        self.grid_opts = tk.Frame(bar, bg=CHALK)
        self.grid_mode = tk.StringVar(value="cmp")
        for text, value in (("Comparer", "cmp"), ("Sans filtre", "raw"), ("Avec filtre", "flt")):
            ttk.Radiobutton(self.grid_opts, text=text, variable=self.grid_mode, value=value,
                            command=self.on_grid_mode).pack(side="left", padx=5)
        tk.Frame(self.grid_opts, bg=HAIR, width=1, height=22).pack(side="left", padx=10)
        tk.Label(self.grid_opts, text="Carreaux (mm)", font=f.small, bg=CHALK, fg=MUTED).pack(side="left")
        w = self.cell_entry(self.grid_opts, 5)
        w.pack(side="left", padx=6)
        w.bind("<Return>", lambda e: self.on_cell_change())
        w.bind("<FocusOut>", lambda e: self.on_cell_change())
        make_button(bar, "Ajuster", self.fit_view, "default", f).pack(side="right", padx=14)

    def _build_stage(self, parent):
        f = self.fonts
        self.stage = tk.Frame(parent, bg=lidar_plot.CANVAS)
        self.stage.pack(fill="both", expand=True)
        self.fig = Figure(figsize=(7, 6), facecolor=lidar_plot.CANVAS)
        self.ax = self.fig.add_axes([0.06, 0.07, 0.92, 0.91])
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.stage)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect("scroll_event", self.on_scroll)

        self.hud = tk.Label(self.stage, text="", font=f.small, bg=CHALK, fg=MUTED, padx=12, pady=7,
                            highlightthickness=1, highlightbackground=HAIR, justify="left")
        self.coords = tk.Label(self.stage, text="", font=f.small, bg=lidar_plot.CANVAS, fg=MUTED)
        self.coords.place(relx=1.0, rely=1.0, x=-12, y=-8, anchor="se")
        self.empty = tk.Frame(self.stage, bg=CHALK, highlightthickness=1, highlightbackground=HAIR,
                              padx=26, pady=20)
        self.toast = tk.Label(self.stage, text="", font=f.bold, bg=INK, fg=MIST, padx=16, pady=8)
        self.empty_key = ""
        clean_box = tk.Frame(self.stage, bg=CHALK, highlightthickness=1, highlightbackground=HAIR)
        ttk.Checkbutton(clean_box, text="Ignorer les points isolés", variable=self.clean_var,
                        command=self.on_clean_toggle).pack(padx=8, pady=4)
        clean_box.place(relx=1.0, x=-14, y=12, anchor="ne")
        self._build_cmp_bar()

    def _build_cmp_bar(self):
        """Curseurs de la vue Grille 2D : séparation sans/avec filtre, et force du filtre."""
        f = self.fonts
        self.cmp_bar = tk.Frame(self.stage, bg=CHALK, highlightthickness=1, highlightbackground=HAIR)

        def scale(parent, to, length, command):
            return tk.Scale(parent, from_=0, to=to, orient="horizontal", showvalue=0, length=length, bd=0,
                            bg=CHALK, troughcolor=MIST, highlightthickness=0, sliderlength=20, width=14,
                            activebackground=BLUE, command=command)

        self.wipe_row = tk.Frame(self.cmp_bar, bg=CHALK)
        self.wipe_row.pack(fill="x", padx=12, pady=(8, 0))
        tk.Label(self.wipe_row, text="Sans filtre", font=f.small, bg=CHALK, fg=MUTED).pack(side="left")
        self.wipe = scale(self.wipe_row, 100, 280, self.on_wipe)
        self.wipe.set(50)
        self.wipe.pack(side="left", padx=8)
        tk.Label(self.wipe_row, text="Avec filtre", font=f.small, bg=CHALK, fg=MUTED).pack(side="left")
        row = self.strength_row = tk.Frame(self.cmp_bar, bg=CHALK)
        row.pack(fill="x", padx=12, pady=(2, 8))
        tk.Label(row, text="Force du filtre", font=f.small, bg=CHALK, fg=MUTED).pack(side="left")
        self.strength = scale(row, 20, 200, self.on_strength)
        self.strength.pack(side="left", padx=8)
        self.strength_lbl = tk.Label(row, text="", font=f.small, bg=CHALK, fg=INK, width=30, anchor="w")
        self.strength_lbl.pack(side="left")

    # ------------------------------------------------------------------
    # Chronologie
    # ------------------------------------------------------------------
    def step_done(self):
        n = len(self.session.entries)
        fl = self.flags
        return {
            "connect": fl["connected_once"] or n > 0,
            "capture": n > 0,
            "place": fl["map_seen"] and n > 0 and (fl["posed"] or n == 1),
            "maps": (fl["tuned"] or fl["exported"]) and n > 0,
            "grid": fl["grid_seen"] and n > 0,
            "export": fl["exported"] and n > 0,
        }

    def step_summaries(self):
        n = len(self.session.entries)
        fl = self.flags
        if self.conn_state == "live":
            conn = (f"{self.port_name} · {self.baud_var.get()} bauds" if self.got_scan
                    else "Connecté, en attente de données")
        elif self.conn_state == "sim":
            conn = "Simulation en cours"
        else:
            conn = "Données importées ou déjà reçues" if n else "Branchez l'ESP32 en USB"
        placed = fl["map_seen"] and (fl["posed"] or n == 1)
        sess = self.session
        return {
            "connect": conn,
            "capture": plural(n, "scan capturé", "scans capturés") if n else "Le robot reste immobile",
            "place": ("Poses réglées sur la carte" if placed else "Un seul scan : rien à aligner" if n == 1
                      else "Alignez les scans sur la carte") if n else "Après la première capture",
            "maps": (f"{plural(len(sess.vmap.segments), 'segment', 'segments')} · grille "
                     f"{sess.grid.nx}×{sess.grid.ny}") if sess.vmap and sess.grid else "Valeurs par défaut",
            "grid": self.grid_summary() if n else "Après la première capture",
            "export": "Export enregistré" if fl["exported"] else "Fichiers pour le TP",
        }

    def grid_summary(self):
        st = self.last_pair.stats if self.last_pair is not None and self.flags["grid_seen"] else None
        if st is None:
            return "Juger le filtre avec un curseur"
        return (f"Occupées : de {st['raw']['occ']} à {st['flt']['occ']} · isolées : de {st['raw']['iso']} "
                f"à {st['flt']['iso']}")

    def update_timeline(self):
        done, sums = self.step_done(), self.step_summaries()
        keys = [k for k, _ in STEPS]
        cur = next((k for k in keys if not done[k] and k not in OPTIONAL), None)
        for k in keys:
            step = self.steps[k]
            step.set_state("done" if done[k] else "current" if k == cur else "todo")
            step.set_summary(sums[k])
        if cur != self.current_step:
            prev, self.current_step = self.current_step, cur
            if cur:
                self.steps[cur].set_open(True)
                self.steps[cur].pulse()
                if prev is not None and cur in ("capture", "place"):
                    self.after(80, lambda: self.scroll.scroll_to(self.steps[cur]))
            if prev == "connect" and prev != cur:
                self.steps["connect"].set_open(False)   # les étapes de travail restent ouvertes

    def open_step(self, key):
        self.steps[key].set_open(True)
        self.after(50, lambda: self.scroll.scroll_to(self.steps[key]))

    # ------------------------------------------------------------------
    # Pastille, HUD, écran vide, message
    # ------------------------------------------------------------------
    def update_chrome(self):
        t = self.live_times
        hz = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0
        kind = self.conn_state
        text = {"live": (f"Lidar connecté · {hz:.1f} scans/s" if self.live else "Lidar connecté · en attente de données"),
                "sim": f"Simulation · {hz:.1f} scans/s", "error": "Connexion échouée"}.get(kind, "Non connecté")
        self.pill_text.configure(text=text)
        color = {"live": OK, "sim": BLUE, "error": BAD}.get(kind, HAIR_STRONG)
        self.led.delete("all")
        self.led.create_oval(3, 3, 13, 13, fill=color, outline=color)

        sess = self.session
        shown = self.live_used() if self.tab == "live" else None
        if shown is not None and len(self.live):
            ignored = len(self.live) - len(shown)
            parts = [f"{len(shown)} points"]
            if self.clean_var.get():
                parts.append(f"{ignored} ignorés")
            parts.append(f"{hz:.1f} scans/s")
            if len(shown):
                parts.append(f"{float(np.max(shown.ranges)):.2f} m max")
            self.hud.configure(text="     ".join(parts))
            self.hud.place(x=14, y=12)
        elif self.tab == "map" and sess.entries:
            vis = len(sess.visible())
            extra = (f"     {len(sess.vmap.segments)} segments     {sess.grid.nx}×{sess.grid.ny} cellules"
                     if sess.vmap and sess.grid else "")
            self.hud.configure(text=f"{plural(vis, 'scan', 'scans')}{extra}")
            self.hud.place(x=14, y=12)
        elif self.tab == "grid" and sess.entries:
            self.hud.configure(text=self.grid_hud_text())
            self.hud.place(x=14, y=12)
        else:
            self.hud.place_forget()
        self.update_empty()

    def update_empty(self):
        key = ""
        if self.tab == "live" and self.live is None:
            key = "wait" if self.source and self.conn_state == "live" else "none"
        elif self.tab in ("map", "grid") and not self.session.entries:
            key = "nomap"
        if key == self.empty_key:
            return
        self.empty_key = key
        for w in self.empty.winfo_children():
            w.destroy()
        if not key:
            self.empty.place_forget()
            return
        f = self.fonts
        titles = {"none": "Aucune donnée du lidar", "wait": "En attente du premier tour",
                  "nomap": "Aucun scan capturé"}
        texts = {"none": "Branchez l'ESP32 en USB puis connectez-le. Sans robot, la simulation montre le fonctionnement.",
                 "wait": "Les points apparaissent dès qu'un tour complet est reçu. Rien après quelques secondes ? "
                         "Appuyez sur le bouton EN de la carte.",
                 "nomap": "Capturez un scan depuis l'onglet Direct, ou importez des fichiers déjà enregistrés."}
        tk.Label(self.empty, text=titles[key], font=f.title, bg=CHALK, fg=INK).pack()
        tk.Label(self.empty, text=texts[key], font=f.text, bg=CHALK, fg=MUTED, wraplength=360,
                 justify="center").pack(pady=(6, 10))
        row = tk.Frame(self.empty, bg=CHALK)
        row.pack()
        buttons = {"none": [("Connecter le lidar", self.connect, "primary"), ("Lancer la simulation", self.toggle_sim, "default")],
                   "wait": [("Voir les données reçues", self.show_monitor, "default")],
                   "nomap": [("Aller à Direct", lambda: self.set_tab("live"), "primary"),
                             ("Importer des fichiers", self.import_files, "default")]}[key]
        for text, cmd, kind in buttons:
            make_button(row, text, cmd, kind, f).pack(side="left", padx=4)
        self.empty.place(relx=0.5, rely=0.5, anchor="center")
        self.empty.lift()

    def show_toast(self, text):
        self.toast.configure(text=text)
        self.toast.place(relx=0.5, rely=1.0, y=-104 if self.tab == "grid" else -34, anchor="s")
        self.toast.lift()
        if self.toast_job:
            self.after_cancel(self.toast_job)
        self.toast_job = self.after(2600, self.toast.place_forget)

    def show_monitor(self):
        self.open_step("connect")
        self.mon_open.set(True)
        self._toggle_monitor()

    # ------------------------------------------------------------------
    # Connexion
    # ------------------------------------------------------------------
    def warn_missing_serial(self):
        if not pyserial_available():
            self.set_status(self.conn_status, "pyserial n'est pas installé : pip install pyserial. "
                                              "La simulation et l'import de fichiers restent disponibles.", "warn")

    def refresh_ports(self):
        if not pyserial_available():
            self.ports = []
            self.port_combo.configure(values=[])
            return
        try:
            self.ports = list_ports()
        except Exception as exc:                            # noqa: BLE001
            self.ports = []
            self.set_status(self.conn_status, f"Liste des ports illisible : {exc}", "err")
        labels = [port_label(p) for p in self.ports]
        self.port_combo.configure(values=labels)
        if labels:
            current = self.port_var.get()
            if current not in labels:
                preferred = next((l for p, l in zip(self.ports, labels) if p["vendor"]), labels[0])
                self.port_var.set(preferred)
        else:
            self.port_var.set("")

    def chosen_device(self):
        text = self.port_var.get().strip()
        for p in self.ports:
            if text == port_label(p) or text == p["device"]:
                return p["device"]
        return text.split(" ")[0] if text else ""

    def connect(self):
        if self.source is not None:
            return
        if not pyserial_available():
            self.conn_state = "error"
            self.set_status(self.conn_status, "pyserial n'est pas installé : pip install pyserial.", "err")
            self.show_help(True)
            self.open_diagnostic()
            self.update_chrome()
            return
        self.refresh_ports()
        device = self.chosen_device()
        if not device:
            self.conn_state = "idle"
            self.set_status(self.conn_status, "Aucun port série détecté. Ouvrez le diagnostic pour voir quoi vérifier.", "warn")
            self.show_help(True)
            self.open_diagnostic()
            self.update_chrome()
            return
        self.start_source(SerialSource(device, int(self.baud_var.get()), self.new_queue(),
                                       tele_flip_y=self.tele_flip.get()), device)
        self.port_name = next((port_label(p) for p in self.ports if p["device"] == device), device)
        self.set_status(self.conn_status, f"Ouverture de {device}…")
        self.show_help(False)

    def new_queue(self):
        self.events = queue.Queue()
        return self.events

    def start_source(self, source, name):
        self.stop_source()
        self.source = source
        self.live = None
        self.live_times = []
        self.bytes_total = 0
        self.got_scan = False
        self.mon_text.delete("1.0", "end")
        source.start()
        self.btn_connect.configure(state="disabled")
        self.btn_disconnect.configure(state="normal")
        self.btn_sim.configure(state="normal")

    def stop_source(self):
        src, self.source = self.source, None
        for job in self.watchdogs:
            self.after_cancel(job)
        self.watchdogs = []
        if src is not None:
            src.stop()
            src.join(timeout=1.5)

    def disconnect(self):
        self.stop_source()
        self.finish_connection("Déconnecté.")

    def finish_connection(self, message=None, keep_error=False):
        if not keep_error:
            self.conn_state = "idle"
        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        self.btn_sim.configure(text="Simulation sans robot")
        if message:
            self.set_status(self.conn_status, message)
        self.update_chrome()
        self.update_timeline()
        self.redraw()

    def toggle_sim(self):
        if isinstance(self.source, SimulatedSource):
            self.stop_source()
            self.finish_connection("Simulation arrêtée.")
            return
        if self.source is not None:
            return
        self.start_source(SimulatedSource(lambda: self.sim_pose, self.new_queue()), "simulation")
        self.conn_state = "sim"
        self.got_scan = True
        self.flags["connected_once"] = True
        self.btn_sim.configure(text="Arrêter la simulation")
        self.btn_connect.configure(state="disabled")
        self.set_status(self.conn_status, "Simulation : le lidar virtuel se place à la position saisie à l'étape 2.", "ok")
        self.show_help(False)
        self.update_chrome()
        self.update_timeline()

    def show_help(self, on):
        if on:
            self.help_btn.pack(anchor="w", pady=(6, 0), after=self.conn_status)
        else:
            self.help_btn.pack_forget()

    def start_watchdogs(self):
        for job in self.watchdogs:
            self.after_cancel(job)

        def silent():
            if isinstance(self.source, SerialSource) and self.bytes_total == 0:
                self.set_status(self.conn_status, "Connecté, mais aucun octet reçu après 4 s. Appuyez sur le bouton EN "
                                                  "de la carte, ou vérifiez que le sketch envoie les données (printScanCSV).", "warn")
                self.show_monitor()
                self.show_help(True)

        def garbled():
            if isinstance(self.source, SerialSource) and not self.got_scan and self.bytes_total > 0:
                self.set_status(self.conn_status, "Des octets arrivent, mais aucun tour complet n'est reconnu. Vérifiez la "
                                                  "vitesse (115200 pour le sketch fourni) et le format : printScanCSV ou "
                                                  "printScanTeleplot.", "warn")
                self.show_monitor()

        self.watchdogs = [self.after(4000, silent), self.after(8000, garbled)]

    # ---- événements venant des threads ----
    def poll(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                self.handle_event(kind, payload)
        except queue.Empty:
            pass
        self.after(30, self.poll)

    def handle_event(self, kind, payload):
        if kind == "scan":
            self.on_rows(payload)
        elif kind == "bytes":
            self.bytes_total, tail = payload
            self.update_monitor(tail)
        elif kind == "opened":
            self.conn_state = "live"
            self.set_status(self.conn_status, "Connecté. Attente du premier tour complet…", "ok")
            self.show_toast("Lidar connecté")
            self.start_watchdogs()
            self.update_chrome()
            self.update_timeline()
        elif kind == "open_error":
            text, _busy = explain_serial_error(payload)
            self.source = None
            self.conn_state = "error"
            self.finish_connection(keep_error=True)
            self.set_status(self.conn_status, text, "err")
            self.show_help(True)
        elif kind == "error":
            self.conn_state = "error"
            self.set_status(self.conn_status, payload, "err")
        elif kind == "closed":
            if self.source is not None:               # la source s'est arrêtée seule (câble retiré...)
                self.source = None
                self.finish_connection(keep_error=self.conn_state == "error")

    def update_monitor(self, tail):
        lines = [l[:110] + ("…" if len(l) > 110 else "") for l in tail.splitlines() if l.strip()][-6:]
        self.mon_text.delete("1.0", "end")
        self.mon_text.insert("1.0", "\n".join(lines) if lines else "Rien reçu pour l'instant.")
        self.mon_bytes.configure(text=f"{plural(self.bytes_total, 'octet reçu', 'octets reçus')}")

    def on_rows(self, rows):
        scan = rows_to_scan(rows, "live", offset_deg=to_float(self.offset_var.get()), mirror=self.mirror.get())
        if len(scan) < 20:
            return
        now = time.monotonic()
        self.live_times.append(now)
        self.live_times = self.live_times[-10:]
        if not self.got_scan:
            self.got_scan = True
            self.flags["connected_once"] = True
            if isinstance(self.source, SerialSource):
                self.set_status(self.conn_status, "Réception en cours.", "ok")
                self.show_help(False)
            self.update_timeline()
        if self.queue_left > 0:
            self.capture_scan(scan)
            self.queue_left -= 1
        if not self.freeze.get():
            self.live = scan
            self.refresh_live_segs()
            if self.tab == "live" and now - self.last_live_draw > 0.08:
                self.last_live_draw = now
                self.redraw()
        self.update_chrome()

    # ------------------------------------------------------------------
    # Position du robot et capture
    # ------------------------------------------------------------------
    def current_capture_pose(self):
        values = tuple(to_float(v.get()) for v in self.cap_vars)
        return self.session.pose_from_input(self.pose_mode.get(), values)

    def on_mode_change(self):
        if self.pose_mode.get() == "rel":
            for v in self.cap_vars:
                v.set("0")                      # on part d'un déplacement nul
        self.update_pose_ui()

    def update_pose_ui(self):
        rel = self.pose_mode.get() == "rel"
        names = ("avant (m)", "gauche (m)", "rotation (°)") if rel else ("x (m)", "y (m)", "θ (°)")
        for var, text in zip(self.cap_labels, names):
            var.set(text)
        pose = self.current_capture_pose()
        self.sim_pose = pose                    # lu par le thread de simulation
        if not rel:
            self.pose_hint.configure(text="")
        elif not self.session.entries:
            self.pose_hint.configure(text="Premier scan : il définit l'origine (0, 0, 0 conseillé).")
        else:
            self.pose_hint.configure(text=f"Position résultante dans le repère de l'origine : "
                                          f"x = {pose[0]:.2f} m, y = {pose[1]:.2f} m, θ = {pose[2]:.1f}°")

    def capture_now(self):
        if self.live is None:
            self.set_status(self.cap_status, "Aucun scan à capturer : le lidar n'envoie pas encore de données.", "err")
            return
        n = max(1, int(to_float(self.ncap_var.get(), 1)))
        self.batch_pose = self.current_capture_pose()      # même position pour tous les tours de ce lot
        self.capture_scan(self.live)
        self.queue_left = n - 1
        if self.pose_mode.get() == "rel":
            for v in self.cap_vars:
                v.set("0")

    def capture_scan(self, scan):
        pose = self.batch_pose or self.current_capture_pose()
        entry = self.session.capture(scan, self.prefix_var.get(), pose)
        self.selected = len(self.session.entries) - 1
        self.flags["exported"] = False
        self.render_scan_list()
        self.schedule_rebuild()
        self.set_status(self.cap_status, f"Scan « {entry.name} » capturé ({len(entry.scan)} points).", "ok")
        self.show_toast(f"Scan {entry.name} capturé")
        self.update_pose_ui()
        self.update_timeline()

    # ------------------------------------------------------------------
    # Liste des scans (étape 3)
    # ------------------------------------------------------------------
    def render_scan_list(self):
        f = self.fonts
        for w in self.scan_box.winfo_children():
            w.destroy()
        self.pose_vars = []
        if not self.session.entries:
            tk.Label(self.scan_box, text="Aucun scan pour l'instant.", bg=CHALK, fg=MUTED,
                     font=f.small).pack(anchor="w")
            self.update_pose_ui()
            return
        for i, e in enumerate(self.session.entries):
            sel = i == self.selected
            row = tk.Frame(self.scan_box, bg=BLUE_SOFT if sel else CHALK, highlightthickness=1,
                           highlightbackground=BLUE if sel else HAIR)
            row.pack(fill="x", pady=3)
            tk.Frame(row, width=5, bg=e.color).pack(side="left", fill="y")
            inner = tk.Frame(row, bg=row["bg"])
            inner.pack(side="left", fill="x", expand=True, padx=8, pady=6)
            top = tk.Frame(inner, bg=row["bg"])
            top.pack(fill="x")
            vis = tk.BooleanVar(value=e.visible)
            cb = tk.Checkbutton(top, variable=vis, bg=row["bg"], activebackground=row["bg"], bd=0,
                                highlightthickness=0, command=lambda e=e, v=vis: self.set_visible(e, v.get()))
            cb.pack(side="left")
            name = tk.Label(top, text=e.name, font=f.bold, bg=row["bg"], fg=INK, anchor="w")
            name.pack(side="left")
            tk.Label(top, text=f"{len(e.scan)} pts", font=f.small, bg=row["bg"], fg=MUTED).pack(side="left", padx=8)
            x = tk.Label(top, text="✕", font=f.text, bg=row["bg"], fg=MUTED, cursor="hand2")
            x.pack(side="right")
            x.bind("<Button-1>", lambda ev, i=i: self.remove_scan(i))
            poses = tk.Frame(inner, bg=row["bg"])
            poses.pack(fill="x", pady=(4, 0))
            vars_ = []
            for k, unit in enumerate(("x", "y", "θ")):
                v = tk.StringVar(value=f"{e.pose[k]:.2f}" if k < 2 else f"{e.pose[k]:.1f}")
                ttk.Entry(poses, textvariable=v, width=7).grid(row=0, column=k, padx=(0, 4), sticky="ew")
                poses.columnconfigure(k, weight=1)
                v.trace_add("write", lambda *a, e=e, k=k, v=v: self.on_pose_edit(e, k, v))
                vars_.append(v)
            self.pose_vars.append(vars_)
            for w in (row, inner, top, name):
                w.bind("<Button-1>", lambda ev, i=i: self.select_scan(i))
        self.update_pose_ui()

    def select_scan(self, i):
        self.selected = i
        self.render_scan_list()
        self.redraw()

    def set_visible(self, entry, value):
        entry.visible = value
        self.schedule_rebuild()
        self.redraw()
        self.update_chrome()

    def remove_scan(self, i):
        self.session.remove(i)
        self.selected = min(self.selected, len(self.session.entries) - 1)
        self.render_scan_list()
        self.schedule_rebuild()
        self.update_chrome()
        self.update_timeline()
        self.redraw()

    def on_pose_edit(self, entry, k, var):
        if self._updating_vars:
            return
        text = var.get().strip()
        if text in ("", "-", ".", "-."):
            return
        try:
            entry.pose[k] = float(text.replace(",", "."))
        except ValueError:
            return
        self.touch_pose()
        self.schedule_rebuild()
        self.redraw()
        self.update_pose_ui()

    def refresh_pose_entries(self):
        self._updating_vars = True
        try:
            for e, vars_ in zip(self.session.entries, self.pose_vars):
                for k, v in enumerate(vars_):
                    v.set(f"{e.pose[k]:.2f}" if k < 2 else f"{e.pose[k]:.1f}")
        finally:
            self._updating_vars = False

    def touch_pose(self):
        if not self.flags["posed"]:
            self.flags["posed"] = True
            self.update_timeline()

    # ------------------------------------------------------------------
    # Cartes
    # ------------------------------------------------------------------
    def params(self):
        return MapParams(resolution=clamp_cell_mm(self.res_var.get()) / 1000.0,
                         forget=min(1.0, max(0.5, to_float(self.forget_var.get(), 1.0))),
                         ang_tol_deg=to_float(self.ang_var.get(), 10.0),
                         dist_tol=to_float(self.dist_var.get(), 0.06),
                         gap_tol=to_float(self.gap_var.get(), 0.15),
                         min_weight=to_float(self.minw_var.get(), 1.0),
                         clean=self.clean_var.get(),
                         min_points=max(1, int(to_float(self.minpts_var.get(), 8))),
                         min_range=max(0.0, to_float(self.minrange_var.get(), 0.20)),
                         break_min=max(0.02, to_float(self.brk_var.get(), 0.10)),
                         confirm=self.confirm_var.get(),
                         confirm_tol=max(0.02, to_float(self.conf_tol_var.get(), 0.10)))

    def on_cell_change(self):
        """Taille des carreaux saisie (mm, entier) : on la ramène à l'intervalle permis puis on recalcule."""
        mm = clamp_cell_mm(self.res_var.get())
        if self.res_var.get() != str(mm):
            self.res_var.set(str(mm))
        self.on_param_change()

    def on_param_change(self):
        self.flags["tuned"] = True
        self.on_clean_toggle()

    def on_clean_toggle(self):
        """Filtre activé, désactivé ou réglé : on recalcule les cartes et on redessine aussi la vue directe."""
        self.refresh_live_segs()
        self.rebuild()
        self.redraw()

    def live_used(self):
        """Le scan de la vue directe, filtré si la case est cochée (mis en cache tant que rien ne change)."""
        if self.live is None or not self.clean_var.get():
            return self.live
        fp = self.params().filter()
        cached = self._live_used
        if cached is None or cached[0] is not self.live or cached[1] != fp:
            cached = (self.live, fp, filter_isolated(self.live, **fp.kwargs()))
            self._live_used = cached
        return cached[2]

    def refresh_live_segs(self):
        shown = self.live_used()
        self.live_segs = extract_segments(shown.points_local()) if shown is not None and self.opt_live_seg.get() else []

    def schedule_rebuild(self):
        if not self.auto_build.get():
            return
        if self.rebuild_job:
            self.after_cancel(self.rebuild_job)
        self.rebuild_job = self.after(150, self.rebuild)

    def rebuild(self):
        self.rebuild_job = None
        try:
            self.session.rebuild(self.params())
        except Exception as exc:                            # noqa: BLE001
            self.set_status(self.map_status, f"Calcul impossible : {exc}", "err")
            return
        self.set_status(self.map_status, self.session.map_info, "ok" if self.session.map_info else "")
        self.update_chrome()
        self.update_timeline()
        if self.tab in ("map", "grid"):
            self.redraw()

    # ------------------------------------------------------------------
    # Dessin et interactions
    # ------------------------------------------------------------------
    def set_tab(self, tab):
        self.tab = tab
        for key, b in self.tab_btns.items():
            on = key == tab
            b.configure(bg=BLUE if on else MIST, fg="#ffffff" if on else MUTED,
                        activebackground=BLUE if on else BLUE_SOFT, activeforeground="#ffffff" if on else BLUE)
        for frame in (self.live_opts, self.map_opts, self.grid_opts):
            frame.pack_forget()
        self.cmp_bar.place_forget()
        if tab == "live":
            self.live_opts.pack(side="left")
        elif tab == "grid":
            self.grid_opts.pack(side="left")
            if self.session.entries:
                self.flags["grid_seen"] = True
            self.grid_view = None
            self.sync_grid_controls()
            self.cmp_bar.place(x=14, rely=1.0, y=-12, anchor="sw")
        else:
            self.map_opts.pack(side="left")
            if self.session.entries:
                self.flags["map_seen"] = True
            if self.auto_build.get() or self.session.grid is None:
                try:
                    self.session.rebuild(self.params())
                except Exception as exc:                    # noqa: BLE001
                    self.set_status(self.map_status, f"Calcul impossible : {exc}", "err")
            self.map_view = None
        self.redraw()
        self.update_chrome()
        self.update_timeline()

    def remember_view(self):
        if self.tab in ("map", "grid"):
            self.ax.apply_aspect()
            view = (*self.ax.get_xlim(), *self.ax.get_ylim())
            if self.tab == "map":
                self.map_view = view
            else:
                self.grid_view = view

    def redraw(self):
        if not hasattr(self, "ax"):
            return
        if self.tab == "live":
            lidar_plot.draw_live(self.ax, self.live_used(), self.live_segs if self.opt_live_seg.get() else None,
                                 self.opt_intensity.get())
        elif self.tab == "grid":
            pair = self.current_pair()
            self.last_pair = pair
            self.grid_art = lidar_plot.draw_grid(self.ax, self.session, pair, self.grid_mode.get(),
                                                 self.wipe.get() / 100.0, self.grid_view, self.selected)
            self.remember_view()
            self.sync_grid_controls()
            self.render_grid_stats()
            self.update_timeline()
        else:
            self.artists = lidar_plot.draw_map(self.ax, self.session, self.selected, self.lay_pts.get(),
                                               self.lay_grid.get(), self.lay_seg.get(), self.map_view)
            self.remember_view()
        self.canvas.draw_idle()

    def fit_view(self):
        self.map_view = None
        self.grid_view = None
        self.redraw()

    # ---- grille 2D ----
    def current_pair(self):
        """Les deux grilles (sans / avec filtre) pour les réglages actuels ; None s'il n'y a aucun scan."""
        if not self.session.entries:
            return None
        params = self.params()
        self.session.filter = params.filter()
        return self.session.grid_pair(clamp_cell_mm(self.res_var.get()), params.forget)

    def grid_hud_text(self):
        pair = self.last_pair or self.current_pair()
        if pair is None:
            return ""
        st = pair.stats
        pct = round(100 * st["ignored"] / st["total"]) if st["total"] else 0
        size = f"Carreaux de {pair.cell_mm} mm · grille {pair.raw.nx}×{pair.raw.ny}"
        if pair.capped:
            size += " (zone très étendue : carreaux agrandis)"
        filt = (f"{st['ignored']} points ignorés ({pct} %)" if self.clean_var.get() else "filtre désactivé")
        return (f"{size}\nSans filtre : {st['raw']['occ']} occupées, dont {st['raw']['iso']} isolées\n"
                f"Avec filtre : {st['flt']['occ']} occupées, dont {st['flt']['iso']} isolées · {filt}")

    def render_grid_stats(self):
        pair = self.last_pair
        self.grid_stats_lbl.configure(text=stats_table(pair.stats) if pair is not None else "")
        self.hud.configure(text=self.grid_hud_text())
        if pair is not None:
            self.hud.place(x=14, y=12)

    def sync_grid_controls(self):
        """Cale le curseur « force du filtre » sur la case et le champ « Points min. » (sans déclencher de calcul)."""
        n = int(to_float(self.minpts_var.get(), 8)) if self.clean_var.get() else 0
        self._grid_sync = True
        try:
            self.strength.set(max(0, min(20, n)))
        finally:
            self._grid_sync = False
        self.strength_lbl.configure(text=f"groupes de {n} points min." if n else "désactivé (aucun point ignoré)")
        if self.grid_mode.get() == "cmp":
            self.wipe_row.pack(fill="x", padx=12, pady=(8, 0), before=self.strength_row)
        else:
            self.wipe_row.pack_forget()

    def on_grid_mode(self):
        self.sync_grid_controls()
        self.redraw()

    def apply_wipe(self):
        lidar_plot.move_wipe(self.grid_art, self.wipe.get() / 100.0)

    def on_wipe(self, _value):
        if self.tab == "grid" and self.grid_art:
            self.apply_wipe()
            self.canvas.draw_idle()

    def on_strength(self, value):
        """Curseur « force du filtre » : 0 = désactivé, sinon c'est le nombre de points minimum d'un groupe."""
        if self._grid_sync:
            return
        n = int(float(value))
        self.strength_lbl.configure(text=f"groupes de {n} points min." if n else "désactivé (aucun point ignoré)")
        self.clean_var.set(n > 0)
        if n > 0:
            self.minpts_var.set(str(n))
        self.flags["tuned"] = True
        if self.strength_job:
            self.after_cancel(self.strength_job)
        self.strength_job = self.after(120, self.apply_strength)       # on attend que la main se stabilise

    def apply_strength(self):
        self.strength_job = None
        self.refresh_live_segs()
        self.rebuild()
        self.redraw()

    def on_live_option(self):
        self.refresh_live_segs()
        self.redraw()

    def on_press(self, ev):
        if self.tab not in ("map", "grid") or ev.inaxes is not self.ax or ev.button != 1:
            return
        sel = self.selected_entry()
        if self.tab == "map" and self.mode_var.get() == "scan" and sel is not None:
            self.drag = dict(kind="scan", pose0=list(sel.pose), x0=ev.xdata, y0=ev.ydata)
        else:
            self.drag = dict(kind="pan", inv=self.ax.transData.inverted(), px=(ev.x, ev.y),
                             xlim=self.ax.get_xlim(), ylim=self.ax.get_ylim())

    def on_motion(self, ev):
        if ev.inaxes is self.ax and ev.xdata is not None:
            self.coords.configure(text=f"x = {ev.xdata:.2f} m · y = {ev.ydata:.2f} m")
        d = self.drag
        if not d or ev.x is None:
            return
        if d["kind"] == "pan":
            p0 = d["inv"].transform(d["px"])
            p1 = d["inv"].transform((ev.x, ev.y))
            dx, dy = p0[0] - p1[0], p0[1] - p1[1]
            self.ax.set_xlim(d["xlim"][0] + dx, d["xlim"][1] + dx)
            self.ax.set_ylim(d["ylim"][0] + dy, d["ylim"][1] + dy)
            if self.tab == "grid":
                self.apply_wipe()                           # la séparation garde sa place à l'écran
            self.canvas.draw_idle()
            return
        sel = self.selected_entry()
        if sel is None or ev.xdata is None:
            return
        import math
        x0, y0, th0 = d["pose0"]
        if ev.key == "shift":                               # Maj : on tourne autour du robot
            a0 = math.atan2(d["y0"] - y0, d["x0"] - x0)
            a1 = math.atan2(ev.ydata - y0, ev.xdata - x0)
            th = th0 + math.degrees(a1 - a0)
            sel.pose[2] = (th + 180.0) % 360.0 - 180.0
        else:
            sel.pose[0] = x0 + (ev.xdata - d["x0"])
            sel.pose[1] = y0 + (ev.ydata - d["y0"])
        lidar_plot.move_scan_artists(self.artists, sel, self.session.filter)
        self.canvas.draw_idle()
        self.refresh_pose_entries()
        self.touch_pose()
        self.schedule_rebuild()

    def on_release(self, ev):
        if self.drag and self.drag["kind"] == "pan":
            self.remember_view()
        self.drag = None

    def on_scroll(self, ev):
        if self.tab not in ("map", "grid") or ev.inaxes is not self.ax:
            return
        factor = 0.85 if ev.button == "up" else 1 / 0.85
        xl, yl = self.ax.get_xlim(), self.ax.get_ylim()
        self.ax.set_xlim(ev.xdata + (xl[0] - ev.xdata) * factor, ev.xdata + (xl[1] - ev.xdata) * factor)
        self.ax.set_ylim(ev.ydata + (yl[0] - ev.ydata) * factor, ev.ydata + (yl[1] - ev.ydata) * factor)
        self.remember_view()
        if self.tab == "grid":
            self.apply_wipe()
        self.canvas.draw_idle()

    def selected_entry(self):
        return self.session.entries[self.selected] if 0 <= self.selected < len(self.session.entries) else None

    def on_key(self, ev):
        focus = self.focus_get()
        if isinstance(focus, (tk.Entry, ttk.Entry, ttk.Combobox, ttk.Spinbox, tk.Text)):
            return
        if ev.keysym == "space":
            self.capture_now()
            return
        sel = self.selected_entry()
        if self.tab != "map" or sel is None:
            return
        shift = bool(ev.state & 0x1)
        step, rot = (0.01, 0.2) if shift else (0.05, 1.0)
        moves = {"Left": (-step, 0, 0), "Right": (step, 0, 0), "Up": (0, step, 0), "Down": (0, -step, 0),
                 "comma": (0, 0, rot), "less": (0, 0, rot), "period": (0, 0, -rot), "greater": (0, 0, -rot)}
        if ev.keysym not in moves:
            return
        dx, dy, dth = moves[ev.keysym]
        sel.pose[0] += dx
        sel.pose[1] += dy
        sel.pose[2] = (sel.pose[2] + dth + 180.0) % 360.0 - 180.0
        self.refresh_pose_entries()
        self.touch_pose()
        self.schedule_rebuild()
        self.redraw()

    # ------------------------------------------------------------------
    # Import / export
    # ------------------------------------------------------------------
    def import_files(self):
        paths = filedialog.askopenfilenames(title="Importer des scans, poses.json ou projet.json",
                                            filetypes=[("Scans et projets", "*.csv *.json *.txt"), ("Tous", "*.*")])
        if paths:
            self.import_paths(paths)

    def import_paths(self, paths):
        try:
            added = self.session.import_files(paths)
        except Exception as exc:                            # noqa: BLE001
            messagebox.showerror("Import impossible", str(exc))
            return
        if added:
            self.flags["exported"] = False
            self.selected = len(self.session.entries) - 1
        self.render_scan_list()
        self.schedule_rebuild()
        self.update_chrome()
        self.update_timeline()
        self.redraw()
        self.show_toast(plural(added, "scan importé", "scans importés") if added else "Aucun scan reconnu dans ces fichiers")

    def clear_all(self):
        if self.session.entries and not messagebox.askyesno("Tout effacer", "Effacer tous les scans capturés ?"):
            return
        self.session.clear()
        self.selected = -1
        self.flags.update(map_seen=False, posed=False, tuned=False, exported=False, grid_seen=False)
        self.last_pair = None
        self.set_status(self.cap_status, "")
        self.set_status(self.map_status, "")
        self.render_scan_list()
        self.update_chrome()
        self.update_timeline()
        self.redraw()

    def need_scans(self):
        if not self.session.entries:
            self.show_toast("Rien à exporter : capturez d'abord un scan")
            return False
        self.session.rebuild(self.params())
        return True

    def mark_exported(self, text):
        self.flags["exported"] = True
        self.show_toast(text)
        self.update_timeline()

    def grid_extra(self):
        """Fichiers ajoutés à l'export quand la grille 2D a été consultée : image de comparaison et chiffres."""
        pair = self.current_pair() if self.flags["grid_seen"] else None
        if pair is None:
            return {}
        text = (f"Carreaux : {pair.cell_mm} mm, {len(self.session.visible())} scan(s)\n"
                f"Filtre : {self.session.filter}\n\n{stats_table(pair.stats)}\n")
        return {"grille2d_comparaison.png": lidar_plot.render_comparison_png(self.session, pair),
                "grille2d_stats.txt": text.encode("utf-8")}

    def export_grid_png(self):
        if not self.need_scans():
            return
        pair = self.current_pair()
        path = filedialog.asksaveasfilename(title="Comparaison de la grille 2D", defaultextension=".png",
                                            initialfile="grille2d_comparaison.png", filetypes=[("Image PNG", "*.png")])
        if path and pair is not None:
            Path(path).write_bytes(lidar_plot.render_comparison_png(self.session, pair))
            self.show_toast(f"Comparaison enregistrée : {Path(path).name}")

    def export_zip(self):
        if not self.need_scans():
            return
        path = filedialog.asksaveasfilename(title="Exporter le relevé", defaultextension=".zip",
                                            initialfile="lidar_export.zip", filetypes=[("Archive zip", "*.zip")])
        if path:
            self.session.export_zip(path, lidar_plot.render_map_png(self.session, self.selected), self.grid_extra())
            self.mark_exported(f"Export : {plural(len(self.session.entries), 'scan', 'scans')} dans {Path(path).name}")

    def export_folder(self):
        if not self.need_scans():
            return
        folder = filedialog.askdirectory(title="Dossier de destination", mustexist=False)
        if folder:
            self.session.export_folder(folder, lidar_plot.render_map_png(self.session, self.selected),
                                       self.grid_extra())
            self.mark_exported(f"Export écrit dans {Path(folder).name}")

    def export_vec(self):
        if self.need_scans() and self.session.vmap is not None:
            path = filedialog.asksaveasfilename(defaultextension=".txt", initialfile="carte_vectorielle.txt")
            if path:
                self.session.vmap.save(path)
                self.mark_exported("Carte vectorielle exportée")

    def export_grid(self):
        if self.need_scans() and self.session.grid is not None:
            path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="grille.csv")
            if path:
                self.session.grid.save_csv(path)
                self.mark_exported("Grille exportée")

    def export_png(self):
        if self.need_scans():
            path = filedialog.asksaveasfilename(defaultextension=".png", initialfile="carte.png")
            if path:
                Path(path).write_bytes(lidar_plot.render_map_png(self.session, self.selected))
                self.mark_exported("Image exportée")

    # ------------------------------------------------------------------
    # Diagnostic
    # ------------------------------------------------------------------
    def open_diagnostic(self):
        win = tk.Toplevel(self)
        win.title("Diagnostic de connexion")
        win.configure(bg=CHALK)
        win.geometry("600x640")
        win.transient(self)
        f = self.fonts
        tk.Label(win, text="Diagnostic de connexion", font=f.title, bg=CHALK, fg=INK).pack(anchor="w", padx=20, pady=(16, 8))
        box = tk.Frame(win, bg=CHALK)
        box.pack(fill="x", padx=20)
        foot = tk.Frame(win, bg=CHALK)
        foot.pack(side="bottom", fill="x", padx=20, pady=14)

        def run():
            for w in box.winfo_children():
                w.destroy()
            rows = []
            if pyserial_available():
                import serial
                rows.append(("ok", "pyserial installé", f"Version {serial.VERSION}."))
                try:
                    ports = list_ports()
                except Exception as exc:                    # noqa: BLE001
                    ports = []
                    rows.append(("bad", "Liste des ports illisible", str(exc)))
                else:
                    if ports:
                        rows.append(("ok", plural(len(ports), "port détecté", "ports détectés"),
                                     "\n".join(port_label(p) for p in ports)))
                    else:
                        rows.append(("warn", "Aucun port série détecté",
                                     "Windows ne voit aucune carte. Suivez les pistes ci-dessous."))
            else:
                rows.append(("bad", "pyserial manquant", "Dans le terminal de VS Code : pip install pyserial"))
            if isinstance(self.source, SerialSource):
                rows.append(("ok", "Port ouvert", f"{self.port_name} · {plural(self.bytes_total, 'octet reçu', 'octets reçus')}."))
            rows.append(("ok", "Système", f"{platform.system()} {platform.release()} · Python {platform.python_version()}"))
            colors = {"ok": OK, "bad": BAD, "warn": WARN}
            icons = {"ok": "✓", "bad": "✕", "warn": "!"}
            for kind, title, detail in rows:
                r = tk.Frame(box, bg=MIST)
                r.pack(fill="x", pady=3)
                tk.Label(r, text=icons[kind], font=f.bold, bg=colors[kind], fg="#ffffff", width=2).pack(side="left", padx=8, pady=8, anchor="n")
                col = tk.Frame(r, bg=MIST)
                col.pack(side="left", fill="x", expand=True, pady=6)
                tk.Label(col, text=title, font=f.bold, bg=MIST, fg=INK, anchor="w").pack(anchor="w")
                tk.Label(col, text=detail, font=f.small, bg=MIST, fg=MUTED, anchor="w", justify="left", wraplength=480).pack(anchor="w")
            self.refresh_ports()

        run()
        tk.Label(win, text="Si la liste des ports est vide", font=f.title, bg=CHALK, fg=INK).pack(anchor="w", padx=20, pady=(12, 4))
        tk.Label(win, text=EMPTY_PORT_HELP, font=f.text, bg=CHALK, fg=INK, anchor="w", justify="left",
                 wraplength=550).pack(anchor="w", padx=20)
        make_button(foot, "Fermer", win.destroy, "default", f).pack(side="right")
        make_button(foot, "Actualiser", run, "primary", f).pack(side="right", padx=8)

    # ------------------------------------------------------------------
    def on_close(self):
        self.stop_source()
        self.destroy()


def main():
    if sys.platform == "win32":                             # texte net sur les écrans à haute densité
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:                                   # noqa: BLE001
            pass
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
