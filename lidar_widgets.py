"""Éléments d'interface Tkinter : thème, boutons, zone défilante et étape de la chronologie.

Palette : brume (fond), craie (cartes), encre, bleu géomètre (actions, étapes faites),
jaune balise (réservé à l'étape en cours).
"""

import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

MIST, CHALK, INK, MUTED = "#edf0f3", "#ffffff", "#14202b", "#5b6875"
HAIR, HAIR_STRONG = "#d9dfe5", "#bcc5ce"
BLUE, BLUE_SOFT, BEACON, BEACON_INK = "#1f3fa6", "#e4e9f8", "#ffc72c", "#2a2000"
OK, WARN, BAD = "#1f7a4a", "#9a5b00", "#b3261e"


class Fonts:
    """Polices disponibles sur la machine (Segoe UI sous Windows, sinon une police sans empattement)."""

    def __init__(self, root):
        families = set(tkfont.families(root))
        text = next((f for f in ("Segoe UI Variable Text", "Segoe UI", "Helvetica Neue", "Noto Sans",
                                 "DejaVu Sans", "Arial") if f in families), "TkDefaultFont")
        display = next((f for f in ("Bahnschrift", "Segoe UI Semibold", "Segoe UI", text)
                        if f in families), text)
        self.text = (text, 10)
        self.small = (text, 9)
        self.bold = (text, 10, "bold")
        self.title = (display, 12, "bold")
        self.brand = (display, 17, "bold")
        self.hud = (display, 11, "bold")
        self.mono = ("Consolas" if "Consolas" in families else "DejaVu Sans Mono", 9)


def setup_ttk(root, fonts):
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", font=fonts.text, background=CHALK, foreground=INK)
    style.configure("TFrame", background=CHALK)
    style.configure("TLabel", background=CHALK, foreground=INK)
    style.configure("TCheckbutton", background=CHALK, foreground=INK, focuscolor=CHALK)
    style.configure("TRadiobutton", background=CHALK, foreground=INK, focuscolor=CHALK)
    style.map("TCheckbutton", background=[("active", CHALK)])
    style.map("TRadiobutton", background=[("active", CHALK)])
    for name in ("TEntry", "TCombobox", "TSpinbox"):
        style.configure(name, fieldbackground=MIST, background=MIST, foreground=INK,
                        bordercolor=HAIR_STRONG, lightcolor=MIST, darkcolor=MIST, padding=4)
        style.map(name, bordercolor=[("focus", BLUE)], fieldbackground=[("readonly", MIST)])
    style.configure("Toolbar.TCheckbutton", background=CHALK)
    style.configure("Toolbar.TRadiobutton", background=CHALK)


def make_button(parent, text, command, kind="default", fonts=None, width=None):
    """Bouton plat : 'primary' (bleu), 'default' (contour), 'quiet' (sans contour), 'danger'."""
    palette = {
        "primary": dict(bg=BLUE, fg="#ffffff", abg="#2a4fc4", afg="#ffffff", border=BLUE),
        "default": dict(bg=CHALK, fg=INK, abg=BLUE_SOFT, afg=BLUE, border=HAIR_STRONG),
        "quiet": dict(bg=CHALK, fg=MUTED, abg=BLUE_SOFT, afg=BLUE, border=CHALK),
        "danger": dict(bg=CHALK, fg=BAD, abg="#fbe9e7", afg=BAD, border=HAIR_STRONG),
    }[kind]
    btn = tk.Button(parent, text=text, command=command, relief="flat", bd=0, cursor="hand2",
                    bg=palette["bg"], fg=palette["fg"], activebackground=palette["abg"],
                    activeforeground=palette["afg"], disabledforeground="#9aa5b0",
                    highlightthickness=1, highlightbackground=palette["border"],
                    highlightcolor=BLUE, padx=12, pady=5, takefocus=0, width=width or 0)
    if fonts:
        btn.configure(font=fonts.bold)
    return btn


class ScrollFrame(tk.Frame):
    """Colonne défilante à la molette."""

    def __init__(self, master, bg=MIST):
        super().__init__(master, bg=bg)
        self.canvas = tk.Canvas(self, bg=bg, highlightthickness=0, bd=0)
        self.bar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = tk.Frame(self.canvas, bg=bg)
        self.window = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.configure(yscrollcommand=self.bar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.bar.pack(side="right", fill="y")
        self.inner.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self.window, width=e.width))
        self.canvas.bind("<Enter>", lambda e: self._bind_wheel(True))
        self.canvas.bind("<Leave>", lambda e: self._bind_wheel(False))
        self.inner.bind("<Enter>", lambda e: self._bind_wheel(True))

    def _bind_wheel(self, on):
        if on:
            self.bind_all("<MouseWheel>", self._wheel)
            self.bind_all("<Button-4>", self._wheel)
            self.bind_all("<Button-5>", self._wheel)
        else:
            for ev in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                self.unbind_all(ev)

    def _wheel(self, event):
        if self.inner.winfo_height() <= self.canvas.winfo_height():
            return
        delta = -1 if (event.num == 4 or getattr(event, "delta", 0) > 0) else 1
        self.canvas.yview_scroll(delta * 2, "units")

    def scroll_to(self, widget):
        """Fait apparaître `widget` (qui doit être dans la colonne) en bas de la zone visible."""
        self.update_idletasks()
        top = widget.winfo_rooty() - self.inner.winfo_rooty()
        bottom = top + widget.winfo_height()
        view_h = self.canvas.winfo_height()
        total = self.inner.winfo_height()
        first = self.canvas.canvasy(0)
        if top < first:
            self.canvas.yview_moveto(top / total)
        elif bottom > first + view_h:
            self.canvas.yview_moveto(max(0, (bottom - view_h)) / total)


class Step(tk.Frame):
    """Une étape de la chronologie : pastille numérotée + ligne verticale + carte repliable.

    États : 'todo' (grisée), 'current' (jaune, ouverte), 'done' (bleu, coche).
    """

    NODE_R, NODE_Y, RAIL_W = 15, 26, 40

    def __init__(self, master, key, number, title, fonts, optional=False, is_last=False):
        super().__init__(master, bg=MIST)
        self.key, self.number, self.fonts = key, number, fonts
        self.optional, self.is_last = optional, is_last
        self.state, self.is_open = "todo", False
        self.on_toggle = None
        self._pulse_job = None

        self.rail = tk.Canvas(self, width=self.RAIL_W, height=1, bg=MIST, highlightthickness=0, bd=0)
        self.rail.grid(row=0, column=0, sticky="ns")
        self.rail.bind("<Configure>", lambda e: self._draw_rail())

        self.card = tk.Frame(self, bg=CHALK, highlightthickness=1, highlightbackground=HAIR)
        self.card.grid(row=0, column=1, sticky="ew", padx=(4, 8), pady=(0, 10))
        self.columnconfigure(1, weight=1)

        self.accent = tk.Frame(self.card, width=4, bg=CHALK)
        self.accent.pack(side="left", fill="y")
        content = tk.Frame(self.card, bg=CHALK)
        content.pack(side="left", fill="both", expand=True)

        self.head = tk.Frame(content, bg=CHALK, cursor="hand2")
        self.head.pack(fill="x", padx=12, pady=(9, 9))
        self.head.columnconfigure(0, weight=1)
        self.title = tk.Label(self.head, text=title, font=fonts.title, bg=CHALK, fg=INK, anchor="w")
        self.title.grid(row=0, column=0, sticky="w")
        self.summary = tk.Label(self.head, text="", font=fonts.small, bg=CHALK, fg=MUTED, anchor="w",
                                justify="left", wraplength=210)
        self.summary.grid(row=1, column=0, sticky="w", pady=(2, 0))
        self.chip = tk.Label(self.head, text="À faire", font=fonts.small, bg=CHALK, fg=MUTED,
                             padx=9, pady=1, highlightthickness=1, highlightbackground=HAIR_STRONG)
        self.chip.grid(row=0, column=1, rowspan=2, padx=(8, 6))
        self.chev = tk.Label(self.head, text="▾", font=fonts.text, bg=CHALK, fg=MUTED)
        self.chev.grid(row=0, column=2, rowspan=2)
        for w in (self.head, self.title, self.summary, self.chip, self.chev):
            w.bind("<Button-1>", lambda e: self.toggle())

        self.body = tk.Frame(content, bg=CHALK)
        self.set_state("todo")

    # ---- ouverture ------------------------------------------------------
    def toggle(self):
        self.set_open(not self.is_open)
        if self.on_toggle:
            self.on_toggle(self)

    def set_open(self, value):
        self.is_open = value
        if value:
            self.body.pack(fill="x", padx=14, pady=(0, 14))
        else:
            self.body.pack_forget()
        self.chev.configure(text="▴" if value else "▾")

    # ---- état -----------------------------------------------------------
    def set_state(self, state):
        self.state = state
        if state == "current":
            self.accent.configure(bg=BEACON)
            self.card.configure(highlightbackground=HAIR_STRONG)
            self.title.configure(fg=INK)
            self.chip.configure(text="En cours", bg=BEACON, fg=BEACON_INK, highlightbackground=BEACON)
        elif state == "done":
            self.accent.configure(bg=CHALK)
            self.card.configure(highlightbackground=HAIR)
            self.title.configure(fg=INK)
            self.chip.configure(text="Fait", bg=CHALK, fg=OK, highlightbackground="#a9d2bb")
        else:
            self.accent.configure(bg=CHALK)
            self.card.configure(highlightbackground=HAIR)
            self.title.configure(fg=MUTED)
            self.chip.configure(text="Facultatif" if self.optional else "À faire", bg=CHALK,
                                fg=MUTED, highlightbackground=HAIR_STRONG)
        self._draw_rail()

    def set_summary(self, text):
        self.summary.configure(text=text)

    # ---- pastille et ligne ---------------------------------------------
    def _draw_rail(self, halo=None):
        c = self.rail
        c.delete("all")
        cx, cy, r = self.RAIL_W // 2, self.NODE_Y, self.NODE_R
        height = max(c.winfo_height(), cy + r + 4)
        if not self.is_last:
            if self.state == "done":
                c.create_line(cx, cy + r, cx, height + 20, fill=BLUE, width=2)
            else:
                c.create_line(cx, cy + r, cx, height + 20, fill=HAIR_STRONG, width=2, dash=(4, 5))
        if halo:
            hr, color = halo
            c.create_oval(cx - hr, cy - hr, cx + hr, cy + hr, outline=color, width=3)
        if self.state == "done":
            c.create_oval(cx - r, cy - r, cx + r, cy + r, fill=BLUE, outline=BLUE)
            c.create_line(cx - 6, cy + 0, cx - 2, cy + 5, cx + 7, cy - 5, fill="#ffffff", width=3,
                          capstyle="round", joinstyle="round")
        elif self.state == "current":
            c.create_oval(cx - r, cy - r, cx + r, cy + r, fill=BEACON, outline=BEACON)
            c.create_text(cx, cy, text=str(self.number), font=self.fonts.title, fill=BEACON_INK)
        else:
            c.create_oval(cx - r, cy - r, cx + r, cy + r, fill=MIST, outline=HAIR_STRONG, width=2)
            c.create_text(cx, cy, text=str(self.number), font=self.fonts.title, fill=MUTED)

    def pulse(self, cycles=3):
        """Un halo jaune qui s'élargit trois fois : attire l'œil sur la nouvelle étape en cours."""
        if self._pulse_job:
            self.after_cancel(self._pulse_job)
        frames = [(self.NODE_R + 1 + i, _blend(BEACON, MIST, i / 12)) for i in range(12)] * cycles

        def step(k=0):
            if k >= len(frames) or self.state != "current":
                self._pulse_job = None
                self._draw_rail()
                return
            self._draw_rail(halo=frames[k])
            self._pulse_job = self.after(45, step, k + 1)

        step()


def _blend(fg, bg, t):
    """Mélange de deux couleurs #rrggbb : t=0 -> fg, t=1 -> bg."""
    a = [int(fg[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(bg[i:i + 2], 16) for i in (1, 3, 5)]
    return "#{:02x}{:02x}{:02x}".format(*(round(x + (y - x) * t) for x, y in zip(a, b)))
