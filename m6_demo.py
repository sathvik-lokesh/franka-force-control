"""M6: demo GIFs, replayed from real simulation runs of the final (M4 v3) controller.

Captions come from the run's own data. Writes results/demo_search.gif and results/demo_height.gif.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT / "src"))
import m3_peg as m3  # noqa: E402
from franka_model import build_model, reset_home  # noqa: E402

RESULTS = ROOT / "results"
W, H, PLOT_H = 440, 330, 132
FPS = 12
INK, MUTED, GRID, SURFACE = (11, 11, 11), (82, 81, 78), (228, 227, 223), (252, 252, 251)
BLUE, ORANGE = (42, 120, 214), (235, 104, 52)
FONT = ImageFont.load_default(size=14)
SMALL = ImageFont.load_default(size=12)
FINAL = dict(robust=True, bumpless_insert=True, v=0.01, pitch=0.00075)


def trial(controller: str, r: float, seed: int, height_err: float = 0.0) -> tuple[dict, mujoco.MjModel, np.ndarray]:
    """Run once with logging, then rebuild the same true model for replay."""
    res = m3.run(controller, r, seed, height_err=height_err, keep_log=True, **FINAL)
    probe = build_model(task="peg", friction=m3.MU, contact_dmin=m3.DMIN)
    d = mujoco.MjData(probe)
    reset_home(probe, d)
    believed = d.site_xpos[probe.site("peg_tip").id][:2].copy()
    hole = believed + np.array(res["true_offset_mm"]) * 1e-3
    model = build_model(task="peg", friction=m3.MU, contact_dmin=m3.DMIN, hole_xy=tuple(hole),
                        plate_top=m3.PLATE_TOP + height_err)
    return res, model, hole


class Panel:
    """One camera view with a live force trace under it."""

    def __init__(self, res: dict, model: mujoco.MjModel, hole: np.ndarray, colour: tuple, title: str, top: float):
        self.res, self.model, self.colour, self.title = res, model, colour, title
        self.data = mujoco.MjData(model)
        self.renderer = mujoco.Renderer(model, H, W)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = [hole[0], hole[1], top + 0.005]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 0.17, 125, -42
        model.geom_rgba[model.geom("hole_floor").id] = [0.05, 0.05, 0.05, 1]   # visual only: makes the hole readable
        lg = res["log"]
        self.t = np.array(lg["t"])
        self.f = np.array(lg["f_total"])
        self.phase = lg["phase"]
        self.qpos = np.array(lg["qpos"])

    def frame(self, tq: float, t_end: float, y_max: float) -> Image.Image:
        i = min(int(np.searchsorted(self.t, tq)), len(self.t) - 1)
        self.data.qpos[:] = self.qpos[i]
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, self.cam)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        img = Image.new("RGB", (W, H + PLOT_H), SURFACE)
        img.paste(Image.fromarray(self.renderer.render()), (0, 0))
        dr = ImageDraw.Draw(img)
        dr.rectangle((0, 0, W, 24), fill=SURFACE)
        dr.text((8, 4), self.title, font=FONT, fill=INK)
        depth = (self.res["height_err"] + m3.PLATE_TOP - self.res["log"]["z"][i]) * 1e3
        where = f"{depth:.1f} mm deep" if depth > 0.5 else (f"{-depth:.1f} mm above plate" if depth < -0.5 else "on the plate")
        dr.text((8, H + 4), f"t = {self.t[i]:4.1f} s   {self.phase[i]}   peg tip: {where}", font=SMALL, fill=INK)
        x0, x1, y0, y1 = 42, W - 10, H + 30, H + PLOT_H - 22
        for v in np.arange(0, y_max + 1e-6, y_max / 2):
            y = y1 - (y1 - y0) * v / y_max
            dr.line((x0, y, x1, y), fill=GRID)
            dr.text((8, y - 7), f"{v:.0f} N", font=SMALL, fill=MUTED)
        dr.text((x0, y1 + 4), "total contact force on the peg", font=SMALL, fill=MUTED)
        n = np.arange(0, i + 1, 20)
        if len(n) > 1:
            xs = x0 + (x1 - x0) * self.t[n] / t_end
            # Max within each plotted bin, so short spikes survive the downsampling.
            ys = [self.f[max(0, k - 20):k + 1].max() for k in n]
            pts = [(float(x), float(y1 - (y1 - y0) * min(v, y_max) / y_max)) for x, v in zip(xs, ys)]
            dr.line(pts, fill=self.colour, width=2)
        return img


def save(frames: list[Image.Image], path: Path, hold: int = 18) -> None:
    pal = [f.convert("P", palette=Image.Palette.ADAPTIVE, colors=128) for f in frames + [frames[-1]] * hold]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=int(1000 / FPS), loop=0, optimize=True)
    print(path.name, f"{len(pal)} frames, {path.stat().st_size / 1e6:.2f} MB")


def demo_search() -> None:
    res, model, hole = trial("force_spiral", 0.0035, 1000)
    assert res["success"], "demo run must be a real success"
    p = Panel(res, model, hole, BLUE, f"Hole 3.5 mm from where the controller thinks: found in {res['t_search']:.1f} s", m3.PLATE_TOP)
    t_end = res["t_total"] + 0.3
    speed = 2.0
    frames = [p.frame(t, t_end, 20.0) for t in np.arange(0.4, t_end, speed / FPS)]
    p.renderer.close()
    save(frames, RESULTS / "demo_search.gif")


def demo_height() -> None:
    err = -0.003
    a_res, a_model, a_hole = trial("force_spiral", 0.0035, 1000, height_err=err)
    b_res, b_model, b_hole = trial("stiff_spiral", 0.0035, 1000, height_err=err)
    assert a_res["success"] and not b_res["success"], "demo must show what the test showed"
    a = Panel(a_res, a_model, a_hole, BLUE, "Force control", m3.PLATE_TOP + err)
    b = Panel(b_res, b_model, b_hole, ORANGE, "Stiff position control", m3.PLATE_TOP + err)
    t_end = a_res["t_total"] + 1.0
    speed = 2.0
    frames = []
    for t in np.arange(0.4, t_end, speed / FPS):
        img = Image.new("RGB", (2 * W + 6, H + PLOT_H + 24), SURFACE)
        img.paste(a.frame(t, t_end, 20.0), (0, 24))
        img.paste(b.frame(t, t_end, 20.0), (W + 6, 24))
        ImageDraw.Draw(img).text((8, 4), "Plate is 3 mm lower than both controllers believe", font=FONT, fill=INK)
        frames.append(img)
    a.renderer.close()
    b.renderer.close()
    print("force t_total", a_res["t_total"], "stiff final depth", b_res["final_depth_mm"], "stiff peak", b_res["peak_force_N"])
    save(frames, RESULTS / "demo_height.gif")


if __name__ == "__main__":
    demo_search()
    demo_height()
