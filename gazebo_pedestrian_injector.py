#!/usr/bin/env python3
"""gazebo_pedestrian_injector - add walking pedestrians to a Gazebo Classic world.

Two kinds of pedestrian:

  periodic  An animated actor that walks A -> B -> A forever. Gazebo plays the
            animation by itself; an optional plugin gives it a collision box.
  trigger   A standing person model that walks A -> B (or back) when you press
            Ctrl+<key> or publish to /<name>/trigger. Needs the `run` command
            (ROS 2) while the simulation is running.

    python3 gazebo_pedestrian_injector.py                    # editor window
    python3 gazebo_pedestrian_injector.py track.world        # editor on a world
    python3 gazebo_pedestrian_injector.py run track.world    # drive trigger pedestrians
    python3 gazebo_pedestrian_injector.py --help             # all commands

The editor needs tkinter. `run` needs ROS 2 (rclpy); keyboard triggers also
need pynput and an X11 session.
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import re
import sys
import tempfile
import threading
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from typing import Callable, Dict, List, Optional, Tuple

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except ImportError:  # headless: everything except the editor works
    tk = None

__version__ = "0.1.0"

DEFAULT_PLUGIN = "libpedestrian_collision_plugin.so"
RESERVED_KEYS = {"c", "d", "z"}  # Ctrl+C / Ctrl+D / Ctrl+Z belong to the terminal


class PedError(ValueError):
    """Invalid input or a world file that cannot be edited."""


# ======================================================================
# World file: marker blocks
# ======================================================================

NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
TAG_PERIODIC = "pedestrian-injector"
TAG_COLLISION = "pedestrian-injector-collision"
TAG_TRIGGER = "pedestrian-trigger"


def validate_name(name: str) -> str:
    if not NAME_RE.match(name or ""):
        raise PedError(f"invalid name {name!r}: use letters, digits and '_', not starting with a digit")
    return name


def _fmt(v: float) -> str:
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _block_re(tag: str, name: str) -> "re.Pattern[str]":
    n = re.escape(name)
    return re.compile(
        r"(?P<indent>[ \t]*)<!-- \[" + tag + ":" + n + r"\] -->.*?<!-- \[/" + tag + ":" + n + r"\] -->",
        re.DOTALL)


class WorldFile:
    """A world file held in memory. Edit, then :meth:`save`."""

    def __init__(self, text: str, path: Optional[str] = None):
        if "</world>" not in text:
            raise PedError("no </world> tag found; is this an SDF world file?")
        self.text = text
        self.path = path

    @classmethod
    def load(cls, path: str) -> "WorldFile":
        with open(path, "r", encoding="utf-8") as f:
            return cls(f.read(), path)

    def save(self, path: Optional[str] = None) -> str:
        """Atomic write; refuses to write XML that does not parse."""
        path = path or self.path
        if not path:
            raise PedError("no output path")
        try:
            ET.fromstring(self.text)
        except ET.ParseError as exc:
            raise PedError(f"refusing to write invalid XML: {exc}") from exc
        fd, tmp = tempfile.mkstemp(prefix=".gpi-", suffix=".tmp", dir=os.path.dirname(os.path.abspath(path)))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(self.text)
            if os.path.exists(path):
                os.chmod(tmp, os.stat(path).st_mode & 0o7777)
            os.replace(tmp, path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise
        return path

    def names(self, tag: str) -> List[str]:
        out: List[str] = []
        for n in re.findall(r"<!-- \[" + tag + r":([^\]]+)\] -->", self.text):
            if n not in out:
                out.append(n)
        return out

    def block(self, tag: str, name: str) -> Optional[str]:
        m = _block_re(tag, name).search(self.text)
        return m.group(0) if m else None

    def entity_names(self) -> set:
        names = set(re.findall(r"<(?:model|actor|light)\s+name=\"([^\"]+)\"", self.text))
        for inc in re.findall(r"<include>(.*?)</include>", self.text, re.DOTALL):
            m = re.search(r"<name>([^<]+)</name>", inc)
            if m:
                names.add(m.group(1).strip())
        return names

    def _indent(self) -> str:
        m = re.search(r"\n([ \t]*)<world\b", self.text)
        return (m.group(1) if m else "") + "  "

    def _indented(self, block: str, indent: str) -> str:
        return "\n".join(indent + ln if ln else ln for ln in block.splitlines())

    def insert(self, block: str) -> None:
        idx = self.text.rfind("</world>")
        line_start = self.text.rfind("\n", 0, idx) + 1
        body = self._indented(block, self._indent())
        if self.text[line_start:idx].strip():
            self.text = self.text[:idx] + "\n" + body + "\n" + self.text[idx:]
        else:
            self.text = self.text[:line_start] + body + "\n\n" + self.text[line_start:]

    def replace_block(self, tag: str, name: str, block: str) -> bool:
        m = _block_re(tag, name).search(self.text)
        if not m:
            return False
        self.text = self.text[:m.start()] + self._indented(block, m.group("indent")) + self.text[m.end():]
        return True

    def remove_block(self, tag: str, name: str) -> bool:
        m = _block_re(tag, name).search(self.text)
        if not m:
            return False
        start = self.text.rfind("\n", 0, m.start())
        end = m.end() + (1 if self.text[m.end():m.end() + 2] == "\n\n" else 0)
        self.text = self.text[:max(start, 0)] + self.text[end:]
        return True


def _meta(block: str, label: str) -> Dict[str, str]:
    m = re.search(r"<!-- " + label + r": (.*?) -->", block)
    out: Dict[str, str] = {}
    if m:
        for kv in m.group(1).split("|"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                out[k.strip()] = v.strip()
    return out


def _float(meta: Dict[str, str], key: str, default: float) -> float:
    try:
        return float(meta.get(key, default))
    except ValueError:
        return default


# ======================================================================
# Periodic pedestrians (actors)
# ======================================================================

@dataclass
class Periodic:
    name: str
    start: Tuple[float, float]
    end: Tuple[float, float]
    speed: float = 1.2
    z: float = 0.0
    offset: float = 0.0      # seconds into the cycle at t=0, to de-sync several walkers
    wait_start: float = 0.0  # pause at A before each A -> B
    wait_end: float = 0.0    # pause at B before turning back
    plugin: str = DEFAULT_PLUGIN  # "" = no collision box

    kind = "periodic"

    def validate(self) -> None:
        validate_name(self.name)
        if self.speed <= 0:
            raise PedError("speed must be > 0")
        if math.dist(self.start, self.end) < 0.05:
            raise PedError("start and end must be at least 5 cm apart")
        if min(self.wait_start, self.wait_end, self.offset) < 0:
            raise PedError("waits and offset cannot be negative")
        if self.plugin and not re.match(r"^[A-Za-z0-9_./-]+$", self.plugin):
            raise PedError(f"invalid plugin file name {self.plugin!r}")

    def one_way(self) -> float:
        return math.dist(self.start, self.end) / self.speed

    def cycle(self) -> float:
        return self.wait_start + self.wait_end + 2 * self.one_way() + 1.0  # two 0.5 s turns

    def waypoints(self) -> List[Tuple[float, float, float, float]]:
        """(time, x, y, yaw) for one cycle, already shifted by ``offset``."""
        (sx, sy), (ex, ey) = self.start, self.end
        dx, dy = ex - sx, ey - sy
        dist = math.hypot(dx, dy)
        yaw_ab, yaw_ba = math.atan2(dy, dx), math.atan2(-dy, -dx)
        steps = max(1, math.ceil(dist / 0.5))
        dt = self.one_way() / steps
        wps = [(0.0, sx, sy, yaw_ab)]
        t = 0.0
        if self.wait_start > 0:
            t += self.wait_start
            wps.append((t, sx, sy, yaw_ab))
        for i in range(1, steps + 1):
            wps.append((t + i * dt, sx + dx * i / steps, sy + dy * i / steps, yaw_ab))
        t += steps * dt
        if self.wait_end > 0:
            t += self.wait_end
            wps.append((t, ex, ey, yaw_ab))
        t += 0.5
        wps.append((t, ex, ey, yaw_ba))
        for i in range(1, steps + 1):
            wps.append((t + i * dt, ex - dx * i / steps, ey - dy * i / steps, yaw_ba))
        t += steps * dt + 0.5
        wps.append((t, sx, sy, yaw_ab))
        return _shift(wps, self.offset % t if t > 0 else 0.0)

    def blocks(self) -> Tuple[Optional[str], str]:
        """(collision model block or None, actor block)."""
        self.validate()
        wps = self.waypoints()
        traj = "\n".join(
            f"      <waypoint><time>{t:.3f}</time><pose>{x:.3f} {y:.3f} {self.z:.3f} 0 0 {yaw:.4f}</pose></waypoint>"
            for t, x, y, yaw in wps)
        _, x0, y0, yaw0 = wps[0]
        params = "|".join([
            f"sx={_fmt(self.start[0])}", f"sy={_fmt(self.start[1])}",
            f"ex={_fmt(self.end[0])}", f"ey={_fmt(self.end[1])}",
            f"speed={_fmt(self.speed)}", f"z={_fmt(self.z)}", f"offset={_fmt(self.offset)}",
            f"wstart={_fmt(self.wait_start)}", f"wend={_fmt(self.wait_end)}",
        ])
        actor = f"""<!-- [{TAG_PERIODIC}:{self.name}] -->
<!-- PARAMS: {params} -->
<actor name="{self.name}">
  <pose>{x0:.3f} {y0:.3f} {self.z:.3f} 0 0 {yaw0:.4f}</pose>
  <skin><filename>walk.dae</filename><scale>1.0</scale></skin>
  <animation name="walking">
    <filename>walk.dae</filename>
    <scale>1.0</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <loop>true</loop>
    <auto_start>true</auto_start>
    <trajectory id="0" type="walking" tension="1.0">
{traj}
    </trajectory>
  </script>
</actor>
<!-- [/{TAG_PERIODIC}:{self.name}] -->"""
        if not self.plugin:
            return None, actor
        collision = f"""<!-- [{TAG_COLLISION}:{self.name}] -->
<model name="{self.name}_collision">
  <pose>{x0:.3f} {y0:.3f} {self.z + 0.9:.3f} 0 0 {yaw0:.4f}</pose>
  <link name="link">
    <kinematic>true</kinematic>
    <gravity>false</gravity>
    <collision name="collision">
      <geometry><box><size>0.5 0.4 1.8</size></box></geometry>
    </collision>
  </link>
  <plugin name="{self.name}_collision_plugin" filename="{self.plugin}">
    <actor_name>{self.name}</actor_name>
    <z_offset>0.9</z_offset>
  </plugin>
</model>
<!-- [/{TAG_COLLISION}:{self.name}] -->"""
        return collision, actor

    @classmethod
    def parse(cls, world: WorldFile, name: str) -> Optional["Periodic"]:
        block = world.block(TAG_PERIODIC, name)
        if block is None:
            return None
        p = _meta(block, "PARAMS")
        coll = world.block(TAG_COLLISION, name)
        plugin = ""
        if coll:
            m = re.search(r'filename="([^"]+)"', coll)
            plugin = m.group(1) if m else DEFAULT_PLUGIN
        # Older files had "delay", which repeated every loop just like wstart.
        wstart = _float(p, "wstart", 0) + _float(p, "delay", 0)
        return cls(name=name, start=(_float(p, "sx", 0), _float(p, "sy", 0)),
                   end=(_float(p, "ex", 0), _float(p, "ey", 0)), speed=_float(p, "speed", 1.2),
                   z=_float(p, "z", 0), offset=_float(p, "offset", 0), wait_start=wstart,
                   wait_end=_float(p, "wend", 0), plugin=plugin)


def _lerp_angle(a: float, b: float, f: float) -> float:
    d = math.atan2(math.sin(b - a), math.cos(b - a))
    return math.atan2(math.sin(a + d * f), math.cos(a + d * f))


def _shift(wps: List[Tuple[float, float, float, float]], o: float) -> List[Tuple[float, float, float, float]]:
    """Start a looping trajectory ``o`` seconds into its cycle."""
    if o <= 1e-6:
        return wps
    period = wps[-1][0]
    at = None
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(wps, wps[1:]):
        if t0 <= o <= t1:
            f = 0.0 if t1 == t0 else (o - t0) / (t1 - t0)
            at = (x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, _lerp_angle(a0, a1, f))
            break
    assert at is not None
    out = [(0.0,) + at]
    out += [(t - o, x, y, a) for t, x, y, a in wps if t > o + 1e-9]
    out += [(t + period - o, x, y, a) for t, x, y, a in wps[1:] if t < o - 1e-9]
    out.append((period,) + at)
    return out


# ======================================================================
# Trigger pedestrians (person model + planar_move)
# ======================================================================

@dataclass
class Trigger:
    name: str
    key: str
    start: Tuple[float, float]
    end: Tuple[float, float]
    speed: float = 1.2
    z: float = 0.0
    yaw: Optional[float] = None  # facing at spawn; None = towards the end point

    kind = "trigger"

    def validate(self) -> None:
        validate_name(self.name)
        if not re.match(r"^[a-z0-9]$", self.key or ""):
            raise PedError("key must be a single letter or digit")
        if self.key in RESERVED_KEYS:
            raise PedError(f"Ctrl+{self.key.upper()} is used by terminals; pick another key")
        if self.speed <= 0:
            raise PedError("speed must be > 0")
        if math.dist(self.start, self.end) < 0.05:
            raise PedError("start and end must be at least 5 cm apart")

    def spawn_yaw(self) -> float:
        if self.yaw is not None:
            return self.yaw
        return math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])

    def block(self) -> str:
        self.validate()
        sx, sy = self.start
        meta = "|".join([
            f"key={self.key}", f"sx={_fmt(sx)}", f"sy={_fmt(sy)}",
            f"ex={_fmt(self.end[0])}", f"ey={_fmt(self.end[1])}", f"speed={_fmt(self.speed)}",
            f"z={_fmt(self.z)}", "yaw=auto" if self.yaw is None else f"yaw={_fmt(self.yaw)}",
        ])
        return f"""<!-- [{TAG_TRIGGER}:{self.name}] -->
<!-- TRIGGER: {meta} -->
<model name="{self.name}">
  <pose>{sx:.3f} {sy:.3f} {self.z:.3f} 0 0 {self.spawn_yaw():.4f}</pose>
  <link name="link">
    <gravity>false</gravity>
    <inertial>
      <mass>80.0</mass>
      <inertia><ixx>21.6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>24.0</iyy><iyz>0</iyz><izz>3.33</izz></inertia>
    </inertial>
    <visual name="visual">
      <geometry><mesh><uri>model://person_standing/meshes/standing.dae</uri></mesh></geometry>
    </visual>
    <collision name="collision">
      <pose>0 0 0.9 0 0 0</pose>
      <geometry><box><size>0.5 0.4 1.8</size></box></geometry>
    </collision>
  </link>
  <plugin name="{self.name}_planar_move" filename="libgazebo_ros_planar_move.so">
    <ros><namespace>/{self.name}</namespace></ros>
    <update_rate>100</update_rate>
    <publish_rate>50</publish_rate>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>false</publish_odom_tf>
    <odometry_frame>{self.name}/odom</odometry_frame>
    <robot_base_frame>{self.name}/base</robot_base_frame>
  </plugin>
</model>
<!-- [/{TAG_TRIGGER}:{self.name}] -->"""

    @classmethod
    def parse(cls, world: WorldFile, name: str) -> Optional["Trigger"]:
        block = world.block(TAG_TRIGGER, name)
        if block is None:
            return None
        p = _meta(block, "TRIGGER")
        yaw_txt = p.get("yaw", "auto").lower()
        try:
            yaw = None if yaw_txt == "auto" else float(yaw_txt)
        except ValueError:
            yaw = None
        return cls(name=name, key=p.get("key", "?").lower(), start=(_float(p, "sx", 0), _float(p, "sy", 0)),
                   end=(_float(p, "ex", 0), _float(p, "ey", 0)), speed=_float(p, "speed", 1.2),
                   z=_float(p, "z", 0), yaw=yaw)


# ======================================================================
# Editing pedestrians in a world
# ======================================================================

def periodic_names(w: WorldFile) -> List[str]:
    return w.names(TAG_PERIODIC)


def trigger_names(w: WorldFile) -> List[str]:
    return w.names(TAG_TRIGGER)


def load_all(w: WorldFile) -> Tuple[List[Periodic], List[Trigger]]:
    per = [p for p in (Periodic.parse(w, n) for n in periodic_names(w)) if p]
    trg = [t for t in (Trigger.parse(w, n) for n in trigger_names(w)) if t]
    return per, trg


def _check_free(w: WorldFile, name: str, ignore: Optional[str] = None) -> None:
    taken = set(w.entity_names()) | set(periodic_names(w)) | set(trigger_names(w))
    if ignore:
        taken -= {ignore, f"{ignore}_collision"}
    for n in (name, f"{name}_collision"):
        if n in taken:
            raise PedError(f"an entity named {n!r} already exists in the world")


def _check_key(w: WorldFile, ped: Trigger, ignore: Optional[str] = None) -> None:
    for t in load_all(w)[1]:
        if t.name != ignore and t.key == ped.key:
            raise PedError(f"key {ped.key!r} is already used by {t.name}")


def add(w: WorldFile, ped) -> None:
    """Add a Periodic or Trigger. Builds the XML first, so a bad input changes nothing."""
    _check_free(w, ped.name)
    if isinstance(ped, Trigger):
        _check_key(w, ped)
        w.insert(ped.block())
    else:
        collision, actor = ped.blocks()
        if collision:
            w.insert(collision)
        w.insert(actor)


def update(w: WorldFile, old_name: str, ped) -> None:
    """Replace a pedestrian in place. Builds the XML first, so a bad input changes nothing."""
    if ped.name != old_name:
        _check_free(w, ped.name, ignore=old_name)
    if isinstance(ped, Trigger):
        if w.block(TAG_TRIGGER, old_name) is None:
            raise PedError(f"no trigger pedestrian {old_name!r}")
        _check_key(w, ped, ignore=old_name)
        block = ped.block()
        w.replace_block(TAG_TRIGGER, old_name, block)
        return
    if w.block(TAG_PERIODIC, old_name) is None:
        raise PedError(f"no periodic pedestrian {old_name!r}")
    collision, actor = ped.blocks()
    w.replace_block(TAG_PERIODIC, old_name, actor)
    if collision is None:
        w.remove_block(TAG_COLLISION, old_name)
    elif not w.replace_block(TAG_COLLISION, old_name, collision):
        m = _block_re(TAG_PERIODIC, ped.name).search(w.text)
        w.text = w.text[:m.start()] + w._indented(collision, m.group("indent")) + "\n\n" + w.text[m.start():]


def remove(w: WorldFile, name: str) -> None:
    found = False
    for tag in (TAG_PERIODIC, TAG_COLLISION, TAG_TRIGGER):
        found |= w.remove_block(tag, name)
    if not found:
        raise PedError(f"no pedestrian {name!r}")


# ======================================================================
# Runtime: driving trigger pedestrians (ROS 2)
# ======================================================================

def control_step(x: float, y: float, yaw: float, tx: float, ty: float, speed: float,
                 tol: float = 0.05) -> Tuple[float, float, float, bool]:
    """Body-frame (vx, vy, wz) towards (tx, ty); True when arrived.

    Walks straight at the target whatever the current heading, and turns the
    body to face the direction of travel while walking.
    """
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < tol:
        return 0.0, 0.0, 0.0, True
    v = min(speed, 1.5 * dist)  # slow down over the last metre
    heading = math.atan2(dy, dx)
    c, s = math.cos(yaw), math.sin(yaw)
    vxw, vyw = v * math.cos(heading), v * math.sin(heading)
    err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
    wz = max(-2.0, min(2.0, 3.0 * err))
    return c * vxw + s * vyw, -s * vxw + c * vyw, wz, False


class TriggerController:
    """Closed-loop A <-> B walking for trigger pedestrians.

    Per pedestrian NAME it uses
      /NAME/cmd_vel   (out, geometry_msgs/Twist)   to libgazebo_ros_planar_move
      /NAME/odom      (in,  nav_msgs/Odometry)     world pose from the same plugin
      /NAME/trigger   (in,  std_msgs/Empty)        walk to the other end
      /NAME/state     (out, std_msgs/String, latched): at_a, at_b, to_a, to_b, unknown
    Keyboard presses and topic messages are queued and handled on the timer,
    so every ROS call happens on the executor thread.
    """

    def __init__(self, node, peds: List[Trigger], on_state: Optional[Callable[[str, str], None]] = None):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Empty, String

        self.node = node
        self.Twist, self.String = Twist, String
        self.on_state = on_state
        self.requests: "queue.Queue[str]" = queue.Queue()
        self.peds: Dict[str, dict] = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for p in peds:
            st = {"ped": p, "pose": None, "target": None, "state": "", "last_odom": None, "warned": False}
            st["cmd"] = node.create_publisher(Twist, f"/{p.name}/cmd_vel", 10)
            st["state_pub"] = node.create_publisher(String, f"/{p.name}/state", latched)
            node.create_subscription(Odometry, f"/{p.name}/odom", lambda m, n=p.name: self._odom(n, m), 10)
            node.create_subscription(Empty, f"/{p.name}/trigger", lambda _m, n=p.name: self.trigger(n), 10)
            self.peds[p.name] = st
        self._set_all("unknown")
        self.timer = node.create_timer(0.05, self._tick, clock=self._steady_clock())
        self._checked = False
        self._t0 = self._now()

    @staticmethod
    def _steady_clock():
        from rclpy.clock import Clock, ClockType
        return Clock(clock_type=ClockType.STEADY_TIME)

    def _now(self) -> float:
        import time
        return time.monotonic()

    def _set_all(self, state: str) -> None:
        for name in self.peds:
            self._set_state(name, state)

    def _set_state(self, name: str, state: str) -> None:
        st = self.peds[name]
        if st["state"] == state:
            return
        st["state"] = state
        st["state_pub"].publish(self.String(data=state))
        if self.on_state:
            self.on_state(name, state)

    def trigger(self, name: str) -> None:
        """Thread-safe: may be called from the keyboard thread."""
        self.requests.put(name)

    def _odom(self, name: str, msg) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        st = self.peds[name]
        st["pose"] = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        st["last_odom"] = self._now()
        if st["target"] is None:
            p = st["ped"]
            x, y, _ = st["pose"]
            da, db = math.dist((x, y), p.start), math.dist((x, y), p.end)
            self._set_state(name, "at_a" if da <= 0.1 else "at_b" if db <= 0.1 else "between")

    def _tick(self) -> None:
        while True:
            try:
                name = self.requests.get_nowait()
            except queue.Empty:
                break
            self._start_walk(name)
        for name, st in self.peds.items():
            if st["target"] is None or st["pose"] is None:
                continue
            p = st["ped"]
            target = st["target"]
            tx, ty = p.end if target == "b" else p.start
            vx, vy, wz, done = control_step(*st["pose"], tx, ty, p.speed)
            msg = self.Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
            st["cmd"].publish(msg)  # planar_move has no timeout, so the final zero matters
            if done:
                st["target"] = None
                self._set_state(name, f"at_{target}")
            elif self._now() - (st["last_odom"] or 0) > 1.0 and not st["warned"]:
                st["warned"] = True
                self.node.get_logger().warning(f"{name}: no odometry for 1 s (simulation paused or stopped?)")
        if not self._checked and self._now() - self._t0 > 2.0:
            self._checked = True
            self._check_setup()

    def _start_walk(self, name: str) -> None:
        st = self.peds.get(name)
        if st is None:
            return
        log = self.node.get_logger()
        if st["target"] is not None:
            log.info(f"{name}: already walking, trigger ignored")
            return
        if st["pose"] is None:
            log.warning(f"{name}: no odometry on /{name}/odom yet; is the simulation running with this world?")
            return
        p = st["ped"]
        x, y, _ = st["pose"]
        st["target"] = "b" if math.dist((x, y), p.start) <= math.dist((x, y), p.end) else "a"
        st["warned"] = False
        self._set_state(name, "to_b" if st["target"] == "b" else "to_a")

    def _check_setup(self) -> None:
        log = self.node.get_logger()
        for name, st in self.peds.items():
            if st["pose"] is None:
                log.warning(f"{name}: nothing on /{name}/odom (pedestrian missing from the running world?)")
            if self.node.count_publishers(f"/{name}/cmd_vel") > 1:
                log.warning(f"{name}: another node also publishes /{name}/cmd_vel (second controller running?)")

    def stop(self) -> None:
        for st in self.peds.values():
            st["target"] = None
            st["cmd"].publish(self.Twist())


def start_keyboard(ctrl: TriggerController, peds: List[Trigger]):
    """Ctrl+<key> hotkeys. Returns the listener or None (with the reason printed)."""
    try:
        from pynput import keyboard
    except ImportError:
        print("keyboard triggers off: pynput is not installed (pip install pynput)", file=sys.stderr)
        return None
    if os.environ.get("XDG_SESSION_TYPE") == "wayland":
        print("note: global hotkeys may not work in a Wayland session; /<name>/trigger always works",
              file=sys.stderr)
    hotkeys = {f"<ctrl>+{p.key}": (lambda n=p.name: ctrl.trigger(n)) for p in peds}
    try:
        listener = keyboard.GlobalHotKeys(hotkeys)
        listener.start()
    except Exception as exc:  # no display, no X server, ...
        print(f"keyboard triggers off: {exc}", file=sys.stderr)
        return None
    return listener


class Runner:
    """Owns rclpy, the node, the executor thread and the keyboard listener."""

    def __init__(self, world_path: str, keyboard: bool = True,
                 on_state: Optional[Callable[[str, str], None]] = None):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as exc:
            raise PedError("ROS 2 (rclpy) is not available; source your ROS 2 setup.bash first") from exc
        peds = load_all(WorldFile.load(world_path))[1]
        if not peds:
            raise PedError("no trigger pedestrians in this world")
        self.rclpy = rclpy
        self._own_init = not rclpy.ok()
        if self._own_init:
            # Keep the ROS context alive on Ctrl+C so close() can still send the
            # final zero velocity (planar_move would otherwise keep sliding).
            try:
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            except (ImportError, TypeError):
                rclpy.init()
        self.node = rclpy.create_node("pedestrian_trigger")
        self.ctrl = TriggerController(self.node, peds, on_state)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.listener = start_keyboard(self.ctrl, peds) if keyboard else None
        self.peds = peds
        self.thread: Optional[threading.Thread] = None

    def spin_in_background(self) -> None:
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self) -> None:
        if self.listener:
            self.listener.stop()
        try:
            self.ctrl.stop()
        except Exception:
            pass
        self.executor.shutdown()
        if self.thread:
            self.thread.join(timeout=2.0)
        self.node.destroy_node()
        if self._own_init and self.rclpy.ok():
            self.rclpy.shutdown()


# ======================================================================
# Editor window (tkinter)
# ======================================================================

_CACHE = os.path.expanduser("~/.cache/gazebo_pedestrian_injector/last_world")


def _remember(path: str) -> None:
    try:
        os.makedirs(os.path.dirname(_CACHE), exist_ok=True)
        with open(_CACHE, "w", encoding="utf-8") as f:
            f.write(path)
    except OSError:
        pass


def _last_world() -> str:
    try:
        with open(_CACHE, encoding="utf-8") as f:
            p = f.read().strip()
        return p if os.path.isfile(p) else ""
    except OSError:
        return ""


class Form:
    """A column of labelled entries backed by StringVars."""

    def __init__(self, parent, fields: List[Tuple[str, str, str]]):
        self.frame = ttk.Frame(parent)
        self.vars: Dict[str, "tk.StringVar"] = {}
        for r, (key, label, default) in enumerate(fields):
            ttk.Label(self.frame, text=label).grid(row=r, column=0, sticky="w", pady=2)
            v = tk.StringVar(value=default)
            ttk.Entry(self.frame, textvariable=v, width=22).grid(row=r, column=1, sticky="ew", padx=(8, 0), pady=2)
            self.vars[key] = v

    def get(self, key: str) -> str:
        return self.vars[key].get().strip()

    def num(self, key: str) -> float:
        try:
            return float(self.get(key))
        except ValueError as exc:
            raise PedError(f"{key}: not a number") from exc

    def set(self, key: str, value) -> None:
        self.vars[key].set(_fmt(value) if isinstance(value, float) else str(value))


class App:
    def __init__(self, root, world: Optional[str]):
        self.root = root
        root.title("Gazebo Pedestrian Injector")
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        self.world_var = tk.StringVar(value=world or _last_world())
        self.status = tk.StringVar()
        self.events: "queue.Queue[Tuple[str, str]]" = queue.Queue()
        self.runner: Optional[Runner] = None

        top = ttk.Frame(root, padding=(12, 12, 12, 0))
        top.pack(fill="x")
        ttk.Label(top, text="World").pack(side="left")
        ttk.Entry(top, textvariable=self.world_var, width=60).pack(side="left", padx=6, fill="x", expand=True)
        ttk.Button(top, text="Open…", command=self.open_world).pack(side="left")
        ttk.Button(top, text="Reload", command=self.refresh).pack(side="left", padx=(6, 0))

        self.nb = ttk.Notebook(root, padding=12)
        self.nb.pack(fill="both", expand=True)
        self._build_periodic()
        self._build_trigger()
        self._build_run()
        ttk.Label(root, textvariable=self.status, anchor="w", padding=(12, 0, 12, 8)).pack(fill="x")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self._poll()

    # ------------------------------------------------------------ plumbing
    def _world(self) -> WorldFile:
        path = self.world_var.get().strip()
        if not path or not os.path.isfile(path):
            raise PedError("choose a world file first")
        return WorldFile.load(path)

    def _do(self, action: Callable[[WorldFile], str]) -> None:
        try:
            w = self._world()
            msg = action(w)
            w.save()
        except (PedError, OSError) as exc:
            messagebox.showerror("Error", str(exc), parent=self.root)
            return
        self.refresh()
        self.status.set(msg)

    def open_world(self) -> None:
        path = filedialog.askopenfilename(title="World file",
                                          filetypes=[("SDF world", "*.world *.sdf"), ("All files", "*")])
        if path:
            self.world_var.set(path)
            self.refresh()

    def refresh(self) -> None:
        for tree in (self.p_tree, self.t_tree):
            tree.delete(*tree.get_children())
        try:
            w = self._world()
        except (PedError, OSError) as exc:
            self.status.set(str(exc))
            return
        if w.path:
            _remember(os.path.abspath(w.path))
        per, trg = load_all(w)
        for p in per:
            self.p_tree.insert("", "end", iid=p.name, values=(
                p.name, f"{_fmt(p.start[0])}, {_fmt(p.start[1])}", f"{_fmt(p.end[0])}, {_fmt(p.end[1])}",
                f"{_fmt(p.speed)} m/s", f"{p.cycle():.1f} s", "yes" if p.plugin else "no"))
        for t in trg:
            self.t_tree.insert("", "end", iid=t.name, values=(
                t.name, f"Ctrl+{t.key.upper()}", f"{_fmt(t.start[0])}, {_fmt(t.start[1])}",
                f"{_fmt(t.end[0])}, {_fmt(t.end[1])}", f"{_fmt(t.speed)} m/s"))
        self._refresh_run(trg)
        self.status.set(f"{len(per)} periodic, {len(trg)} trigger pedestrians")

    def _tree(self, parent, cols: List[Tuple[str, int]], on_select) -> "ttk.Treeview":
        tree = ttk.Treeview(parent, columns=[c for c, _ in cols], show="headings", height=13, selectmode="browse")
        for c, w in cols:
            tree.heading(c, text=c)
            tree.column(c, width=w, minwidth=40)
        tree.grid(row=0, column=0, sticky="nsew")
        tree.bind("<<TreeviewSelect>>", on_select)
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)
        return tree

    def _buttons(self, parent, row: int, add, upd, delete) -> None:
        b = ttk.Frame(parent)
        b.grid(row=row, column=0, sticky="w", pady=(10, 0))
        ttk.Button(b, text="Add", command=add).pack(side="left")
        ttk.Button(b, text="Update", command=upd).pack(side="left", padx=6)
        ttk.Button(b, text="Delete", command=delete).pack(side="left")

    def _delete(self, tree) -> None:
        sel = tree.selection()
        if sel and messagebox.askyesno("Delete", f"Remove {sel[0]} from the world?", parent=self.root):
            self._do(lambda w: (remove(w, sel[0]), f"removed {sel[0]}")[1])

    def _update(self, tree, build) -> None:
        sel = tree.selection()
        if not sel:
            messagebox.showinfo("Update", "Select a pedestrian in the list first.", parent=self.root)
            return
        self._do(lambda w: (update(w, sel[0], build()), f"updated {sel[0]}")[1])

    def _add(self, build) -> None:
        def act(w: WorldFile) -> str:
            ped = build()
            add(w, ped)
            return f"added {ped.name}"
        self._do(act)

    # ------------------------------------------------------------ periodic
    def _build_periodic(self) -> None:
        tab = ttk.Frame(self.nb, padding=8)
        self.nb.add(tab, text="Periodic")
        self.p_tree = self._tree(tab, [("Name", 120), ("Start", 90), ("End", 90), ("Speed", 70),
                                       ("Cycle", 60), ("Collision", 70)], self._p_selected)
        side = ttk.Frame(tab, padding=(12, 0, 0, 0))
        side.grid(row=0, column=1, sticky="n")
        ttk.Label(side, text="Walks A → B → A in a loop (animated actor).", foreground="#666").grid(
            row=0, column=0, sticky="w", pady=(0, 6))
        self.p_form = Form(side, [
            ("name", "Name", "pedestrian_1"), ("sx", "Start x", "0"), ("sy", "Start y", "-3"),
            ("ex", "End x", "0"), ("ey", "End y", "3"), ("speed", "Speed (m/s)", "1.2"), ("z", "Z", "0"),
            ("offset", "Cycle offset (s)", "0"), ("wstart", "Wait at start (s)", "0"),
            ("wend", "Wait at end (s)", "0"), ("plugin", "Collision plugin", DEFAULT_PLUGIN),
        ])
        self.p_form.frame.grid(row=1, column=0, sticky="ew")
        ttk.Label(side, text="Empty plugin = no collision box.\n"
                             "Offset starts the loop part-way, so walkers do not move in sync.",
                  foreground="#666", justify="left").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self._buttons(side, 3, lambda: self._add(self._p_build), lambda: self._update(self.p_tree, self._p_build),
                      lambda: self._delete(self.p_tree))

    def _p_build(self) -> Periodic:
        f = self.p_form
        p = Periodic(name=f.get("name"), start=(f.num("sx"), f.num("sy")), end=(f.num("ex"), f.num("ey")),
                     speed=f.num("speed"), z=f.num("z"), offset=f.num("offset"), wait_start=f.num("wstart"),
                     wait_end=f.num("wend"), plugin=f.get("plugin"))
        p.validate()
        return p

    def _p_selected(self, _e=None) -> None:
        sel = self.p_tree.selection()
        if not sel:
            return
        try:
            p = Periodic.parse(self._world(), sel[0])
        except (PedError, OSError):
            return
        if p:
            f = self.p_form
            for k, v in (("name", p.name), ("sx", p.start[0]), ("sy", p.start[1]), ("ex", p.end[0]),
                         ("ey", p.end[1]), ("speed", p.speed), ("z", p.z), ("offset", p.offset),
                         ("wstart", p.wait_start), ("wend", p.wait_end), ("plugin", p.plugin)):
                f.set(k, v)

    # ------------------------------------------------------------- trigger
    def _build_trigger(self) -> None:
        tab = ttk.Frame(self.nb, padding=8)
        self.nb.add(tab, text="Trigger")
        self.t_tree = self._tree(tab, [("Name", 130), ("Key", 70), ("Start", 100), ("End", 100), ("Speed", 80)],
                                 self._t_selected)
        side = ttk.Frame(tab, padding=(12, 0, 0, 0))
        side.grid(row=0, column=1, sticky="n")
        ttk.Label(side, text="Stands still until triggered, then walks\nto the other end (see the Run tab).",
                  foreground="#666", justify="left").grid(row=0, column=0, sticky="w", pady=(0, 6))
        self.t_form = Form(side, [
            ("name", "Name", "trigger_ped_1"), ("key", "Key (Ctrl+…)", "l"), ("sx", "Start x", "5"),
            ("sy", "Start y", "-3"), ("ex", "End x", "5"), ("ey", "End y", "3"), ("speed", "Speed (m/s)", "1.2"),
            ("z", "Z", "0"), ("yaw", "Initial yaw (rad)", "auto"),
        ])
        self.t_form.frame.grid(row=1, column=0, sticky="ew")
        self._buttons(side, 2, lambda: self._add(self._t_build), lambda: self._update(self.t_tree, self._t_build),
                      lambda: self._delete(self.t_tree))

    def _t_build(self) -> Trigger:
        f = self.t_form
        yaw_txt = f.get("yaw").lower()
        yaw = None if yaw_txt in ("", "auto") else f.num("yaw")
        t = Trigger(name=f.get("name"), key=f.get("key").lower(), start=(f.num("sx"), f.num("sy")),
                    end=(f.num("ex"), f.num("ey")), speed=f.num("speed"), z=f.num("z"), yaw=yaw)
        t.validate()
        return t

    def _t_selected(self, _e=None) -> None:
        sel = self.t_tree.selection()
        if not sel:
            return
        try:
            t = Trigger.parse(self._world(), sel[0])
        except (PedError, OSError):
            return
        if t:
            f = self.t_form
            for k, v in (("name", t.name), ("key", t.key), ("sx", t.start[0]), ("sy", t.start[1]),
                         ("ex", t.end[0]), ("ey", t.end[1]), ("speed", t.speed), ("z", t.z),
                         ("yaw", "auto" if t.yaw is None else t.yaw)):
                f.set(k, v)

    # ----------------------------------------------------------------- run
    def _build_run(self) -> None:
        tab = ttk.Frame(self.nb, padding=8)
        self.nb.add(tab, text="Run")
        bar = ttk.Frame(tab)
        bar.grid(row=0, column=0, sticky="w")
        self.kb_var = tk.BooleanVar(value=True)
        self.run_btn = ttk.Button(bar, text="Start", command=self.toggle_run)
        self.run_btn.pack(side="left")
        ttk.Checkbutton(bar, text="Keyboard (Ctrl+key)", variable=self.kb_var).pack(side="left", padx=10)
        self.r_tree = ttk.Treeview(tab, columns=("Name", "Key", "Topic", "State"), show="headings", height=11)
        for c, w in (("Name", 140), ("Key", 70), ("Topic", 260), ("State", 120)):
            self.r_tree.heading(c, text=c)
            self.r_tree.column(c, width=w)
        self.r_tree.grid(row=1, column=0, sticky="nsew", pady=8)
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)
        ttk.Label(tab, foreground="#666", justify="left", text=(
            "Start this while Gazebo runs the same world. Ctrl+key or\n"
            "ros2 topic pub --once /NAME/trigger std_msgs/msg/Empty {}  sends the pedestrian to the other end.\n"
            "Double-click a row to trigger it from here.")).grid(row=2, column=0, sticky="w")
        self.r_tree.bind("<Double-1>", self._run_click)

    def _refresh_run(self, trg: List[Trigger]) -> None:
        old = {i: self.r_tree.set(i, "State") for i in self.r_tree.get_children()}
        self.r_tree.delete(*self.r_tree.get_children())
        for t in trg:
            self.r_tree.insert("", "end", iid=t.name, values=(
                t.name, f"Ctrl+{t.key.upper()}", f"/{t.name}/trigger", old.get(t.name, "-")))

    def toggle_run(self) -> None:
        if self.runner:
            self.runner.close()
            self.runner = None
            self.run_btn.config(text="Start")
            self.status.set("controller stopped")
            return
        try:
            self.runner = Runner(self.world_var.get().strip(), keyboard=self.kb_var.get(),
                                 on_state=lambda n, s: self.events.put((n, s)))
        except (PedError, OSError) as exc:
            messagebox.showerror("Run", str(exc), parent=self.root)
            return
        self.runner.spin_in_background()
        self.run_btn.config(text="Stop")
        self.status.set(f"controller running for {len(self.runner.peds)} trigger pedestrians")

    def _run_click(self, _e=None) -> None:
        sel = self.r_tree.selection()
        if sel and self.runner:
            self.runner.ctrl.trigger(sel[0])

    def _poll(self) -> None:
        while True:
            try:
                name, state = self.events.get_nowait()
            except queue.Empty:
                break
            if self.r_tree.exists(name):
                self.r_tree.set(name, "State", state)
        self.root.after(100, self._poll)

    def close(self) -> None:
        if self.runner:
            self.runner.close()
        self.root.destroy()


def run_gui(world: Optional[str]) -> int:
    if tk is None:
        raise PedError("tkinter is not installed (on Ubuntu: sudo apt install python3-tk)")
    root = tk.Tk()
    App(root, world)
    root.mainloop()
    return 0


# ======================================================================
# Command line
# ======================================================================

def cmd_list(a) -> int:
    per, trg = load_all(WorldFile.load(a.world))
    for p in per:
        print(f"periodic  {p.name:<20} ({_fmt(p.start[0])}, {_fmt(p.start[1])}) <-> "
              f"({_fmt(p.end[0])}, {_fmt(p.end[1])})  {_fmt(p.speed)} m/s  cycle {p.cycle():.1f} s"
              f"  {'collision' if p.plugin else 'no collision'}")
    for t in trg:
        print(f"trigger   {t.name:<20} ({_fmt(t.start[0])}, {_fmt(t.start[1])}) <-> "
              f"({_fmt(t.end[0])}, {_fmt(t.end[1])})  {_fmt(t.speed)} m/s  Ctrl+{t.key.upper()}")
    if not per and not trg:
        print("no pedestrians")
    return 0


def cmd_add_periodic(a) -> int:
    w = WorldFile.load(a.world)
    p = Periodic(a.name, tuple(a.start), tuple(a.end), a.speed, a.z, a.offset, a.wait_start, a.wait_end,
                 "" if a.no_collision else a.plugin)
    add(w, p)
    w.save()
    print(f"added {p.name} (cycle {p.cycle():.1f} s)")
    return 0


def cmd_add_trigger(a) -> int:
    w = WorldFile.load(a.world)
    t = Trigger(a.name, a.key.lower(), tuple(a.start), tuple(a.end), a.speed, a.z, a.yaw)
    add(w, t)
    w.save()
    print(f"added {t.name} (Ctrl+{t.key.upper()}, /{t.name}/trigger)")
    return 0


def cmd_edit(a) -> int:
    w = WorldFile.load(a.world)
    ped = Periodic.parse(w, a.name) or Trigger.parse(w, a.name)
    if ped is None:
        raise PedError(f"no pedestrian {a.name!r}")
    changes = {k: v for k, v in (("name", a.new_name), ("speed", a.speed), ("z", a.z)) if v is not None}
    if a.start:
        changes["start"] = tuple(a.start)
    if a.end:
        changes["end"] = tuple(a.end)
    if isinstance(ped, Periodic):
        for k, v in (("offset", a.offset), ("wait_start", a.wait_start), ("wait_end", a.wait_end)):
            if v is not None:
                changes[k] = v
    else:
        if a.key:
            changes["key"] = a.key.lower()
        if a.yaw is not None:
            changes["yaw"] = None if a.yaw == "auto" else float(a.yaw)
    new = replace(ped, **changes)
    update(w, a.name, new)
    w.save()
    print(f"updated {new.name}")
    return 0


def cmd_remove(a) -> int:
    w = WorldFile.load(a.world)
    for n in a.names:
        remove(w, n)
    w.save()
    print(f"removed {', '.join(a.names)}")
    return 0


def cmd_run(a) -> int:
    runner = Runner(a.world, keyboard=not a.no_keyboard,
                    on_state=lambda n, s: print(f"{n}: {s}", flush=True))
    for p in runner.peds:
        how = f"Ctrl+{p.key.upper()} or " if runner.listener else ""
        print(f"{p.name}: {how}ros2 topic pub --once /{p.name}/trigger std_msgs/msg/Empty {{}}")
    print("Ctrl+C to quit")
    try:
        runner.executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # e.g. ExternalShutdownException
        print(f"stopped: {exc}", file=sys.stderr)
    finally:
        runner.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="gazebo_pedestrian_injector.py",
                                description="Add walking pedestrians to a Gazebo Classic world.")
    p.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = p.add_subparsers(dest="cmd", metavar="COMMAND")

    s = sub.add_parser("list", help="show the pedestrians in a world")
    s.add_argument("world")
    s.set_defaults(func=cmd_list)

    def common(s, trigger: bool):
        s.add_argument("world")
        s.add_argument("name")
        s.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"), required=True)
        s.add_argument("--end", nargs=2, type=float, metavar=("X", "Y"), required=True)
        s.add_argument("--speed", type=float, default=1.2, help="m/s (default 1.2)")
        s.add_argument("--z", type=float, default=0.0)

    s = sub.add_parser("add-periodic", help="actor that walks start -> end -> start forever")
    common(s, False)
    s.add_argument("--offset", type=float, default=0.0, help="start this many seconds into the loop")
    s.add_argument("--wait-start", type=float, default=0.0, help="pause at start before each walk (s)")
    s.add_argument("--wait-end", type=float, default=0.0, help="pause at end before turning back (s)")
    s.add_argument("--plugin", default=DEFAULT_PLUGIN, help=f"collision plugin file (default {DEFAULT_PLUGIN})")
    s.add_argument("--no-collision", action="store_true", help="no collision box")
    s.set_defaults(func=cmd_add_periodic)

    s = sub.add_parser("add-trigger", help="person that walks to the other end when triggered")
    common(s, True)
    s.add_argument("--key", required=True, help="Ctrl+KEY triggers it")
    s.add_argument("--yaw", type=float, help="facing at spawn (default: towards the end)")
    s.set_defaults(func=cmd_add_trigger)

    s = sub.add_parser("edit", help="change a pedestrian in place")
    s.add_argument("world")
    s.add_argument("name")
    s.add_argument("--name", dest="new_name")
    s.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"))
    s.add_argument("--end", nargs=2, type=float, metavar=("X", "Y"))
    s.add_argument("--speed", type=float)
    s.add_argument("--z", type=float)
    s.add_argument("--offset", type=float)
    s.add_argument("--wait-start", type=float)
    s.add_argument("--wait-end", type=float)
    s.add_argument("--key")
    s.add_argument("--yaw", help="number or 'auto'")
    s.set_defaults(func=cmd_edit)

    s = sub.add_parser("remove", help="remove pedestrians")
    s.add_argument("world")
    s.add_argument("names", nargs="+")
    s.set_defaults(func=cmd_remove)

    s = sub.add_parser("run", help="drive the trigger pedestrians while Gazebo runs (needs ROS 2)")
    s.add_argument("world")
    s.add_argument("--no-keyboard", action="store_true", help="only react to /NAME/trigger")
    s.set_defaults(func=cmd_run)

    s = sub.add_parser("gui", help="open the editor window (needs tkinter)")
    s.add_argument("world", nargs="?")
    s.set_defaults(func=lambda a: run_gui(a.world))
    return p


COMMANDS = {"list", "add-periodic", "add-trigger", "edit", "remove", "run", "gui"}


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or (argv[0] not in COMMANDS and not argv[0].startswith("-")):
        argv = ["gui"] + argv  # no command: open the editor (optionally on the given world)
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (PedError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
