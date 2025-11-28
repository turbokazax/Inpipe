import tkinter as tk
from tkinter import ttk
import threading
import math

from Mechas.DCMotor import DCMotor
from Misc.OpModes import OpModes

motor1 = DCMotor(0)
motor1.enableTorque()
motor1.setOpMode(OpModes.EXTENDED_POSITION)
motor1.setReverseMode(False)
motor2 = DCMotor(1)
motor2.enableTorque()
motor2.setOpMode(OpModes.EXTENDED_POSITION)
motor2.setReverseMode(False)

motors = {
    "Motor 0": motor1,
    "Motor 1": motor2,
}
activeMotor = motor1
# ---- Helper wrappers you can map to your real API ----
def set_velocity(vel_raw: float):
    """
    Set motor velocity in RAW Dynamixel units (0..2920 per your note).
    Maps directly to DCMotor.setGoalVelocity().
    """
    try:
        activeMotor.setGoalVelocity(int(vel_raw), verbose=False)
    except Exception as e:
        raise e

def set_goal_angle(angle_deg: float):
    """
    Accept degrees from the GUI and convert to ticks for DCMotor.setGoalPosition().
    """
    try:
        ticks = int(round(float(angle_deg) / activeMotor.AnglePerPosTick))
        activeMotor.setGoalPosition(ticks, verbose=False)
    except Exception as e:
        raise e

def get_present_angle_safe():
    """
    Read present position in ticks and convert to degrees for display.
    Returns (ticks, degrees) or (None, None) on failure.
    """
    try:
        ticks = activeMotor.getCurrentPosition(verbose=False)
        deg = float(ticks) * activeMotor.AnglePerPosTick
        return ticks, deg
    except Exception:
        return None, None
# ------------------------------------------------------

# ---- Replace these with your real motor functions (kept, still used by buttons) ----
def rotate_a_cw(degrees: float):
    activeMotor.rotateByAngle(degrees)

def rotate_b_ccw(degrees: float):
    activeMotor.rotateByAngle(-degrees)
# ------------------------------------------------------

class MotorGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        # --- Top bar: OpMode selector + live position ---
        top = ttk.Frame(self, padding=(12, 12, 12, 0))
        top.pack(fill="x")

        ttk.Label(top, text="OpMode:").pack(side="left")
        self.mode_var = tk.StringVar(value="POSITION")
        self.mode_cb = ttk.Combobox(top, width=22, state="readonly",
                                    values=["VELOCITY", "POSITION", "EXTENDED_POSITION", "PWM"],
                                    textvariable=self.mode_var)
        self.mode_cb.pack(side="left", padx=(6, 12))
        self.mode_cb.bind("<<ComboboxSelected>>", self.on_mode_change)

        ttk.Label(top, text="Motor:").pack(side="left")
        self.motor_var = tk.StringVar(value="Motor 0")
        self.motor_cb = ttk.Combobox(top, width=10, state="readonly",
                                     values=list(motors.keys()),
                                     textvariable=self.motor_var)
        self.motor_cb.pack(side="left", padx=(6, 12))
        self.motor_cb.bind("<<ComboboxSelected>>", self.on_motor_change)

        # Direction indicator, toggle, and reset encoder (stacked vertically)
        dir_frame = ttk.Frame(top)
        dir_frame.pack(side="left", padx=(6, 4))
        self.dir_var = tk.StringVar(value="Dir: ?")
        self.dir_label = ttk.Label(dir_frame, textvariable=self.dir_var)
        self.dir_label.pack(side="top", anchor="w")
        self.dir_btn = ttk.Button(dir_frame, text="Toggle dir", command=self.on_toggle_dir)
        self.dir_btn.pack(side="top", fill="x", pady=(2, 0))
        self.reset_btn = ttk.Button(dir_frame, text="Reset Enc", command=self.on_reset_encoder)
        self.reset_btn.pack(side="top", fill="x", pady=(2, 0))

        # Live position label
        self.pos_var = tk.StringVar(value="Position: —")
        self.pos_label = ttk.Label(top, textvariable=self.pos_var)
        self.pos_label.pack(side="left", padx=(12, 0))

        self.pos_deg_var = tk.StringVar(value="—°")
        self.pos_ticks_var = tk.StringVar(value="ticks: —")

        self.title("Dynamixel Test")
        self.geometry("640x320")
        self.resizable(False, False)

        # --- Controls ---
        frm = ttk.Frame(self, padding=12)
        frm.pack(fill="both", expand=True)

        # Removed global angle controls and rotate buttons here

        # Mode-specific UI container (will be re-rendered on mode change)
        self.mode_area = ttk.Frame(frm)
        self.mode_area.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(8, 4))

        # Log box
        ttk.Label(frm, text="Log:").grid(row=2, column=0, columnspan=2, sticky="w", pady=(10, 0))
        self.log = tk.Text(frm, width=44, height=6, state="disabled")
        self.log.grid(row=3, column=0, columnspan=2, sticky="nsew")

        # Make grid stretch properly
        frm.columnconfigure(0, weight=0)
        frm.columnconfigure(1, weight=1)
        frm.rowconfigure(3, weight=1)

        # Removed key binding for Enter

        # Render initial UI for default mode and start position polling
        self._velocity_after_id = None
        self.render_mode_ui()
        self.after(100, self._update_position)
        self._update_dir_label()

    # ---- Helpers ----
    def get_degrees(self) -> float:
        try:
            return float(self.deg_var.get())
        except ValueError:
            self.append_log("Invalid degrees. Using 0.\n")
            return 0.0

    def append_log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg)
        self.log.see("end")
        self.log.configure(state="disabled")

    def disable_buttons(self, disabled: bool):
        state = "disabled" if disabled else "normal"
        for name in ("btn_rel_plus", "btn_rel_minus"):
            btn = getattr(self, name, None)
            if btn is None:
                continue
            try:
                # winfo_exists() returns 1 if the underlying Tk widget still exists
                if int(btn.winfo_exists()):
                    btn.configure(state=state)
                else:
                    setattr(self, name, None)
            except Exception:
                # If widget is stale/destroyed, drop the reference
                setattr(self, name, None)

    # ---- Mode handling and live position update ----
    def on_mode_change(self, event=None):
        mode = self.mode_var.get()
        # Map to your enum if desired
        try:
            activeMotor.setOpMode(getattr(OpModes, mode))
        except Exception:
            pass
        self.render_mode_ui()

    def on_motor_change(self, event=None):
        global activeMotor
        name = self.motor_var.get()
        m = motors.get(name)
        if m is not None:
            activeMotor = m
            # Keep OpMode in sync with current GUI selection
            try:
                activeMotor.setOpMode(getattr(OpModes, self.mode_var.get()))
            except Exception:
                pass
        # Refresh direction indicator whenever motor changes
        self._update_dir_label()

    def _update_dir_label(self):
        """Update UI text showing whether the active motor is inverted (reverse mode)."""
        global activeMotor
        try:
            dm = activeMotor.getDriveMode(verbose=False)
            inverted = bool(dm & 0x01)
            txt = "Dir: Inverted" if inverted else "Dir: Normal"
        except Exception:
            txt = "Dir: ?"
        self.dir_var.set(txt)

    def on_toggle_dir(self):
        """Toggle reverse mode on the active motor and refresh the label."""
        global activeMotor
        try:
            activeMotor.invert()
            self.append_log("Toggled motor direction.\n")
        except Exception as e:
            self.append_log(f"Direction toggle error: {e}\n")
        self._update_dir_label()

    def on_reset_encoder(self):
        """Reset the active motor's encoder to zero."""
        global activeMotor
        try:
            activeMotor.resetEncoder(verbose=True)
            self.append_log("Encoder reset to 0.\n")
        except Exception as e:
            self.append_log(f"Encoder reset error: {e}\n")

    def clear_mode_area(self):
        for w in self.mode_area.winfo_children():
            w.destroy()
        # Clear references to mode-specific buttons that may have been destroyed
        for name in ("btn_rel_plus", "btn_rel_minus"):
            if hasattr(self, name):
                setattr(self, name, None)

    def render_mode_ui(self):
        self.clear_mode_area()
        mode = self.mode_var.get()

        if mode == "VELOCITY":
            ttk.Label(self.mode_area, text="Velocity (raw units 0..2920)").grid(row=0, column=0, sticky="w")
            ttk.Label(self.mode_area, text="0").grid(row=1, column=0, sticky="w")
            ttk.Label(self.mode_area, text="2920").grid(row=1, column=2, sticky="e")

            self.vel_var = tk.DoubleVar(value=0.0)
            self.vel_val_lbl = ttk.Label(self.mode_area, text="Current: 0")
            self.vel_val_lbl.grid(row=0, column=1, sticky="e")

            self.vel_scale = ttk.Scale(self.mode_area, from_=0, to=2920,
                                       orient="horizontal",
                                       command=self.on_velocity_change,
                                       variable=self.vel_var)
            self.vel_scale.grid(row=1, column=1, sticky="ew", padx=8)
            self.mode_area.columnconfigure(1, weight=1)

        elif mode in ("POSITION", "EXTENDED_POSITION"):
            # Position control with Absolute/Relative selector
            ttk.Label(self.mode_area, text="Position control").grid(row=0, column=0, sticky="w", columnspan=4)
            ttk.Label(self.mode_area, text="Mode:").grid(row=1, column=0, sticky="w")
            self.pos_type_var = getattr(self, "pos_type_var", tk.StringVar(value="Absolute"))
            type_cb = ttk.Combobox(self.mode_area, width=12, state="readonly",
                                   values=["Absolute", "Relative"], textvariable=self.pos_type_var)
            type_cb.grid(row=1, column=1, sticky="w", padx=(6, 12))
            type_cb.bind("<<ComboboxSelected>>", lambda e: self.render_mode_ui())
            # Duplicate current position (deg) on the right
            ttk.Label(self.mode_area, text="Current position:").grid(row=1, column=2, sticky="e", padx=(12, 6))
            ttk.Label(self.mode_area, textvariable=self.pos_deg_var).grid(row=1, column=3, sticky="w")
            self.mode_area.columnconfigure(2, weight=0)
            self.mode_area.columnconfigure(3, weight=1)

            # Show raw tick position just below the angle readout
            ttk.Label(self.mode_area, text="Ticks:").grid(row=2, column=2, sticky="e", padx=(12, 6))
            ttk.Label(self.mode_area, textvariable=self.pos_ticks_var).grid(row=2, column=3, sticky="w")

            if self.pos_type_var.get() == "Absolute":
                ttk.Label(self.mode_area, text="Goal position (ticks):").grid(row=3, column=0, sticky="w")
                self.goal_ticks_var = getattr(self, "goal_ticks_var", tk.StringVar(value="0"))
                ttk.Entry(self.mode_area, width=12, textvariable=self.goal_ticks_var).grid(row=3, column=1, sticky="w", padx=(6, 12))
                ttk.Button(self.mode_area, text="Set Goal", command=self.on_set_goal).grid(row=3, column=2, sticky="w")
            else:
                ttk.Label(self.mode_area, text="Delta angle (deg):").grid(row=3, column=0, sticky="w")
                self.delta_var = getattr(self, "delta_var", tk.StringVar(value="30"))
                ttk.Entry(self.mode_area, width=10, textvariable=self.delta_var).grid(row=3, column=1, sticky="w", padx=(6, 12))
                self.btn_rel_plus = ttk.Button(self.mode_area, text="+ Rotate", command=lambda: self.on_rotate_relative(+1))
                self.btn_rel_minus = ttk.Button(self.mode_area, text="− Rotate", command=lambda: self.on_rotate_relative(-1))
                self.btn_rel_plus.grid(row=4, column=0, sticky="ew", pady=(6, 0), columnspan=2)
                self.btn_rel_minus.grid(row=4, column=2, sticky="ew", pady=(6, 0))

        elif mode == "PWM":
            ttk.Label(self.mode_area, text="PWM mode UI not implemented yet.").grid(row=0, column=0, sticky="w")

    def on_velocity_change(self, _evt=None):
        # Debounce rapid slider events; send after 100 ms
        if self._velocity_after_id is not None:
            self.after_cancel(self._velocity_after_id)
        val = float(self.vel_var.get())
        self.vel_val_lbl.configure(text=f"Current: {int(val)}")

        def send():
            try:
                set_velocity(val)
                self.append_log(f"Set velocity (raw): {int(val)}\n")
            except Exception as e:
                self.append_log(f"Velocity error: {e}\n")

        self._velocity_after_id = self.after(100, send)

    def on_set_goal(self):
        mode = self.mode_var.get()
        if mode not in ("POSITION", "EXTENDED_POSITION"):
            return
        if getattr(self, "pos_type_var", None) and self.pos_type_var.get() == "Absolute":
            try:
                ticks = int(self.goal_ticks_var.get())
            except (ValueError, AttributeError):
                self.append_log("Invalid goal position (ticks).\n")
                return
            self.append_log(f"Setting goal position to {ticks} ticks...\n")
            def send_ticks(t=ticks):
                try:
                    activeMotor.setGoalPosition(int(t), verbose=False)
                    self.after(0, lambda: self.append_log("Done.\n"))
                except Exception as e:
                    self.after(0, lambda: self.append_log(f"Error: {e}\n"))
            # Do NOT disable/enable buttons in Absolute mode (buttons may not exist)
            threading.Thread(target=send_ticks, daemon=True).start()
            return
        try:
            angle = float(self.goal_var.get())
        except (ValueError, AttributeError):
            self.append_log("Invalid goal angle.\n")
            return
        self.append_log(f"Setting goal angle to {angle}°...\n")
        threading.Thread(target=self._run_safe, args=(set_goal_angle, angle), daemon=True).start()

    def _update_position(self):
        ticks, pos = get_present_angle_safe()
        if ticks is None:
            self.pos_var.set("Position: N/A")
            self.pos_deg_var.set("N/A")
            self.pos_ticks_var.set("ticks: N/A")
        else:
            # Show 1 decimal place for degrees
            self.pos_var.set(f"Position: {pos:.1f}°")
            self.pos_deg_var.set(f"{pos:.1f}°")
            self.pos_ticks_var.set(f"{int(ticks)}")
        self.after(100, self._update_position)

    def on_rotate_relative(self, direction: int):
        try:
            degrees = float(self.delta_var.get())
        except (ValueError, AttributeError):
            self.append_log("Invalid delta angle.\n")
            return
        deg = direction * degrees
        self.append_log(f"Rotating by {deg:+.1f}°...\n")
        self.disable_buttons(True)
        threading.Thread(target=self._run_safe, args=(rotate_a_cw if deg >= 0 else rotate_b_ccw, abs(deg)), daemon=True).start()

    # Run a motor function safely off the main thread, then re-enable buttons
    def _run_safe(self, fn, degrees):
        try:
            fn(degrees)
            self.after(0, lambda: self.append_log("Done.\n"))
        except Exception as e:
            self.after(0, lambda: self.append_log(f"Error: {e}\n"))
        finally:
            self.after(0, lambda: self.disable_buttons(False))

if __name__ == "__main__":
    app = MotorGUI()
    app.mainloop()