import time
import dearpygui.dearpygui as dpg

def _make_callbacks(shared):
    def freq_cb(sender, app_data, user_data):
        shared['verlet_rebuild_freq'] = int(app_data)
    def skin_cb(sender, app_data, user_data):
        try:
            shared['verlet_skin'] = float(app_data)
        except Exception:
            pass
    def iters_cb(sender, app_data, user_data):
        shared['collision_iterations'] = int(app_data)
    def pause_cb():
        shared['toggle_pause'] = True
    def reset_cb():
        shared['reset_world'] = True
    def spawn_cb():
        shared['spawn_emitter'] = True
    def exit_cb():
        shared['__exit__'] = True
    def spring_constraint_length_cb(sender, app_data, user_data):
        try:
            shared['spring_length'] = float(app_data)
        except Exception:
            pass
    def spring_constraint_stiffness_cb(sender, app_data, user_data):
        try:
            shared['spring_stiffness'] = float(app_data)
        except Exception:
            pass
    def spring_constraint_damping_cb(sender, app_data, user_data):
        try:
            shared['spring_damping'] = float(app_data)
        except Exception:
            pass
    return freq_cb, skin_cb, iters_cb, pause_cb, reset_cb, spawn_cb, exit_cb, spring_constraint_length_cb, spring_constraint_stiffness_cb, spring_constraint_damping_cb

def run_gui(shared):
    """
    Run DearPyGui in its own process. Writes values into `shared` dict.
    """
    dpg.create_context()

    # grab callbacks
    freq_cb, skin_cb, iters_cb, pause_cb, reset_cb, spawn_cb, exit_cb, \
        spring_length_cb, spring_stiffness_cb, spring_damping_cb = _make_callbacks(shared)

    # Main controls window (give it a tag)
    with dpg.window(label="Physics Controls", tag="controls_window", width=380, height=320):
        dpg.add_text("Collision tuning")
        dpg.add_spacer()
        dpg.add_text("Verlet rebuild frequency (frames)")
        dpg.add_slider_int(label="Rebuild Freq", tag="freq_slider", default_value=int(shared.get('verlet_rebuild_freq', 5)),
                           min_value=1, max_value=20, callback=freq_cb)
        dpg.add_text("Verlet skin")
        dpg.add_slider_float(label="Skin", tag="skin_slider", default_value=float(shared.get('verlet_skin', 4.0)),
                             min_value=0.0, max_value=100.0, callback=skin_cb)
        dpg.add_text("Collision iterations")
        dpg.add_slider_int(label="Iterations", tag="iters_slider", default_value=int(shared.get('collision_iterations', 4)),
                           min_value=1, max_value=10, callback=iters_cb)
        dpg.add_text("Constraint iterations")
        dpg.add_slider_int(label="Iterations", tag="constraint_iters_slider", default_value=int(shared.get('constraint_iterations', 2)),
                           min_value=1, max_value=50, callback=lambda s, a, u: shared.update({'constraint_iterations': int(a)}))
        dpg.add_separator()
        dpg.add_button(label="Pause / Toggle", callback=lambda s,a,u: pause_cb())
        dpg.group(horizontal=True)
        dpg.add_button(label="Reset World", callback=lambda s,a,u: reset_cb())
        dpg.add_button(label="Spawn Emitter", callback=lambda s,a,u: spawn_cb())
        dpg.group(horizontal=True)
        dpg.add_button(label="Exit GUI", callback=lambda s,a,u: exit_cb())
        dpg.add_spacer()
        dpg.add_text("Selected Constraint:", tag="constraint_type_label")
        dpg.add_text("", tag="constraint_type_text")
        dpg.add_text("Status:", tag="status_label")
        dpg.add_text("", tag="status_text")
    
    # Spring controls window (create with a tag so we can hide/show it)
    with dpg.window(label="Spring Constraint Controls", tag="spring_window", pos=(0, 270), width=380, height=200):
        dpg.add_text("Spring Constraint Parameters")
        dpg.add_spacer()
        dpg.add_text("Rest Length")
        dpg.add_input_float(label="Length", tag="spring_length_input", default_value=float(shared.get('spring_length', 100.0)),
                            callback=lambda s,a,u: spring_length_cb(s,a,u))
        dpg.add_text("Stiffness")
        dpg.add_input_float(label="Stiffness", tag="spring_stiffness_input", default_value=float(shared.get('spring_stiffness', 15.0)),
                            callback=lambda s,a,u: spring_stiffness_cb(s,a,u))
        dpg.add_text("Damping (optional, 0 for none)")
        dpg.add_input_float(label="Damping", tag="spring_damping_input", default_value=float(shared.get('spring_damping', 0.0)),
                            callback=lambda s,a,u: spring_damping_cb(s,a,u))

    # start with spring window hidden until a constraint is selected
    dpg.hide_item("spring_window")

    dpg.create_viewport(title='Physics Controls', width=400, height=500)
    dpg.set_primary_window("controls_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    try:
        while not shared.get('__exit__', False) and dpg.is_dearpygui_running():
            # update status from shared
            try:
                status = f"freq={shared.get('verlet_rebuild_freq', 5)}, skin={float(shared.get('verlet_skin', 0.0)):.2f}, coll_iters={shared.get('collision_iterations', 4)}, const_iters={shared.get('constraint_iterations', 2)}"
            except Exception:
                status = "status error"
            dpg.set_value("status_text", status)

            # show constraint-specific controls only when a constraint is selected
            selected = shared.get('selected_constraint')  # expected to be None or dict with at least 'type'
            if selected:
                ctype = selected.get('type', 'Unknown')
                dpg.set_value("constraint_type_text", str(ctype))

                # helper to safely coerce values to float with fallback
                def safe_float(val, fallback):
                    try:
                        if val is None:
                            return float(fallback)
                        return float(val)
                    except Exception:
                        try:
                            return float(fallback)
                        except Exception:
                            return 0.0

                if ctype == "Spring":
                    # populate spring controls from the selected constraint (or shared fallbacks)
                    length = safe_float(selected.get('length'), shared.get('spring_length', 100.0))
                    stiffness = safe_float(selected.get('stiffness'), shared.get('spring_stiffness', 15.0))
                    damping = safe_float(selected.get('damping'), shared.get('spring_damping', 0.0))
                    # update existing widgets (created once above) and show the spring window
                    dpg.set_value("spring_length_input", length)
                    dpg.set_value("spring_stiffness_input", stiffness)
                    dpg.set_value("spring_damping_input", damping)
                    # reflect these in shared so main process can pick up edits
                    shared['spring_length'] = length
                    shared['spring_stiffness'] = stiffness
                    shared['spring_damping'] = damping
                    dpg.show_item("spring_window")
                else:
                    # not a spring -> hide spring controls
                    dpg.hide_item("spring_window")
            else:
                dpg.set_value("constraint_type_text", "None")
                dpg.hide_item("spring_window")

            dpg.render_dearpygui_frame()
            time.sleep(0.01)
    finally:
        dpg.destroy_context()
    
if __name__ == "__main__":
    from multiprocessing import Manager
    mgr = Manager()
    shared = mgr.dict()
    shared['verlet_rebuild_freq'] = 5
    shared['verlet_skin'] = 4.0
    shared['collision_iterations'] = 4
    run_gui(shared)