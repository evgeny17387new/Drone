import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time
from matplotlib.widgets import CheckButtons

port = "COM11"      # change to your Arduino port
baud = 115200      # must match Arduino sketch
ser = serial.Serial(port, baud, timeout=1)

# Dynamic storage for labels and their data
data = {}
xdata = []
lines = {}
visible = {}
fig, ax = plt.subplots()

# Set legend to fixed position in lower left
legend = ax.legend(loc='lower left')

# Maximize window on start (Windows)
try:
    mng = plt.get_current_fig_manager()
    mng.window.state('zoomed')
except Exception:
    pass

# Thread-safe buffer for latest serial line
latest_line = [None]
lock = threading.Lock()

SAMPLE_LIMIT = 10

def serial_reader():
    while True:
        line = ser.readline().decode().strip()
        # Debug print: just print the lines
        print(line)
        # Only save line for plotting if it does NOT contain LoopTimer
        if "looptimer:" not in line.lower():
            with lock:
                latest_line[0] = line

# Start serial reading thread
thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

check_buttons = None
# Move checkboxes to upper right corner to avoid y-axis overlap
rax = plt.axes([0.85, 0.75, 0.13, 0.2])  # Fixed position in left upper corner

# Callback for checkbox changes
def func(label):
    visible[label] = not visible[label]
    lines[label].set_visible(visible[label])
    plt.draw()

def update_checkboxes():
    global check_buttons
    # Instead of clearing and recreating, just update labels and states
    labels = list(visible.keys())
    states = [visible[l] for l in labels]
    if check_buttons is None:
        check_buttons = CheckButtons(rax, labels, states)
        check_buttons.on_clicked(func)
    else:
        # Update labels and states without recreating axes
        check_buttons.labels = labels
        for i, state in enumerate(states):
            check_buttons.set_active(i) if state else check_buttons.set_active(i)
    plt.draw()

def update(frame):
    global check_buttons
    with lock:
        line = latest_line[0]
    if not line:
        return list(lines.values())
    try:
        pairs = line.split(",")
        values = {}
        new_labels = False
        for pair in pairs:
            if ":" in pair:
                label, value = pair.split(":", 1)
                try:
                    values[label] = float(value)
                except ValueError:
                    continue
        if not values:
            return list(lines.values())
        if len(xdata) < SAMPLE_LIMIT:
            xdata.append(len(xdata))
        for label, val in values.items():
            if label not in data:
                data[label] = []
                lines[label] = ax.plot([], [], label=label)[0]
                visible[label] = True
                # Update legend only once, keep it fixed
                legend = ax.legend(loc='lower left')
                new_labels = True
            data[label].append(val)
            # Limit to last SAMPLE_LIMIT points for performance
            if len(data[label]) > SAMPLE_LIMIT:
                data[label] = data[label][-SAMPLE_LIMIT:]
        # Limit xdata as well
        for label in values:
            lines[label].set_data(xdata, data[label])
            lines[label].set_visible(visible[label])
        # Autoscale y-axis based only on visible values
        if xdata:
            visible_points = []
            for label in data:
                if visible.get(label, False):
                    visible_points.extend(data[label])
            if visible_points:
                ymin = min(visible_points)
                ymax = max(visible_points)
                ax.set_ylim(ymin, ymax)
        ax.relim()
        ax.autoscale_view()
        if new_labels:
            update_checkboxes()
    except Exception as e:
        print(f"Error parsing line: {e}")
    return list(lines.values())

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.show()
