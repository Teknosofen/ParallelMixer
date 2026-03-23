"""
Surface plot of interpolated valve flow data with measured test lines overlaid.
Axes: X = Control voltage (V), Y = Supply pressure (mbar), Z = Flow (slm)
"""

import io
import struct
import ctypes
import warnings
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

warnings.filterwarnings("ignore", message=".*tight_layout.*")
warnings.filterwarnings("ignore", message=".*Glyph.*")


def copy_figure_to_clipboard(fig):
    """Copy the current matplotlib figure to the Windows clipboard as a bitmap."""
    fig.canvas.draw()
    buf = fig.canvas.buffer_rgba()
    w, h = fig.canvas.get_width_height()
    arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)
    # BMP stores rows bottom-up, BGR order
    arr_bgr = arr[::-1, :, [2, 1, 0]]
    row_bytes = w * 3
    row_pad = (4 - row_bytes % 4) % 4
    row_size = row_bytes + row_pad
    img_size = row_size * h
    header = struct.pack(
        '<IiiHHIIiiII',
        40, w, h, 1, 24, 0, img_size, 0, 0, 0, 0,
    )
    padding = b'\x00' * row_pad
    rows = b''.join(arr_bgr[i].tobytes() + padding for i in range(h))
    dib = header + rows

    CF_DIB = 8
    GMEM_MOVEABLE = 0x0002
    kernel32 = ctypes.windll.kernel32
    user32 = ctypes.windll.user32

    # Set proper 64-bit return / argument types
    kernel32.GlobalAlloc.restype = ctypes.c_void_p
    kernel32.GlobalAlloc.argtypes = [ctypes.c_uint, ctypes.c_size_t]
    kernel32.GlobalLock.restype = ctypes.c_void_p
    kernel32.GlobalLock.argtypes = [ctypes.c_void_p]
    kernel32.GlobalUnlock.argtypes = [ctypes.c_void_p]
    user32.SetClipboardData.argtypes = [ctypes.c_uint, ctypes.c_void_p]

    user32.OpenClipboard(0)
    user32.EmptyClipboard()
    hmem = kernel32.GlobalAlloc(GMEM_MOVEABLE, len(dib))
    ptr = kernel32.GlobalLock(hmem)
    ctypes.memmove(ptr, dib, len(dib))
    kernel32.GlobalUnlock(hmem)
    user32.SetClipboardData(CF_DIB, hmem)
    user32.CloseClipboard()
    print("✓ Figure copied to clipboard — paste with Ctrl+V")


def on_key(event):
    if event.key in ('ctrl+shift+c', 'ctrl+C'):
        copy_figure_to_clipboard(event.canvas.figure)

# ── Interpolated table (V% \ Psupply) ──────────────────────────
interp_voltage = np.array([
    5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5,
    10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0,
])
interp_psupply = np.array([302, 451, 604])

# Flow(slm) – rows = voltage, columns = [302, 451, 604]
interp_flow = np.array([
    [0.01,   0.01,   0.01],
    [0.42,   0.17,   0.11],
    [3.06,   1.42,   0.91],
    [10.94,  8.73,   6.39],
    [19.86,  18.87,  17.29],
    [30.13,  31.04,  31.61],
    [42.32,  46.83,  48.91],
    [55.82,  62.05,  73.77],
    [74.05,  74.92,  91.36],
    [107.83, 111.69, 121.12],
    [111.79, 147.76, 160.04],
    [111.63, 157.79, 191.88],
    [112.03, 159.17, 197.01],
    [111.91, 158.96, 195.44],
    [112.16, 159.43, 194.00],
    [112.14, 158.53, 194.14],
    [112.04, 158.55, 193.97],
    [112.42, 158.84, 194.10],
    [112.39, 159.47, 193.89],
])

# Build mesh for surface  (finer interpolation for smooth surface)
from scipy.interpolate import RectBivariateSpline

# RectBivariateSpline wants strictly increasing x and y
spl = RectBivariateSpline(interp_voltage, interp_psupply, interp_flow, kx=3, ky=2)

V_fine = np.linspace(interp_voltage.min(), interp_voltage.max(), 120)
P_fine = np.linspace(interp_psupply.min(), interp_psupply.max(), 80)
V_mesh, P_mesh = np.meshgrid(V_fine, P_fine, indexing='ij')
Z_mesh = spl(V_fine, P_fine)

# ── Measured data – 3 bar test ──────────────────────────────────
data_3bar = np.array([
    [0.000,  0.002,  301.9], [0.200,  0.001,  301.9], [0.400,  0.001,  301.9],
    [0.600,  0.001,  301.9], [0.800,  0.002,  302.2], [1.000,  0.001,  302.0],
    [1.200,  0.001,  301.9], [1.400,  0.000,  301.8], [1.600,  0.001,  301.8],
    [1.800,  0.001,  301.7], [2.000,  0.001,  301.9], [2.200,  0.001,  301.8],
    [2.400,  0.000,  301.9], [2.600,  0.001,  301.4], [2.800, -0.000,  302.0],
    [3.000,  0.002,  301.7], [3.200,  0.001,  301.9], [3.400,  0.001,  301.6],
    [3.600,  0.001,  301.9], [3.800,  0.001,  301.7], [4.000,  0.002,  301.9],
    [4.200,  0.001,  301.7], [4.400,  0.002,  301.9], [4.600,  0.002,  301.7],
    [4.800,  0.003,  301.8], [5.000,  0.014,  301.9], [5.200,  0.072,  301.7],
    [5.400,  0.233,  301.1], [5.600,  0.614,  299.5], [5.800,  1.377,  296.6],
    [6.000,  3.058,  291.3], [6.200,  6.332,  286.8], [6.400,  9.269,  283.1],
    [6.600, 12.605,  281.3], [6.800, 16.084,  279.2], [7.000, 19.855,  278.2],
    [7.200, 23.786,  276.4], [7.400, 27.953,  273.6], [7.600, 32.306,  270.7],
    [7.800, 37.248,  267.3], [8.000, 42.323,  264.0], [8.200, 47.431,  262.6],
    [8.400, 52.923,  257.8], [8.600, 58.719,  252.2], [8.800, 65.838,  245.8],
    [9.000, 74.052,  237.9], [9.200, 89.534,  223.9], [9.400,105.091,  208.3],
    [9.600,110.561,  189.6], [9.800,111.601,  185.7],[10.000,111.792,  185.9],
    [10.200,111.701, 186.3],[10.400,111.778,  186.5],[10.600,111.477,  186.2],
    [10.800,111.835, 186.1],[11.000,112.027,  186.2],[11.200,112.009,  186.4],
    [11.400,112.089, 186.5],[11.600,111.739,  186.6],[11.800,111.881,  186.5],
    [12.000,112.164, 186.8],[12.200,112.569,  186.6],[12.400,112.035,  186.8],
    [12.600,112.239, 186.6],[12.800,111.942,  186.4],[13.000,112.036,  186.8],
    [13.200,112.322, 186.5],[13.400,112.328,  186.7],[13.600,112.513,  186.5],
    [13.800,112.089, 186.8],[14.000,112.387,  186.8],
])

# ── Measured data – 4.5 bar test ────────────────────────────────
data_45bar = np.array([
    [0.000,  0.000,  450.7], [0.250,  0.002,  450.8], [0.500,  0.002,  450.8],
    [0.750,  0.002,  450.6], [1.000,  0.001,  450.7], [1.250,  0.002,  450.6],
    [1.500,  0.002,  450.6], [1.750,  0.001,  450.6], [2.000, -0.000,  450.6],
    [2.250,  0.001,  450.6], [2.500,  0.001,  450.5], [2.750,  0.001,  450.5],
    [3.000,  0.001,  450.7], [3.250,  0.002,  450.7], [3.500,  0.001,  450.5],
    [3.750,  0.000,  450.7], [4.000,  0.001,  450.7], [4.250,  0.001,  450.3],
    [4.500,  0.001,  450.5], [4.750,  0.002,  450.3], [5.000,  0.010,  450.5],
    [5.250,  0.045,  450.3], [5.500,  0.168,  450.1], [5.750,  0.538,  448.8],
    [6.000,  1.424,  445.2], [6.250,  3.853,  439.9], [6.500,  8.727,  434.4],
    [6.750, 13.296,  430.9], [7.000, 18.870,  427.6], [7.250, 24.679,  424.4],
    [7.500, 31.039,  421.6], [7.750, 38.428,  420.4], [8.000, 46.833,  416.4],
    [8.250, 57.210,  409.6], [8.500, 62.053,  406.0], [8.750, 68.056,  402.3],
    [9.000, 74.922,  397.8], [9.250, 99.271,  386.0], [9.500,111.693,  367.9],
    [9.750,134.480,  344.3],[10.000,147.757,  318.3],[10.250,154.216,  305.2],
    [10.500,157.791, 299.2],[10.750,158.787,  297.6],[11.000,159.171,  297.7],
    [11.250,159.140, 298.0],[11.500,158.963,  298.1],[11.750,158.663,  297.7],
    [12.000,159.431, 298.1],[12.250,158.372,  298.2],[12.500,158.527,  298.4],
    [12.750,158.576, 298.6],[13.000,158.546,  298.5],[13.250,158.844,  298.5],
    [13.500,158.839, 298.6],[13.750,159.389,  298.5],[14.000,159.470,  298.4],
])

# ── Measured data – 6 bar test ──────────────────────────────────
data_6bar = np.array([
    [0.000,  0.001,  603.7], [0.200,  0.003,  603.7], [0.400,  0.001,  603.7],
    [0.600,  0.001,  603.7], [0.800,  0.000,  603.7], [1.000,  0.002,  603.7],
    [1.200,  0.001,  603.9], [1.400,  0.001,  603.6], [1.600,  0.002,  603.7],
    [1.800,  0.001,  603.6], [2.000,  0.001,  603.5], [2.200,  0.001,  603.7],
    [2.400,  0.002,  603.7], [2.600,  0.002,  603.2], [2.800,  0.001,  603.4],
    [3.000,  0.002,  603.7], [3.200,  0.001,  603.6], [3.400,  0.002,  603.7],
    [3.600,  0.001,  603.6], [3.800,  0.002,  603.6], [4.000,  0.001,  603.6],
    [4.200,  0.001,  603.4], [4.400,  0.002,  603.6], [4.600,  0.003,  603.5],
    [4.800,  0.004,  603.5], [5.000,  0.006,  603.3], [5.200,  0.014,  603.4],
    [5.400,  0.054,  603.4], [5.600,  0.173,  603.0], [5.800,  0.437,  602.3],
    [6.000,  0.905,  600.3], [6.200,  1.865,  596.6], [6.400,  4.083,  592.2],
    [6.600,  8.700,  589.2], [6.800, 12.885,  587.7], [7.000, 17.292,  585.1],
    [7.200, 22.483,  582.5], [7.400, 28.820,  580.1], [7.600, 34.395,  578.6],
    [7.800, 41.690,  576.9], [8.000, 48.906,  573.4], [8.200, 59.441,  568.8],
    [8.400, 68.540,  562.9], [8.600, 78.993,  559.5], [8.800, 84.808,  555.2],
    [9.000, 91.355,  551.4], [9.200,103.175,  545.9], [9.400,115.088,  535.7],
    [9.600,127.143,  523.0], [9.800,142.866,  509.3],[10.000,160.038,  489.5],
    [10.200,177.273, 467.0],[10.400,188.874,  446.1],[10.600,194.883,  424.7],
    [10.800,196.560, 409.0],[11.000,197.012,  393.8],[11.200,195.498,  383.9],
    [11.400,195.512, 380.8],[11.600,195.363,  379.3],[11.800,194.407,  378.3],
    [12.000,194.002, 377.5],[12.200,194.481,  377.2],[12.400,194.269,  376.8],
    [12.600,194.009, 376.6],[12.800,193.594,  376.3],[13.000,193.968,  376.1],
    [13.200,194.266, 376.1],[13.400,194.986,  376.1],[13.600,193.216,  375.6],
    [13.800,193.795, 375.8],[14.000,193.885,  375.8],
])

# ── Plotting ────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 9))
ax = fig.add_subplot(111, projection='3d')

# Surface (interpolated)
surf = ax.plot_surface(
    V_mesh, P_mesh, Z_mesh,
    cmap=cm.viridis, alpha=0.55, edgecolor='none',
)

# Measured lines
for data, label, color in [
    (data_3bar,  '~3 bar (302 mbar)',  'red'),
    (data_45bar, '~4.5 bar (451 mbar)', 'blue'),
    (data_6bar,  '~6 bar (604 mbar)',  'green'),
]:
    v, flow, ps = data[:, 0], data[:, 1], data[:, 2]
    ax.plot(v, ps, flow, color=color, linewidth=2.2, label=label, zorder=5)

ax.set_xlabel('Control voltage (V)', fontsize=11, labelpad=10)
ax.set_ylabel('Supply pressure (mbar)', fontsize=11, labelpad=10)
ax.set_zlabel('Flow (slm)', fontsize=11, labelpad=10)
ax.set_title('Valve Flow vs Control Voltage & Supply Pressure', fontsize=13, pad=18)

ax.legend(loc='upper left', fontsize=9)
fig.colorbar(surf, ax=ax, shrink=0.45, aspect=12, label='Flow (slm)')

ax.view_init(elev=28, azim=-55)

# Keyboard shortcut: Ctrl+Shift+C copies to clipboard
fig.canvas.mpl_connect('key_press_event', on_key)

# Add a clickable "Copy" button
from matplotlib.widgets import Button
ax_btn = fig.add_axes([0.01, 0.01, 0.12, 0.045])  # [left, bottom, width, height]
btn = Button(ax_btn, 'Copy', color='lightgoldenrodyellow', hovercolor='khaki')
btn.on_clicked(lambda _: copy_figure_to_clipboard(fig))

fig.text(0.14, 0.02, 'or press Ctrl+Shift+C',
         fontsize=7, color='grey', style='italic')

fig.subplots_adjust(left=0.05, right=0.92, bottom=0.08, top=0.95)
plt.show()
