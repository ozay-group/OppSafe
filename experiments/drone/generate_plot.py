import pandas as pd
import matplotlib.pyplot as plt
from scipy.io import loadmat
from matplotlib.patches import Polygon

import numpy as np
import matplotlib as mpl
import matplotlib.cm as cm
from matplotlib.collections import LineCollection

import numpy as np
import matplotlib as mpl
import matplotlib.cm as cm
from matplotlib.colors import LinearSegmentedColormap 
import pandas as pd
from matplotlib.patches import Polygon
from scipy.io import loadmat
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42

bad_start_color = '#FF0201'
bad_end_color = '#FFB400'
good_start_color = '#5800FF'
good_end_color = '#01C0FF'
hazard_color = '#FFD3C2'

# Create linear color maps
good_results_cmap = LinearSegmentedColormap.from_list('good_results', [good_start_color, good_end_color], N=256)
bad_results_cmap = LinearSegmentedColormap.from_list('bad_results', [bad_start_color, bad_end_color], N=256)


def plot_trajectory_two_colormaps(data1, data2, landmarks, poly_vs, file_name):
    fig, ax = plt.subplots(dpi=400)
    ax.set_position([0.1, 0.1, 0.775, 0.8])

    # Define the entire area vertices
    x_limit, y_limit = 3, 3  # These values should be large enough to cover the entire plotting area
    entire_area = [(-x_limit, -y_limit), (-x_limit, y_limit), (x_limit, y_limit), (x_limit, -y_limit)]

    def exp_proc(x):
        return np.exp(x) ** 2

    # If poly_vs is provided, cut out the inside area from the entire area
    if poly_vs is not None:
        polygon = Polygon(entire_area, closed=True, facecolor=hazard_color, edgecolor='none', alpha=1, zorder=0,
                          label='hazard area')
        ax.add_patch(polygon)

        inside_polygon = Polygon(poly_vs, closed=True, facecolor='white', edgecolor=hazard_color, zorder=0)
        ax.add_patch(inside_polygon)

    # Plotting for Data 1 (using viridis_r colormap)
    drone_x1, drone_y1 = data1['state_estimate_x'], data1['state_estimate_y']
    alphas_min1 = np.minimum(data1['alpha_x'], data1['alpha_y']).replace(-np.inf, -1)
    alphas_min_transformed1 = exp_proc(alphas_min1)
    norm1 = mpl.colors.Normalize(vmin=exp_proc(-1), vmax=exp_proc(1))
    cmap1 = bad_results_cmap
    for i in range(1, len(drone_x1)):
        color = cmap1(norm1(alphas_min_transformed1.iloc[i - 1]))
        ax.plot(drone_x1.iloc[i - 1:i + 1], drone_y1.iloc[i - 1:i + 1], color=color, linewidth=2,
                solid_capstyle='round')

    # Plotting for Data 2 (using RdYlGn_r colormap)
    drone_x2, drone_y2 = data2['state_estimate_x'], data2['state_estimate_y']
    alphas_min2 = np.minimum(data2['alpha_x'], data2['alpha_y']).replace(-np.inf, -1)
    alphas_min_transformed2 = exp_proc(alphas_min2)
    norm2 = mpl.colors.Normalize(vmin=exp_proc(-1), vmax=exp_proc(1))
    cmap2 = good_results_cmap 
    for i in range(1, len(drone_x2)):
        color = cmap2(norm2(alphas_min_transformed2.iloc[i - 1]))
        ax.plot(drone_x2.iloc[i - 1:i + 1], drone_y2.iloc[i - 1:i + 1], color=color, linewidth=2,
                solid_capstyle='round')

    landmarks_x, landmarks_y = zip(*landmarks)
    ax.scatter(landmarks_x, landmarks_y, color='black', marker='x', s=15, label='Landmarks', zorder=10)

    ax.set_aspect('equal')
    ax.set_xlim(-1.9, 1.9)
    ax.set_ylim(-1.5, 1.5)

    # Create colorbars

    cax1 = fig.add_axes([0.9, 0.52, 0.025, 0.38])  # [left, bottom, width, height]
    cax2 = fig.add_axes([0.9, 0.1, 0.025, 0.38])

    cbar1 = fig.colorbar(cm.ScalarMappable(norm=norm1, cmap=cmap1), cax=cax1, ax=ax, orientation='vertical', pad=0.01,
                         shrink=0.8)
    cbar2 = fig.colorbar(cm.ScalarMappable(norm=norm2, cmap=cmap2), cax=cax2, ax=ax, orientation='vertical', pad=0.01,
                         shrink=0.8)

    # cbar1 = fig.colorbar(cm.ScalarMappable(norm=norm1, cmap=cmap1), ax=ax, orientation='vertical', pad=0.01)
    cbar1.set_label('robust', labelpad=-3, size=12)
    cbar1.set_ticks([exp_proc(0), exp_proc(1), exp_proc(-1)])
    cbar1.set_ticklabels(['0', '1', '-inf'])
    cbar1.ax.tick_params(labelsize=8)

    # cbar2 = fig.colorbar(cm.ScalarMappable(norm=norm2, cmap=cmap2), ax=ax, orientation='vertical', pad=0.15)
    cbar2.set_label('opportunistic', labelpad=-3, size=12)
    cbar2.set_ticks([exp_proc(0), exp_proc(1), exp_proc(-1)])
    cbar2.set_ticklabels(['0', '1', '-inf'])
    cbar2.ax.tick_params(labelsize=8)

    # plt.tight_layout()

    ax.set_xlabel("X Position", fontsize=12)
    ax.set_ylabel("Y Position", fontsize=12, labelpad=-3)

    ax.legend(fontsize="12")
    plt.savefig(file_name, format='eps')


# Example usage
if __name__ == '__main__':
    data_ = loadmat('data/poly_vs.mat')
    poly_vs = data_.get('V', None)
    lms = [[0, 0.25],
           [0.5, 1],
           [1.2, 1],
           [1.2, 0.5],
           [1, 0.5],
           [1, -0.5],
           [1.2, -0.5],
           [1.2, -1],
           [0.3, -1],
           [0.3, -0.5],
           [0.5, -0.5],
           [0.5, 0.25],
           [0, -0.5],
           [-0.5, 0.25],
           [-0.5, -0.5],
           [-0.3, -0.5],
           [-0.3, -1],
           [-1.2, -1],
           [-1.2, -0.5],
           [-1, -0.5],
           [-1, 0.5],
           [-1.2, 0.5],
           [-1.2, 1],
           [-0.5, 1],
           [0, 0.25]]
    # the trajectories for experiments without measurement noise
    data1 = pd.read_csv('data/v_flight_data_robust_d.csv')
    data2 = pd.read_csv('data/v_flight_data_opp_d.csv')
    plot_trajectory_two_colormaps(data1, data2, lms, poly_vs, 'flight_plot_t_M.eps')
    # the trajectories for experiments with measurement noise
    data1 = pd.read_csv('data/v_flight_data_robust_no_d.csv')
    data2 = pd.read_csv('data/v_flight_data_opp_no_d.csv')
    plot_trajectory_two_colormaps(data1, data2, lms, poly_vs, 'flight_plot_t_M_d.eps')

