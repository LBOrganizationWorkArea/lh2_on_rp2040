import numpy as np
import seaborn as sns

# Fix to avoid Type 3 fonts on the figures
# http://phyletica.org/matplotlib-fonts/
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

def plot_distance_histograms(x_dist, y_dist, z_dist):
    """
    Plot a histogram of the distance between grid-points in the X, Y and Z direction.
    Also prints the mean and std deviation of this distances.
    """
    # Print mean and std deviation of the distance in all 3 x_axis
    print(f"X mean = {x_dist.mean()}")
    print(f"X std = {x_dist.std()}")
    print(f"Y mean = {y_dist.mean()}")
    print(f"Y std = {y_dist.std()}")
    print(f"Z mean = {z_dist.mean()}")
    print(f"Z std = {z_dist.std()}")
    
    # Plot the results
    fig = plt.figure(layout="constrained")
    gs = GridSpec(2, 6, figure = fig)
    # Define individual subplots
    hist_x_ax    = fig.add_subplot(gs[0:2, 0:2])
    hist_y_ax    = fig.add_subplot(gs[0:2, 2:4])
    hist_z_ax    = fig.add_subplot(gs[0:2, 4:6])
    axs = (hist_x_ax, hist_y_ax, hist_z_ax)

    # X histogram
    n, bins, patches = hist_x_ax.hist(x_dist, 50, density=False)
    hist_x_ax.axvline(x=x_dist.mean(), color='red', label="Mean")
    # Y histogram
    n, bins, patches = hist_y_ax.hist(y_dist, 50, density=False)
    hist_y_ax.axvline(x=y_dist.mean(), color='red', label="Mean")
    # Z histogram
    n, bins, patches = hist_z_ax.hist(z_dist, 50, density=False)
    hist_z_ax.axvline(x=z_dist.mean(), color='red', label="Mean")

    # Add labels and grids
    for ax in axs:
        ax.grid()
        ax.legend()
    # 
    hist_x_ax.set_xlabel('Distance X axis [mm]')
    hist_y_ax.set_xlabel('Distance Y axis [mm]')
    hist_z_ax.set_xlabel('Distance Z axis [mm]')
    hist_x_ax.set_ylabel('Measurements')
    fig.suptitle('Distance Between Grid points\n(Should be 40mm)')

    plt.show()

def plot_reconstructed_3D_scene(point3D, t_star, R_star, df=None):
    """
    Plot a 3D scene with the traingulated points previously calculated
    ---
    input:
    point3D - array [3,N] - triangulated points of the positions of the LH2 receveier
    t_star  - array [3,1] - Translation vector between the first and the second lighthouse basestation
    R_star  - array [3,3] - Rotation matrix between the first and the second lighthouse basestation
    df      - dataframe   - dataframe holding the real positions of the gridpoints
    """
    ## Plot the two coordinate systems
    #  x is blue, y is red, z is green

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_proj_type('ortho')
    # First lighthouse:
    ax.quiver(0,0,0,0.1,0,0,color='xkcd:blue',lw=3)
    ax.quiver(0,0,0,0,0,-0.1,color='xkcd:red',lw=3)
    ax.quiver(0,0,0,0,0.1,0,color='xkcd:green',lw=3)

    # Second lighthouse:
    t_star_rotated = np.array([t_star.item(0), t_star.item(1), t_star.item(2)])
    print(R_star)
    print(t_star_rotated)
    x_axis = np.array([0.1,0,0])@np.linalg.inv(R_star)
    y_axis = np.array([0,0.1,0])@np.linalg.inv(R_star)
    z_axis = np.array([0,0,0.1])@np.linalg.inv(R_star)
    ax.quiver(t_star_rotated[0],t_star_rotated[2],-t_star_rotated[1],x_axis[0],x_axis[2],-x_axis[1], color='xkcd:blue',lw=3)
    ax.quiver(t_star_rotated[0],t_star_rotated[2],-t_star_rotated[1],y_axis[0],y_axis[2],-y_axis[1],color='xkcd:red',lw=3)
    ax.quiver(t_star_rotated[0],t_star_rotated[2],-t_star_rotated[1],z_axis[0],z_axis[2],-z_axis[1],color='xkcd:green',lw=3)
    ax.scatter(point3D[:,0],point3D[:,2],-point3D[:,1], alpha=0.1)

    # Plot the real 
    ax.text(-0.18,-0.1,0,s='LHA')
    ax.text(t_star_rotated[0], t_star_rotated[2], -t_star_rotated[1],s='LHB')

    ax.axis('equal')
    ax.set_title('Corrected - Elevation Angle')
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')


    plt.show()

def plot_projected_LH_views(pts_a, pts_b):
    """
    Plot the projected views from each of the lighthouse
    """
    fig = plt.figure(layout="constrained")
    gs = GridSpec(6, 3, figure = fig)
    lh1_ax    = fig.add_subplot(gs[0:3, 0:3])
    lh2_ax = fig.add_subplot(gs[3:6, 0:3])
    axs = (lh1_ax, lh2_ax)

    # 2D plots - LH2 perspective
    lh1_ax.scatter(pts_a[:,0], pts_a[:,1], edgecolor='blue', facecolor='blue', alpha=0.5, lw=1, label="LH1")
    lh2_ax.scatter(pts_b[:,0], pts_b[:,1], edgecolor='blue', facecolor='blue', alpha=0.5, lw=1, label="LH2")
    # Plot one synchronized point to check for a delay.

    # Add labels and grids
    for ax in axs:
        ax.grid()
        ax.legend()
        ax.axis('equal')
        ax.set_xlabel('U [px]')
        ax.set_ylabel('V [px]')
        ax.invert_yaxis()

    plt.show()

def plot_transformed_3D_data(df):
    """
    Plot the difference between the ground truth and the reconstructed data.
    This will make it easy to plot the error. 
    """
    # Create a figure to plot
    fig2 = plt.figure(layout="constrained", figsize=(5,4))
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.set_proj_type('ortho')

    # Plot the ground truth points
    real_points = np.unique(df[['real_x_mm','real_y_mm','real_z_mm']].to_numpy(), axis=0)  # Plot just a single point per grid position to save on computational power.
    ax2.scatter(real_points[:,0], real_points[:,1], real_points[:,2], alpha=0.5 ,color='xkcd:green', label="ground truth")
    ax2.scatter([], [], color="xkcd:blue", label="lighthouse")
    # Plot real dataset points
    points = np.zeros_like(real_points,dtype=float)
    for idx in range(real_points.shape[0]):
        x,y,z = real_points[idx]
        points[idx] = df.loc[(df['real_x_mm'] == x)  & (df['real_y_mm'] == y) & (df['real_z_mm'] == z), ['Rt_x','Rt_y','Rt_z']].mean().values

    # points = df[['Rt_x','Rt_y','Rt_z']].to_numpy()
    ax2.scatter(points[:,0], points[:,1], points[:,2], alpha=0.8, color='xkcd:blue')
    ax2.axis('equal')
    ax2.legend()

    # ax2.set_title('Tangent projection')
    ax2.set_xlabel('X [mm]')
    ax2.set_ylabel('Y [mm]')
    ax2.set_zlabel('Z [mm]')


    ax2.view_init(11,-81, 0)
    ax2.set_xlim3d((-16.9, 257.5))
    ax2.set_ylim3d((-78.4, 196))
    ax2.set_zlim3d((-25, 170))

    plt.savefig('Result-E-2lh_3d-solvedscene.pdf')

    plt.show()

    # ax.scatter(p000[:,0],p000[:,2],-p000[:,1], color='green')

def plot_error_histogram(df):
    """ 
    Calculate and plot a histogram  of the error of the reconstructed points, vs. 
    the ground truth.
    """
    # Extract needed data from the main dataframe
    points = df[['Rt_x','Rt_y','Rt_z']].to_numpy()
    ground_truth = df[['real_x_mm','real_y_mm','real_z_mm']].to_numpy()

    # Calculate distance between points and their ground truth
    errors =  np.linalg.norm(ground_truth - points, axis=1)
    # print the mean and standard deviation
    print(f"Mean Absolute Error = {errors.mean()} mm")
    print(f"Root Mean Square Error = {np.sqrt((errors**2).mean())} mm")
    print(f"Error Standard Deviation = {errors.std()} mm")

    # prepare the plot
    fig = plt.figure(layout="constrained", figsize=(5,4))
    gs = GridSpec(3, 3, figure = fig)
    hist_ax    = fig.add_subplot(gs[0:3, 0:3])
    axs = (hist_ax,)


    sns.histplot(data=errors,  bins=50, ax=hist_ax, linewidth=0, color="xkcd:baby blue")
    hist_ax.set_xlim((0, 10))
    ax2 = hist_ax.twinx()
    sns.kdeplot(data=errors, ax=ax2, label="density", color="xkcd:black", linewidth=1, linestyle='--')

    hist_ax.axvline(x=errors.mean(), color='xkcd:red', label="Mean")
    # Trick to get the legend  unified between the TwinX plots
    hist_ax.plot([], [], color="xkcd:black", linestyle='--', label = 'density')

    xticks_locs = np.linspace(0, 10, 5)  # 5 x-ticks from 0 to 10
    hist_ax.set_xticks(xticks_locs)


    # Plot the error histogram
    # n, bins, patches = hist_ax.hist(errors, 50, density=False)
    # hist_ax.axvline(x=errors.mean(), color='red', label="Mean")

    for ax in axs:
        ax.grid()
        ax.legend()
    
    hist_ax.set_xlabel('Distance Error [mm]')
    hist_ax.set_ylabel('Measurements')

    plt.savefig('Result-F-2lh_3d-histogram.pdf')


    plt.show()

    return

def plot_acc_vs_npoints(df_plot):
    
    # Find out how many unique N_Points are available in this experiment
    unique_n_points = np.unique(df_plot['n_points'].to_numpy().astype(int), axis=0)

    # Go through all the available N_point experiments
    mae_std = np.empty((1+unique_n_points.max()-8,3))
    for i in unique_n_points:
        # Get the mean of the MAE, and the STD of the MAE
        mae = df_plot.loc[(df_plot['n_points'] == i), 'MAE'].values.mean(axis=0)    
        std = df_plot.loc[(df_plot['n_points'] == i), 'MAE'].values.std(axis=0)
        # Add it to our empty array for plotting later
        mae_std[int(i)-8] = np.array([i, mae, std])    

    # prepare the plot
    fig = plt.figure(layout="constrained", figsize=(5,4))
    gs = GridSpec(3, 3, figure = fig)
    error_ax    = fig.add_subplot(gs[0:3, 0:3])
    axs = (error_ax,)

    # Plot Y = MAE, X = N_points
    error_ax.plot(mae_std[:,0], mae_std[:,1], 'xkcd:blue')
    # Add and area with 2 std deviation
    error_ax.fill_between(mae_std[:,0], np.clip(mae_std[:,1] - 1*mae_std[:,2], 0.0, 1e10), mae_std[:,1] + 1*mae_std[:,2], alpha=0.2, edgecolor='xkcd:indigo', facecolor='lightblue', linestyle='dashed', antialiased=True)

    for ax in axs:
        ax.grid()
        # ax.legend()
    
    error_ax.set_xlabel('Number of points')
    error_ax.set_ylabel('Mean Average Error [mm]')

    error_ax.set_xlim((8, 100))
    error_ax.set_ylim((0, 50))

    plt.savefig('Result-G-2lh_3d-pufpr.pdf')

    print(mae_std)
    plt.show()

def plot_acc_vs_mad(df_plot):
    
    # Find out how many unique N_Points are available in this experiment
    unique_MAD = np.unique(df_plot['MAD'].to_numpy(), axis=0)

    # prepare the plot
    fig = plt.figure(layout="constrained", figsize=(5,4))
    gs = GridSpec(3, 3, figure = fig)
    error_ax    = fig.add_subplot(gs[0:3, 0:3])
    axs = (error_ax,)

    # # Plot Y = MAE, X = MAD
    for i in unique_MAD:
        mae = df_plot.loc[(df_plot['MAD'] == i), 'MAE'].values
        # error_ax.scatter([i]*mae.shape[0], mae, color='xkcd:blue', alpha=0.3)
        error_ax.scatter([i], mae.min(), color='xkcd:red', alpha=1)
    # error_ax.scatter([i]*mae.shape[0], mae, color='xkcd:blue', alpha=0.3, label='Reconstruction Error')
    error_ax.scatter([i], mae.min(), color='xkcd:red', alpha=1, label='Best Reconstruction')

    for ax in axs:
        ax.grid()
        ax.legend()
    
    error_ax.set_xlim((30, 160))
    error_ax.set_ylim((0, 120))
    error_ax.set_xlabel('Median Average Deviation [mm]')
    error_ax.set_ylabel('Medium Average Reconstruction Error [mm]')

    print(df_plot)

    plt.savefig('Result-H-2lh_3d-mad_v_mae.pdf')
    plt.show()
    print(5)
