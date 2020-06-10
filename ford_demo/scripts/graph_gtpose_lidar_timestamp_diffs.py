import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

usage_msg = 'Usage: {} <gtpose_lidar_info_file.csv> [<save_dir>]'.format(sys.argv[0])
if len(sys.argv) <= 1 or len(sys.argv) >= 4 or sys.argv[1] == '-h':
    print(usage_msg)
    sys.exit(1)

savedir = '.' if len(sys.argv) == 2 else sys.argv[2]
infofile = sys.argv[1]

if savedir != '.' and not os.path.exists(savedir):
    os.makedirs(savedir)

# Open the stats file.
outfile = open(os.path.join(savedir, 'stats.txt'), 'w')

print('Loading the information file: ' + infofile)
headers = [ 'mid','px','py','pz','ox','oy','oz','ow','gttime','redtime','greentime','bluetime','yellowtime' ]
dtypes = [ str ] + [ np.float64 ] * 12
infodf = pd.read_csv(infofile, dtype=dict(zip(headers, dtypes)))
print('Loaded {} entries.\n'.format(len(infodf.index)))
outfile.write('Loaded {} entries.\n\n'.format(len(infodf.index)))
gttimes = infodf['gttime'].to_numpy()

# Compute the distance of this log.
prev_pt = infodf[['px','py','pz']].iloc[:-1]
next_pt = infodf[['px','py','pz']].iloc[1:]

distances = np.sqrt(np.sum(((next_pt.to_numpy() - prev_pt.to_numpy()) ** 2), axis=1)).flatten()
outfile.write('Total distance: {:.3f} meters.\n'.format(np.sum(distances)))
outfile.write('Distance stats [meters]:\n  Min: {:10.3f}'.format(distances.min()))
outfile.write('\n  Mean: {:9.3f} +/- {:.3f}\n  Median: {:7.3f}'.format(distances.mean(),
    np.std(distances), np.median(distances)))
outfile.write('\n  Max: {:10.3f}\n\n'.format(distances.max()))

# Graph information related to the position and distances.
sc = plt.scatter(infodf['px'].to_numpy(), infodf['py'].to_numpy(),
    c=gttimes - gttimes[0], s=0.25)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Vehicle Trajectory')
plt.colorbar(sc)
plt.savefig(os.path.join(savedir, 'vehicle_trajectory.png'))
plt.close()

sc = plt.scatter(gttimes[1:] - gttimes[0], distances, c=distances, s=0.25)
plt.colorbar(sc)
plt.xlabel('GT Pose time [sec]')
plt.ylabel('Distance from previous GT [m]')
plt.title('Distances between GT Poses')
plt.savefig(os.path.join(savedir, 'dist_between_poses.png'))
plt.close()

# Compute the time states for this log.
outfile.write('Total ground truth duration: {:.3f} secs.\n'.format(
    infodf.loc[len(infodf.index)-1,'gttime'] - infodf.loc[0,'gttime']))

scan_time_hdrs = headers[-4:]
scan_time_diffs = np.abs(infodf['gttime'].to_numpy().reshape((-1,1)) \
    - infodf[scan_time_hdrs].to_numpy())
timediffs = np.amax(scan_time_diffs, axis=1)
outfile.write('Ground truth Pose - LiDAR scan time statistics [secs]:\n')
outfile.write('  Min: {:15.6f}\n  Mean: {:14.6f} +/- {:.6f}\n  Median: {:12.6f}\n  Max: {:15.6f}\n\n'.format(
    timediffs.min(), timediffs.mean(), np.std(timediffs), np.median(timediffs), timediffs.max()))

sc = plt.scatter(gttimes - gttimes[0], timediffs, s=0.25, c=timediffs)
plt.xlabel('GT Pose Time [sec]')
plt.ylabel('Max(GT Pose Time - LiDAR Times) [sec]')
plt.title('Max(GT Pose Time - LiDAR Times) vs GT Time')
plt.colorbar(sc)
plt.savefig(os.path.join(savedir, 'max_diff_gttime_lidartime.png'))
plt.close()

mean_scan_times = np.mean(infodf[scan_time_hdrs].to_numpy(), axis=1, keepdims=True)
scan_time_diffs = np.abs(mean_scan_times - infodf[scan_time_hdrs].to_numpy())
timediffs = np.amax(scan_time_diffs, axis=1).flatten()
outfile.write('LiDAR scan time statistics [secs]:\n')
outfile.write('  Min: {:15.6f}\n  Mean: {:14.6f}\n  Median: {:12.6f}\n  Max: {:15.6f}\n\n'.format(
    timediffs.min(), timediffs.mean(), np.median(timediffs), timediffs.max()))

sc = plt.scatter(gttimes - gttimes[0], timediffs, s=0.25, c=timediffs)
plt.xlabel('GT Pose Time [sec]')
plt.ylabel('Max(Mean LiDAR Times - LiDAR Times) [sec]')
plt.title('Max(Mean LiDAR Times - LiDAR Times) vs GT Time')
plt.colorbar(sc)
plt.savefig(os.path.join(savedir, 'max_diff_lidartimes.png'))
plt.close()

outfile.close()
print('Done!')

